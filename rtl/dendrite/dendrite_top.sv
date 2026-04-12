// Dendrite — RVV vector coprocessor for CVA6.
//
// Four compute tiles (banked VRF + systolic each) run one warp in parallel.
// Divergence predictor sits after the tile array on the telemetry path; the
// scheduler consumes pred_skip_in latched when the prior op completed POST_DP.
`default_nettype none

module dendrite_top #(
    parameter int VLEN     = 512,
    parameter int SEW      = 32,
    parameter int VLMAX    = VLEN / SEW,
    parameter int NREGS    = 8,
    parameter int N_LANES  = 4,
    parameter int N_WARPS  = VLMAX / N_LANES,
    parameter int N_CORES  = N_WARPS,
    parameter int VLEN_BANK = VLEN / N_CORES,
    parameter int MASK_BITS = VLMAX
)(
    input  wire         clk,
    input  wire         rst_n,

    input  wire         req_valid,
    output wire         req_ready,
    input  wire [31:0]  req_instr,
    input  wire [31:0]  req_rs1,

    output wire         resp_valid,
    input  wire         resp_ready,
    output wire [63:0]  resp_data,
    output wire         resp_error
);

    wire [$clog2(NREGS)-1:0] vrf_rs1_addr, vrf_rs2_addr, vrf_rd_rd_addr, vrf_wr_addr;
    wire [VLEN-1:0]         vrf_rs1_data, vrf_rs2_data, vrf_rd_rd_data, vrf_wr_data;
    wire                    vrf_wr_en;
    wire [MASK_BITS-1:0]    vrf_mask_v0;

    wire                    ws_op_valid, ws_op_ready;
    wire [4:0]              ws_op_type;
    wire                    ws_op_masked;
    wire [VLEN-1:0]         ws_op_vs1, ws_op_vs2, ws_op_vd_old;
    wire [SEW-1:0]          ws_op_scalar;
    wire [VLMAX-1:0]       ws_op_mask;
    wire                    ws_wb_valid, ws_wb_ready;
    wire [VLEN-1:0]         ws_wb_data;

    wire [N_CORES-1:0]                    sa_clear;
    wire [N_LANES-1:0]                    sa_lane_en [N_CORES];
    wire [3:0]                            sa_op_mode;
    wire                                  sa_unit_opt_en;
    wire signed [SEW-1:0]                 sa_a_in  [N_CORES][N_LANES];
    wire signed [SEW-1:0]                 sa_b_in  [N_CORES][N_LANES];
    wire signed [2*SEW-1:0]               sa_c_out [N_CORES][N_LANES][N_LANES];

    wire                    dp_query_valid;
    wire [4:0]              dp_query_op_type;
    wire [N_WARPS-1:0]      dp_skip_pred;
    reg  [N_WARPS-1:0]      pred_skip_q;

    wire [VLEN_BANK-1:0]    bank_rs1 [N_CORES];
    wire [VLEN_BANK-1:0]    bank_rs2 [N_CORES];
    wire [VLEN_BANK-1:0]    bank_rd  [N_CORES];
    wire [MASK_BITS-1:0]    bank_mask [N_CORES];

    genvar gi;

    // Banked read concat: element order 0..15 → core0 (0-3), core1 (4-7), ...
    assign vrf_rs1_data = {bank_rs1[3], bank_rs1[2], bank_rs1[1], bank_rs1[0]};
    assign vrf_rs2_data = {bank_rs2[3], bank_rs2[2], bank_rs2[1], bank_rs2[0]};
    assign vrf_rd_rd_data = {bank_rd[3], bank_rd[2], bank_rd[1], bank_rd[0]};

    assign vrf_mask_v0 = bank_mask[0];

    // Latched skip hints for the *next* vector op (queried after systolic work).
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            pred_skip_q <= '0;
        else if (dp_query_valid)
            pred_skip_q <= dp_skip_pred;
    end

    reg                      dp_update_valid;
    reg [4:0]                dp_update_op_type;
    reg [N_WARPS-1:0]        dp_update_warp_active;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dp_update_valid       <= 1'b0;
            dp_update_op_type     <= '0;
            dp_update_warp_active <= '0;
        end else begin
            dp_update_valid <= ws_wb_valid && ws_wb_ready;
            dp_update_op_type <= ws_op_type;
            for (int w = 0; w < N_WARPS; w++)
                dp_update_warp_active[w] <= ws_op_masked ?
                    |ws_op_mask[w*N_LANES +: N_LANES] : 1'b1;
        end
    end

    // ── Warp scheduler (feeds all tiles in parallel) ─────────
    warp_scheduler #(
        .VLEN    (VLEN),
        .SEW     (SEW),
        .N_LANES (N_LANES),
        .N_CORES (N_CORES)
    ) u_warp_sched (
        .clk            (clk),
        .rst_n          (rst_n),
        .op_valid       (ws_op_valid),
        .op_ready       (ws_op_ready),
        .op_type        (ws_op_type),
        .op_masked      (ws_op_masked),
        .op_vs1         (ws_op_vs1),
        .op_vs2         (ws_op_vs2),
        .op_vd_old      (ws_op_vd_old),
        .op_scalar      (ws_op_scalar),
        .op_mask        (ws_op_mask),
        .pred_skip_in   (pred_skip_q),
        .dp_query_valid (dp_query_valid),
        .dp_query_op_type (dp_query_op_type),
        .sa_clear       (sa_clear),
        .sa_lane_en     (sa_lane_en),
        .sa_op_mode     (sa_op_mode),
        .sa_unit_opt_en (sa_unit_opt_en),
        .sa_a_in        (sa_a_in),
        .sa_b_in        (sa_b_in),
        .sa_c_out       (sa_c_out),
        .wb_valid       (ws_wb_valid),
        .wb_data        (ws_wb_data),
        .wb_ready       (ws_wb_ready)
    );

    // ── Four compute tiles (VRF bank + systolic) ─────────────
    generate
        for (gi = 0; gi < N_CORES; gi++) begin : g_core
            dendrite_compute_core #(
                .VLEN_BANK (VLEN_BANK),
                .SEW       (SEW),
                .NREGS     (NREGS),
                .N_LANES   (N_LANES),
                .MASK_BITS (MASK_BITS)
            ) u_core (
                .clk            (clk),
                .rst_n          (rst_n),
                .vrf_rs1_addr   (vrf_rs1_addr),
                .vrf_rs1_data   (bank_rs1[gi]),
                .vrf_rs2_addr   (vrf_rs2_addr),
                .vrf_rs2_data   (bank_rs2[gi]),
                .vrf_rd_rd_addr (vrf_rd_rd_addr),
                .vrf_rd_rd_data (bank_rd[gi]),
                .vrf_wr_en      (vrf_wr_en),
                .vrf_wr_addr    (vrf_wr_addr),
                .vrf_wr_data    (vrf_wr_data[gi*VLEN_BANK +: VLEN_BANK]),
                .vrf_mask_v0    (bank_mask[gi]),
                .sa_clear       (sa_clear[gi]),
                .sa_lane_en     (sa_lane_en[gi]),
                .sa_op_mode     (sa_op_mode),
                .sa_unit_opt_en (sa_unit_opt_en),
                .sa_a_in        (sa_a_in[gi]),
                .sa_b_in        (sa_b_in[gi]),
                .sa_c_out       (sa_c_out[gi])
            );
        end
    endgenerate

    // ── Divergence predictor (after compute tiles on the datapath) ──
    divergence_predictor #(
        .N_WARPS (N_WARPS)
    ) u_div_pred (
        .clk                (clk),
        .rst_n              (rst_n),
        .query_valid        (dp_query_valid),
        .query_op_type      (dp_query_op_type),
        .pred_skip          (dp_skip_pred),
        .update_valid       (dp_update_valid),
        .update_op_type     (dp_update_op_type),
        .update_warp_active (dp_update_warp_active)
    );

    coprocessor_if #(
        .VLEN    (VLEN),
        .SEW     (SEW),
        .NREGS   (NREGS)
    ) u_copro (
        .clk            (clk),
        .rst_n          (rst_n),
        .req_valid      (req_valid),
        .req_ready      (req_ready),
        .req_instr      (req_instr),
        .req_rs1        (req_rs1),
        .resp_valid     (resp_valid),
        .resp_ready     (resp_ready),
        .resp_data      (resp_data),
        .resp_error     (resp_error),
        .vrf_rs1_addr   (vrf_rs1_addr),
        .vrf_rs1_data   (vrf_rs1_data),
        .vrf_rs2_addr   (vrf_rs2_addr),
        .vrf_rs2_data   (vrf_rs2_data),
        .vrf_rd_rd_addr (vrf_rd_rd_addr),
        .vrf_rd_rd_data (vrf_rd_rd_data),
        .vrf_wr_en      (vrf_wr_en),
        .vrf_wr_addr    (vrf_wr_addr),
        .vrf_wr_data    (vrf_wr_data),
        .vrf_mask_v0    (vrf_mask_v0),
        .ws_op_valid    (ws_op_valid),
        .ws_op_ready    (ws_op_ready),
        .ws_op_type     (ws_op_type),
        .ws_op_masked   (ws_op_masked),
        .ws_op_vs1      (ws_op_vs1),
        .ws_op_vs2      (ws_op_vs2),
        .ws_op_vd_old   (ws_op_vd_old),
        .ws_op_scalar   (ws_op_scalar),
        .ws_op_mask     (ws_op_mask),
        .ws_wb_valid    (ws_wb_valid),
        .ws_wb_ready    (ws_wb_ready),
        .ws_wb_data     (ws_wb_data)
    );

endmodule
