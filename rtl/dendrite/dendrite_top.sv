// Dendrite — RVV vector coprocessor for CVA6.
//
// VLEN=512, SEW=32, 8 vector registers, 4x4 INT32 systolic array.
// Warp scheduler breaks 16-element vectors into 4 warps of 4 lanes.
// Divergence predictor skips warps with all-masked lanes.
//
//   CVA6 (RV64GC scalar core)
//     │  req/resp handshake
//     ▼
//   coprocessor_if ──► vector_regfile (8 × 512b)
//     │                     │
//     │  op dispatch         │ operand read
//     ▼                     ▼
//   warp_scheduler ◄── divergence_predictor
//     │
//     │  per-warp feed
//     ▼
//   systolic_array (4×4 INT32 PE grid)
`default_nettype none

module dendrite_top #(
    parameter int VLEN     = 512,
    parameter int SEW      = 32,
    parameter int VLMAX    = VLEN / SEW,    // 16
    parameter int NREGS    = 8,
    parameter int N_LANES  = 4,
    parameter int N_WARPS  = VLMAX / N_LANES  // 4
)(
    input  wire         clk,
    input  wire         rst_n,

    // ── CVA6 coprocessor port ───────────────────────────────
    input  wire         req_valid,
    output wire         req_ready,
    input  wire [31:0]  req_instr,
    input  wire [31:0]  req_rs1,

    output wire         resp_valid,
    input  wire         resp_ready,
    output wire [63:0]  resp_data,
    output wire         resp_error
);

    // ════════════════════════════════════════════════════════
    //  Internal wires
    // ════════════════════════════════════════════════════════

    // Coprocessor IF ↔ Vector Register File
    wire [$clog2(NREGS)-1:0] vrf_rs1_addr, vrf_rs2_addr, vrf_rd_rd_addr, vrf_wr_addr;
    wire [VLEN-1:0]          vrf_rs1_data, vrf_rs2_data, vrf_rd_rd_data, vrf_wr_data;
    wire                     vrf_wr_en;
    wire [VLMAX-1:0]         vrf_mask_v0;

    // Coprocessor IF ↔ Warp Scheduler
    wire                     ws_op_valid, ws_op_ready;
    wire [4:0]               ws_op_type;
    wire                     ws_op_masked;
    wire [VLEN-1:0]          ws_op_vs1, ws_op_vs2, ws_op_vd_old;
    wire [SEW-1:0]           ws_op_scalar;
    wire [VLMAX-1:0]         ws_op_mask;
    wire                     ws_wb_valid, ws_wb_ready;
    wire [VLEN-1:0]          ws_wb_data;

    // Warp Scheduler ↔ Systolic Array
    wire                     sa_clear;
    wire [N_LANES-1:0]       sa_lane_en;
    wire signed [SEW-1:0]    sa_a_in  [N_LANES];
    wire signed [SEW-1:0]    sa_b_in  [N_LANES];
    wire signed [2*SEW-1:0]  sa_c_out [N_LANES][N_LANES]; // accumulator readout (future matmul mode)

    // Warp Scheduler ↔ Divergence Predictor
    wire                     dp_query_valid;
    wire [N_WARPS-1:0]       dp_skip_pred;

    // ════════════════════════════════════════════════════════
    //  Divergence predictor feedback — compute actual warp activity
    // ════════════════════════════════════════════════════════
    reg                      dp_update_valid;
    reg [4:0]                dp_update_op_type;
    reg [N_WARPS-1:0]        dp_update_warp_active;

    // Derive actual activity from mask when writeback fires
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dp_update_valid       <= 1'b0;
            dp_update_op_type     <= '0;
            dp_update_warp_active <= '0;
        end else begin
            dp_update_valid <= ws_wb_valid && ws_wb_ready;
            dp_update_op_type <= ws_op_type;
            // A warp was "active" if any lane in its mask slice was set
            // For unmasked ops, all warps are active (don't pollute predictor)
            for (int w = 0; w < N_WARPS; w++)
                dp_update_warp_active[w] <= ws_op_masked ?
                    |ws_op_mask[w*N_LANES +: N_LANES] : 1'b1;
        end
    end

    // ════════════════════════════════════════════════════════
    //  Module instances
    // ════════════════════════════════════════════════════════

    // ── Vector Register File ────────────────────────────────
    vector_regfile #(
        .VLEN  (VLEN),
        .NREGS (NREGS),
        .SEW   (SEW)
    ) u_vrf (
        .clk        (clk),
        .rst_n      (rst_n),
        .rs1_addr   (vrf_rs1_addr),
        .rs1_data   (vrf_rs1_data),
        .rs2_addr   (vrf_rs2_addr),
        .rs2_data   (vrf_rs2_data),
        .rd_rd_addr (vrf_rd_rd_addr),
        .rd_rd_data (vrf_rd_rd_data),
        .wr_en      (vrf_wr_en),
        .wr_addr    (vrf_wr_addr),
        .wr_data    (vrf_wr_data),
        .mask_v0    (vrf_mask_v0)
    );

    // ── Coprocessor Interface ───────────────────────────────
    coprocessor_if #(
        .VLEN    (VLEN),
        .SEW     (SEW),
        .NREGS   (NREGS)
    ) u_copro (
        .clk          (clk),
        .rst_n        (rst_n),
        // CVA6 port
        .req_valid    (req_valid),
        .req_ready    (req_ready),
        .req_instr    (req_instr),
        .req_rs1      (req_rs1),
        .resp_valid   (resp_valid),
        .resp_ready   (resp_ready),
        .resp_data    (resp_data),
        .resp_error   (resp_error),
        // VRF ports
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
        // Warp scheduler
        .ws_op_valid  (ws_op_valid),
        .ws_op_ready  (ws_op_ready),
        .ws_op_type   (ws_op_type),
        .ws_op_masked (ws_op_masked),
        .ws_op_vs1    (ws_op_vs1),
        .ws_op_vs2    (ws_op_vs2),
        .ws_op_vd_old (ws_op_vd_old),
        .ws_op_scalar (ws_op_scalar),
        .ws_op_mask   (ws_op_mask),
        .ws_wb_valid  (ws_wb_valid),
        .ws_wb_ready  (ws_wb_ready),
        .ws_wb_data   (ws_wb_data)
    );

    // ── Warp Scheduler ──────────────────────────────────────
    warp_scheduler #(
        .VLEN    (VLEN),
        .SEW     (SEW),
        .N_LANES (N_LANES)
    ) u_warp_sched (
        .clk          (clk),
        .rst_n        (rst_n),
        // Dispatch
        .op_valid     (ws_op_valid),
        .op_ready     (ws_op_ready),
        .op_type      (ws_op_type),
        .op_masked    (ws_op_masked),
        .op_vs1       (ws_op_vs1),
        .op_vs2       (ws_op_vs2),
        .op_vd_old    (ws_op_vd_old),
        .op_scalar    (ws_op_scalar),
        .op_mask      (ws_op_mask),
        // Divergence predictor
        .dp_query_valid (dp_query_valid),
        .dp_skip_pred   (dp_skip_pred),
        // Systolic array
        .sa_clear     (sa_clear),
        .sa_lane_en   (sa_lane_en),
        .sa_a_in      (sa_a_in),
        .sa_b_in      (sa_b_in),
        .sa_c_out     (sa_c_out),
        // Writeback
        .wb_valid     (ws_wb_valid),
        .wb_data      (ws_wb_data),
        .wb_ready     (ws_wb_ready)
    );

    // ── Divergence Predictor ────────────────────────────────
    divergence_predictor #(
        .N_WARPS (N_WARPS)
    ) u_div_pred (
        .clk                (clk),
        .rst_n              (rst_n),
        .query_valid        (dp_query_valid),
        .query_op_type      (ws_op_type),
        .pred_skip          (dp_skip_pred),
        .update_valid       (dp_update_valid),
        .update_op_type     (dp_update_op_type),
        .update_warp_active (dp_update_warp_active)
    );

    // ── Systolic Array ──────────────────────────────────────
    systolic_array #(
        .N  (N_LANES),
        .DW (SEW)
    ) u_systolic (
        .clk     (clk),
        .rst_n   (rst_n),
        .clear   (sa_clear),
        .lane_en (sa_lane_en),
        .a_in    (sa_a_in),
        .b_in    (sa_b_in),
        .c_out   (sa_c_out)
    );

endmodule
