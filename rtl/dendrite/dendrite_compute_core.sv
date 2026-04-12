// One Dendrite compute tile: banked vector register file + local systolic array.
// Four tiles cover the full VLEN in parallel (one warp per tile) to hide latency.
`default_nettype none

module dendrite_compute_core #(
    parameter int VLEN_BANK = 128,   // VLEN / N_TILES (512/4)
    parameter int SEW       = 32,
    parameter int NREGS     = 8,
    parameter int N_LANES  = 4,
    parameter int MASK_BITS = 16     // architectural v0.t width (bank0 holds packed bits)
)(
    input  wire                     clk,
    input  wire                     rst_n,

    // ── Banked vector register file (same addrs broadcast from IF) ──
    input  wire [$clog2(NREGS)-1:0] vrf_rs1_addr,
    output wire [VLEN_BANK-1:0]     vrf_rs1_data,
    input  wire [$clog2(NREGS)-1:0] vrf_rs2_addr,
    output wire [VLEN_BANK-1:0]     vrf_rs2_data,
    input  wire [$clog2(NREGS)-1:0] vrf_rd_rd_addr,
    output wire [VLEN_BANK-1:0]     vrf_rd_rd_data,
    input  wire                     vrf_wr_en,
    input  wire [$clog2(NREGS)-1:0] vrf_wr_addr,
    input  wire [VLEN_BANK-1:0]     vrf_wr_data,
    output wire [MASK_BITS-1:0]     vrf_mask_v0,

    // ── Local systolic execution fabric ───────────────────────────────
    input  wire                     sa_clear,
    input  wire [N_LANES-1:0]       sa_lane_en,
    input  wire [3:0]               sa_op_mode,
    input  wire                     sa_unit_opt_en,
    input  wire signed [SEW-1:0]    sa_a_in  [N_LANES],
    input  wire signed [SEW-1:0]    sa_b_in  [N_LANES],
    output wire signed [2*SEW-1:0]  sa_c_out [N_LANES][N_LANES]
);

    vector_regfile #(
        .VLEN       (VLEN_BANK),
        .NREGS      (NREGS),
        .SEW        (SEW),
        .MASK_BITS  (MASK_BITS)
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

    systolic_array #(
        .N  (N_LANES),
        .DW (SEW)
    ) u_systolic (
        .clk         (clk),
        .rst_n       (rst_n),
        .clear       (sa_clear),
        .lane_en     (sa_lane_en),
        .op_mode     (sa_op_mode),
        .unit_opt_en (sa_unit_opt_en),
        .a_in        (sa_a_in),
        .b_in        (sa_b_in),
        .c_out       (sa_c_out)
    );

endmodule
