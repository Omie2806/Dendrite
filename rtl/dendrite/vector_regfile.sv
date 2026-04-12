// Vector register file — 8 registers x VLEN=512 bits.
//
// Each register holds 16 x INT32 elements (SEW=32).
// Two read ports (vs1, vs2) + one write port (vd), all full-width.
// v0 doubles as the mask register (lower 16 bits = lane enables).
`default_nettype none

module vector_regfile #(
    parameter int VLEN   = 512,
    parameter int NREGS  = 8,
    parameter int SEW    = 32,
    parameter int VLMAX  = VLEN / SEW,    // elements per register
    parameter int MASK_BITS = VLMAX       // v0.t mask width (lower bits of v0)
)(
    input  wire                     clk,
    input  wire                     rst_n,

    // Read port 1 (vs1)
    input  wire [$clog2(NREGS)-1:0] rs1_addr,
    output wire [VLEN-1:0]          rs1_data,

    // Read port 2 (vs2)
    input  wire [$clog2(NREGS)-1:0] rs2_addr,
    output wire [VLEN-1:0]          rs2_data,

    // Read port 3 — destination read for accumulate ops (vmacc etc.)
    input  wire [$clog2(NREGS)-1:0] rd_rd_addr,
    output wire [VLEN-1:0]          rd_rd_data,

    // Write port (vd)
    input  wire                     wr_en,
    input  wire [$clog2(NREGS)-1:0] wr_addr,
    input  wire [VLEN-1:0]          wr_data,

    // Mask readout — v0 lower MASK_BITS (lane enables), always available
    output wire [MASK_BITS-1:0]     mask_v0
);

    reg [VLEN-1:0] vreg [NREGS];

    // Combinational reads
    assign rs1_data   = vreg[rs1_addr];
    assign rs2_data   = vreg[rs2_addr];
    assign rd_rd_data = vreg[rd_rd_addr];
    assign mask_v0    = vreg[0][MASK_BITS-1:0];

    // Synchronous write
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int i = 0; i < NREGS; i++)
                vreg[i] <= '0;
        end else if (wr_en) begin
            vreg[wr_addr] <= wr_data;
        end
    end

endmodule
