// 4x4 INT32 systolic array — output-stationary dataflow.
//
// Modes:
//   MATMUL  – classical systolic matrix multiply (diagonal-skewed feed)
//   ELEMWISE – use row 0 as 4 independent MAC lanes (for vadd/vmul/vmacc)
//
// The wrapper feeds A from the left edge, B from the top edge.
// After computation, C is read out from each PE's accumulator.
`default_nettype none

module systolic_array #(
    parameter int N  = 4,           // grid dimension
    parameter int DW = 32           // element data width
)(
    input  wire                     clk,
    input  wire                     rst_n,
    input  wire                     clear,      // zero all accumulators
    input  wire [N-1:0]             lane_en,    // per-lane mask (for masked RVV ops)

    // Left-edge inputs  (one per row)
    input  wire signed [DW-1:0]     a_in  [N],
    // Top-edge inputs   (one per col)
    input  wire signed [DW-1:0]     b_in  [N],

    // Accumulator readout (flat: row-major)
    output wire signed [2*DW-1:0]   c_out [N][N]
);

    // Internal wires ────────────────────────────────────────
    wire signed [DW-1:0] a_wire [N][N+1];   // horizontal: [row][col]
    wire signed [DW-1:0] b_wire [N+1][N];   // vertical:   [row][col]

    // Connect edge inputs
    genvar r, c;
    generate
        for (r = 0; r < N; r++) begin : g_edge_a
            assign a_wire[r][0] = a_in[r];
        end
        for (c = 0; c < N; c++) begin : g_edge_b
            assign b_wire[0][c] = b_in[c];
        end
    endgenerate

    // PE grid
    generate
        for (r = 0; r < N; r++) begin : g_row
            for (c = 0; c < N; c++) begin : g_col
                pe #(.DW(DW)) u_pe (
                    .clk    (clk),
                    .rst_n  (rst_n),
                    .clear  (clear),
                    .en     (lane_en[r]),        // row-wise lane mask
                    .a_in   (a_wire[r][c]),
                    .b_in   (b_wire[r][c]),
                    .a_out  (a_wire[r][c+1]),
                    .b_out  (b_wire[r+1][c]),
                    .c_out  (c_out[r][c])
                );
            end
        end
    endgenerate

endmodule
