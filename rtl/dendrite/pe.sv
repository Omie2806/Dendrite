// Processing Element — INT32 multiply-accumulate with systolic pass-through.
// Data flows: A left→right, B top→down.  Each PE accumulates C += A*B.
`default_nettype none

module pe #(
    parameter int DW = 32   // element width
)(
    input  wire             clk,
    input  wire             rst_n,
    input  wire             clear,          // zero the accumulator
    input  wire             en,             // MAC enable (gate for masked-off lanes)

    input  wire signed [DW-1:0] a_in,       // from left neighbour / edge
    input  wire signed [DW-1:0] b_in,       // from top  neighbour / edge

    output reg  signed [DW-1:0] a_out,      // to right neighbour
    output reg  signed [DW-1:0] b_out,      // to bottom neighbour
    output wire signed [2*DW-1:0] c_out     // accumulated result (64-bit to avoid overflow)
);

    reg signed [2*DW-1:0] acc;
    assign c_out = acc;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            acc   <= '0;
            a_out <= '0;
            b_out <= '0;
        end else if (clear) begin
            acc   <= '0;
            a_out <= '0;
            b_out <= '0;
        end else begin
            a_out <= a_in;
            b_out <= b_in;
            if (en)
                acc <= acc + (64'(a_in) * 64'(b_in));
        end
    end

endmodule
