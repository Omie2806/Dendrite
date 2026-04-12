// Processing Element — INT32 SIMD ALU with systolic pass-through.
// Data flows: A left→right, B top→down.
// All RVV operations execute through op_mode: arithmetic (ADD/SUB/MUL/MAC),
// logic (AND/OR/XOR), shifts (SLL/SRL), and compares (SEQ/SLT).
`default_nettype none

module pe #(
    parameter int DW = 32   // element width
)(
    input  wire             clk,
    input  wire             rst_n,
    input  wire             clear,          // zero the accumulator
    input  wire             en,             // MAC enable (gate for masked-off lanes)
    input  wire [3:0]       op_mode,        // operation mode (MAC / bypass / logic / shift / cmp)
    input  wire             unit_opt_en,    // enable multiply bypass for 0/1/-1 constants

    input  wire signed [DW-1:0] a_in,       // from left neighbour / edge
    input  wire signed [DW-1:0] b_in,       // from top  neighbour / edge

    output reg  signed [DW-1:0] a_out,      // to right neighbour
    output reg  signed [DW-1:0] b_out,      // to bottom neighbour
    output wire signed [2*DW-1:0] c_out     // accumulated result (64-bit to avoid overflow)
);

    reg signed [2*DW-1:0] acc;
    assign c_out = acc;

    localparam logic [3:0] OP_MUL     = 4'd0;
    localparam logic [3:0] OP_PASS_A  = 4'd1;
    localparam logic [3:0] OP_NEG_A   = 4'd2;
    localparam logic [3:0] OP_PASS_B  = 4'd3;
    localparam logic [3:0] OP_AND     = 4'd4;
    localparam logic [3:0] OP_OR      = 4'd5;
    localparam logic [3:0] OP_XOR     = 4'd6;
    localparam logic [3:0] OP_SLL     = 4'd7;
    localparam logic [3:0] OP_SRL     = 4'd8;
    localparam logic [3:0] OP_SEQ     = 4'd9;
    localparam logic [3:0] OP_SLT     = 4'd10;
    localparam logic [3:0] OP_ADD     = 4'd11;
    localparam logic [3:0] OP_SUB     = 4'd12;

    localparam signed [DW-1:0] CONST_ONE     = 1;
    localparam signed [DW-1:0] CONST_NEG_ONE = -1;

    wire signed [2*DW-1:0] a_ext = {{DW{a_in[DW-1]}}, a_in};
    wire signed [2*DW-1:0] b_ext = {{DW{b_in[DW-1]}}, b_in};

    reg signed [2*DW-1:0] mul_term;
    reg signed [2*DW-1:0] acc_term;

    // Bypass trivial multiplier cases when one operand is 0/1/-1.
    always_comb begin
        if (unit_opt_en) begin
            if ((a_in == '0) || (b_in == '0))
                mul_term = '0;
            else if (a_in == CONST_ONE)
                mul_term = b_ext;
            else if (b_in == CONST_ONE)
                mul_term = a_ext;
            else if (a_in == CONST_NEG_ONE)
                mul_term = -b_ext;
            else if (b_in == CONST_NEG_ONE)
                mul_term = -a_ext;
            else
                mul_term = $signed(a_in) * $signed(b_in);
        end else begin
            mul_term = $signed(a_in) * $signed(b_in);
        end
    end

    always_comb begin
        case (op_mode)
            OP_MUL:    acc_term = mul_term;
            OP_PASS_A: acc_term = a_ext;
            OP_NEG_A:  acc_term = -a_ext;
            OP_PASS_B: acc_term = b_ext;
            OP_AND:    acc_term = {{DW{1'b0}}, a_in & b_in};
            OP_OR:     acc_term = {{DW{1'b0}}, a_in | b_in};
            OP_XOR:    acc_term = {{DW{1'b0}}, a_in ^ b_in};
            OP_SLL:    acc_term = {{DW{1'b0}}, a_in << b_in[4:0]};
            OP_SRL:    acc_term = {{DW{1'b0}}, a_in >> b_in[4:0]};
            OP_SEQ:    acc_term = {{(2*DW-1){1'b0}}, a_in == b_in};
            OP_SLT:    acc_term = {{(2*DW-1){1'b0}}, $signed(a_in) < $signed(b_in)};
            OP_ADD:    acc_term = a_ext + b_ext;
            OP_SUB:    acc_term = a_ext - b_ext;
            default:   acc_term = mul_term;
        endcase
    end

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
                acc <= acc + acc_term;
        end
    end

endmodule
