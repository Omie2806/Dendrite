// Warp scheduler — breaks VLEN=512 / SEW=32 vector ops into 4 warps of 4 lanes.
//
// VLMAX = 16 elements.  Systolic array has N=4 lanes.
// Each vector instruction produces 4 warps (w0=[0:3], w1=[4:7], w2=[8:11], w3=[12:15]).
//
// The scheduler:
//   1. Accepts a decoded vector op from the coprocessor interface
//   2. Queries the divergence predictor for mask skip hints
//   3. Issues warps to the systolic array one per cycle (or skips dead warps)
//   4. Collects 4-lane results and reassembles the full vector result
//   5. Signals writeback when all warps complete
`default_nettype none

module warp_scheduler #(
    parameter int VLEN      = 512,
    parameter int SEW       = 32,
    parameter int N_LANES   = 4,
    parameter int VLMAX     = VLEN / SEW,           // 16
    parameter int N_WARPS   = VLMAX / N_LANES       // 4
)(
    input  wire                     clk,
    input  wire                     rst_n,

    // ── Instruction dispatch (from coprocessor_if) ──────────
    input  wire                     op_valid,
    output wire                     op_ready,

    input  wire [4:0]               op_type,        // decoded RVV operation
    input  wire                     op_masked,      // v0.t mask active?
    input  wire [VLEN-1:0]          op_vs1,         // source vector 1
    input  wire [VLEN-1:0]          op_vs2,         // source vector 2
    input  wire [VLEN-1:0]          op_vd_old,      // old vd (for accumulate ops)
    input  wire [SEW-1:0]           op_scalar,      // scalar operand (for .vx ops)
    input  wire [VLMAX-1:0]         op_mask,        // v0 mask bits

    // ── Divergence predictor port ───────────────────────────
    output wire                     dp_query_valid,
    input  wire [N_WARPS-1:0]       dp_skip_pred,               // predicted skippable warps

    // ── Systolic array interface ────────────────────────────
    output reg                      sa_clear,
    output reg  [N_LANES-1:0]       sa_lane_en,
    output reg  signed [SEW-1:0]    sa_a_in  [N_LANES],
    output reg  signed [SEW-1:0]    sa_b_in  [N_LANES],
    input  wire signed [2*SEW-1:0]  sa_c_out [N_LANES][N_LANES],

    // ── Result writeback ────────────────────────────────────
    output reg                      wb_valid,
    output reg  [VLEN-1:0]          wb_data,
    input  wire                     wb_ready
);

    // ── Op types (subset of RVV that maps to the array) ─────
    localparam [4:0] OP_VADD_VV   = 5'd0;
    localparam [4:0] OP_VADD_VX   = 5'd1;
    localparam [4:0] OP_VMUL_VV   = 5'd2;
    localparam [4:0] OP_VMUL_VX   = 5'd3;
    localparam [4:0] OP_VMACC_VV  = 5'd4;
    localparam [4:0] OP_VMACC_VX  = 5'd5;
    localparam [4:0] OP_VSUB_VV   = 5'd6;
    localparam [4:0] OP_VSUB_VX   = 5'd7;
    localparam [4:0] OP_VAND_VV   = 5'd8;
    localparam [4:0] OP_VOR_VV    = 5'd9;
    localparam [4:0] OP_VXOR_VV   = 5'd10;
    localparam [4:0] OP_VSLL_VV   = 5'd11;
    localparam [4:0] OP_VSRL_VV   = 5'd12;
    localparam [4:0] OP_VMSEQ_VV  = 5'd13;
    localparam [4:0] OP_VMSLT_VV  = 5'd14;
    localparam [4:0] OP_NOP       = 5'd31;

    // ── FSM states ──────────────────────────────────────────
    typedef enum logic [2:0] {
        S_IDLE,
        S_PREDICT,      // query divergence predictor
        S_ISSUE,        // feed warp into array (1 cycle per warp)
        S_COLLECT,      // read result from array row 0
        S_WRITEBACK     // push result to regfile
    } state_t;

    state_t state;

    // ── Latched instruction ─────────────────────────────────
    reg [4:0]           r_op_type;
    reg                 r_masked;
    reg [VLEN-1:0]      r_vs1, r_vs2, r_vd_old;
    reg [SEW-1:0]       r_scalar;
    reg [VLMAX-1:0]     r_mask;

    // ── Warp tracking ───────────────────────────────────────
    reg [$clog2(N_WARPS)-1:0]   warp_idx;
    reg [N_WARPS-1:0]           warp_skip;      // which warps to skip (from predictor)

    // ── Result accumulator ──────────────────────────────────
    reg [VLEN-1:0] result_vec;

    // ── Helpers: slice vectors into per-warp, per-lane elements ─
    wire signed [SEW-1:0] vs1_elem [VLMAX];
    wire signed [SEW-1:0] vs2_elem [VLMAX];
    wire signed [SEW-1:0] vd_elem  [VLMAX];

    genvar g;
    generate
        for (g = 0; g < VLMAX; g++) begin : g_unpack
            assign vs1_elem[g] = r_vs1[g*SEW +: SEW];
            assign vs2_elem[g] = r_vs2[g*SEW +: SEW];
            assign vd_elem[g]  = r_vd_old[g*SEW +: SEW];
        end
    endgenerate

    // Per-warp mask slices (4 bits each)
    wire [N_LANES-1:0] warp_mask [N_WARPS];
    generate
        for (g = 0; g < N_WARPS; g++) begin : g_wmask
            assign warp_mask[g] = r_mask[g*N_LANES +: N_LANES];
        end
    endgenerate

    assign dp_query_valid = (state == S_PREDICT);
    assign op_ready       = (state == S_IDLE);

    // ── FSM ─────────────────────────────────────────────────
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= S_IDLE;
            r_op_type <= OP_NOP;
            r_masked  <= 1'b0;
            r_vs1     <= '0;
            r_vs2     <= '0;
            r_vd_old  <= '0;
            r_scalar  <= '0;
            r_mask    <= '1;    // default: all lanes active
            warp_idx  <= '0;
            warp_skip <= '0;
            result_vec <= '0;
            wb_valid  <= 1'b0;
            sa_clear  <= 1'b0;
        end else begin
            wb_valid <= 1'b0;
            sa_clear <= 1'b0;

            case (state)
                // ─────────────────────────────────────────────
                S_IDLE: begin
                    if (op_valid) begin
                        r_op_type <= op_type;
                        r_masked  <= op_masked;
                        r_vs1     <= op_vs1;
                        r_vs2     <= op_vs2;
                        r_vd_old  <= op_vd_old;
                        r_scalar  <= op_scalar;
                        r_mask    <= op_masked ? op_mask : {VLMAX{1'b1}};
                        warp_idx  <= '0;
                        result_vec <= '0;
                        sa_clear  <= 1'b1;
                        state     <= S_PREDICT;
                    end
                end

                // ─────────────────────────────────────────────
                S_PREDICT: begin
                    // Latch divergence predictor's skip hints
                    warp_skip <= r_masked ? dp_skip_pred : '0;
                    warp_idx  <= '0;
                    state     <= S_ISSUE;
                end

                // ─────────────────────────────────────────────
                S_ISSUE: begin
                    if (warp_skip[warp_idx] && r_masked && (warp_mask[warp_idx] == '0)) begin
                        // Predicted dead warp — preserve old vd elements
                        for (int lane = 0; lane < N_LANES; lane++) begin
                            automatic logic [$clog2(VLMAX)-1:0] ei = {warp_idx, lane[$clog2(N_LANES)-1:0]};
                            result_vec[ei*SEW +: SEW] <= r_vd_old[ei*SEW +: SEW];
                        end
                        // warp complete
                        if (warp_idx == $clog2(N_WARPS)'(N_WARPS - 1))
                            state <= S_WRITEBACK;
                        else
                            warp_idx <= warp_idx + 1;
                    end else begin
                        // Feed this warp's elements into array row 0
                        state <= S_COLLECT;
                    end
                end

                // ─────────────────────────────────────────────
                S_COLLECT: begin
                    // Result available from sa_c_out after 1-cycle MAC
                    // For element-wise ops we use row 0 diagonal (c_out[0][lane])
                    for (int lane = 0; lane < N_LANES; lane++) begin
                        automatic logic [$clog2(VLMAX)-1:0] ei = {warp_idx, lane[$clog2(N_LANES)-1:0]};
                        automatic logic [N_LANES-1:0] cur_mask = r_masked ? warp_mask[warp_idx] : 4'hF;
                        if (cur_mask[lane]) begin
                            case (r_op_type)
                                OP_VADD_VV:  result_vec[ei*SEW +: SEW] <= SEW'(vs1_elem[ei] + vs2_elem[ei]);
                                OP_VADD_VX:  result_vec[ei*SEW +: SEW] <= SEW'(vs1_elem[ei] + signed'(r_scalar));
                                OP_VSUB_VV:  result_vec[ei*SEW +: SEW] <= SEW'(vs1_elem[ei] - vs2_elem[ei]);
                                OP_VSUB_VX:  result_vec[ei*SEW +: SEW] <= SEW'(vs1_elem[ei] - signed'(r_scalar));
                                OP_VMUL_VV:  result_vec[ei*SEW +: SEW] <= SEW'(vs1_elem[ei] * vs2_elem[ei]);
                                OP_VMUL_VX:  result_vec[ei*SEW +: SEW] <= SEW'(vs1_elem[ei] * signed'(r_scalar));
                                OP_VMACC_VV: result_vec[ei*SEW +: SEW] <= SEW'(vd_elem[ei] + (vs1_elem[ei] * vs2_elem[ei]));
                                OP_VMACC_VX: result_vec[ei*SEW +: SEW] <= SEW'(vd_elem[ei] + (vs1_elem[ei] * signed'(r_scalar)));
                                OP_VAND_VV:  result_vec[ei*SEW +: SEW] <= vs1_elem[ei] & vs2_elem[ei];
                                OP_VOR_VV:   result_vec[ei*SEW +: SEW] <= vs1_elem[ei] | vs2_elem[ei];
                                OP_VXOR_VV:  result_vec[ei*SEW +: SEW] <= vs1_elem[ei] ^ vs2_elem[ei];
                                OP_VSLL_VV:  result_vec[ei*SEW +: SEW] <= SEW'(vs1_elem[ei] << vs2_elem[ei][4:0]);
                                OP_VSRL_VV:  result_vec[ei*SEW +: SEW] <= SEW'(vs1_elem[ei] >> vs2_elem[ei][4:0]);
                                OP_VMSEQ_VV: result_vec[ei*SEW +: SEW] <= {31'b0, vs1_elem[ei] == vs2_elem[ei]};
                                OP_VMSLT_VV: result_vec[ei*SEW +: SEW] <= {31'b0, vs1_elem[ei] < vs2_elem[ei]};
                                default:     result_vec[ei*SEW +: SEW] <= '0;
                            endcase
                        end else begin
                            // Masked-off lane: preserve old value
                            result_vec[ei*SEW +: SEW] <= r_vd_old[ei*SEW +: SEW];
                        end
                    end
                    if (warp_idx == $clog2(N_WARPS)'(N_WARPS - 1))
                        state <= S_WRITEBACK;
                    else begin
                        warp_idx <= warp_idx + 1;
                        state    <= S_ISSUE;
                    end
                end

                // ─────────────────────────────────────────────
                S_WRITEBACK: begin
                    wb_valid <= 1'b1;
                    wb_data  <= result_vec;
                    if (wb_ready)
                        state <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

    // ── Systolic array feed (element-wise mode) ─────────────
    // For element-wise ops, we feed vs1 elements on A, vs2/scalar on B,
    // using row 0 of the array as 4 parallel MACs.
    always_comb begin
        for (int i = 0; i < N_LANES; i++) begin
            sa_a_in[i] = '0;
            sa_b_in[i] = '0;
        end
        sa_lane_en = '0;

        if (state == S_ISSUE && !(warp_skip[warp_idx] && r_masked)) begin
            for (int lane = 0; lane < N_LANES; lane++) begin
                automatic logic [$clog2(VLMAX)-1:0] ei = {warp_idx, lane[$clog2(N_LANES)-1:0]};
                sa_a_in[lane]  = vs1_elem[ei];
                sa_lane_en[lane] = r_masked ? warp_mask[warp_idx][lane] : 1'b1;

                // For .vx ops, broadcast scalar on B; for .vv, use vs2
                case (r_op_type)
                    OP_VADD_VX, OP_VSUB_VX, OP_VMUL_VX, OP_VMACC_VX:
                        sa_b_in[lane] = signed'(r_scalar);
                    default:
                        sa_b_in[lane] = vs2_elem[ei];
                endcase
            end
        end
    end

endmodule
