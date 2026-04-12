// Warp scheduler — VLEN/SEW vector ops over N_CORES parallel tiles.
//
// Every RVV operation executes through the systolic array PE lanes.
// The scheduler micro-sequences a diagonal-skewed feed: lane i is fed
// on run_step i, and its diagonal PE[i][i] accumulates on step 2*i
// (accounting for systolic ripple delay).
//
// Divergence predictor is queried in S_POST_DP; pred_skip_in carries
// the latched prediction from the previous op's POST_DP cycle.
`default_nettype none

module warp_scheduler #(
    parameter int VLEN     = 512,
    parameter int SEW      = 32,
    parameter int N_LANES  = 4,
    parameter int VLMAX    = VLEN / SEW,
    parameter int N_WARPS  = VLMAX / N_LANES,
    parameter int N_CORES  = N_WARPS
)(
    input  wire                     clk,
    input  wire                     rst_n,

    input  wire                     op_valid,
    output wire                     op_ready,

    input  wire [4:0]               op_type,
    input  wire                     op_masked,
    input  wire [VLEN-1:0]          op_vs1,
    input  wire [VLEN-1:0]          op_vs2,
    input  wire [VLEN-1:0]          op_vd_old,
    input  wire [SEW-1:0]           op_scalar,
    input  wire [VLMAX-1:0]         op_mask,

    input  wire [N_WARPS-1:0]       pred_skip_in,
    output wire                     dp_query_valid,
    output wire [4:0]               dp_query_op_type,

    output reg  [N_CORES-1:0]                    sa_clear,
    output reg  [N_LANES-1:0]                    sa_lane_en [N_CORES],
    output reg  [3:0]                            sa_op_mode,
    output reg                                   sa_unit_opt_en,
    output reg  signed [SEW-1:0]                 sa_a_in  [N_CORES][N_LANES],
    output reg  signed [SEW-1:0]                 sa_b_in  [N_CORES][N_LANES],
    input  wire signed [2*SEW-1:0]               sa_c_out [N_CORES][N_LANES][N_LANES],

    output reg                      wb_valid,
    output reg  [VLEN-1:0]          wb_data,
    input  wire                     wb_ready
);

    // ── Internal op codes ───────────────────────────────────
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

    // ── PE op_mode codes (must match pe.sv) ─────────────────
    localparam [3:0] SA_OP_MUL    = 4'd0;
    localparam [3:0] SA_OP_PASS_A = 4'd1;
    localparam [3:0] SA_OP_AND    = 4'd4;
    localparam [3:0] SA_OP_OR     = 4'd5;
    localparam [3:0] SA_OP_XOR    = 4'd6;
    localparam [3:0] SA_OP_SLL    = 4'd7;
    localparam [3:0] SA_OP_SRL    = 4'd8;
    localparam [3:0] SA_OP_SEQ    = 4'd9;
    localparam [3:0] SA_OP_SLT    = 4'd10;
    localparam [3:0] SA_OP_ADD    = 4'd11;
    localparam [3:0] SA_OP_SUB    = 4'd12;

    // ── FSM ─────────────────────────────────────────────────
    typedef enum logic [3:0] {
        S_IDLE,
        S_PREDICT,
        S_ISSUE,
        S_ARRAY_RUN,
        S_ARRAY_WAIT,
        S_COLLECT,
        S_POST_DP,
        S_WRITEBACK
    } state_t;

    state_t state;

    // ── Latched operands ────────────────────────────────────
    reg [4:0]           r_op_type;
    reg                 r_masked;
    reg [VLEN-1:0]      r_vs1, r_vs2, r_vd_old;
    reg [SEW-1:0]       r_scalar;
    reg [VLMAX-1:0]     r_mask;

    reg [N_WARPS-1:0]   warp_skip;

    // ── Array run counters ──────────────────────────────────
    reg [1:0]           phase_idx;
    localparam int RUN_STEPS = (2 * N_LANES) - 1;
    localparam int RUN_W = $clog2(RUN_STEPS + 1);
    localparam int LANE_W = $clog2(N_LANES);
    reg [RUN_W-1:0]     run_step;

    reg [VLEN-1:0]      result_vec;

    // ── Element unpacking ───────────────────────────────────
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

    wire [N_LANES-1:0] warp_mask [N_WARPS];
    generate
        for (g = 0; g < N_WARPS; g++) begin : g_wmask
            assign warp_mask[g] = r_mask[g*N_LANES +: N_LANES];
        end
    endgenerate

    // ── Combinational outputs ───────────────────────────────
    assign dp_query_valid   = (state == S_POST_DP);
    assign dp_query_op_type = r_op_type;
    assign op_ready         = (state == S_IDLE);

    // ── Helper: is this warp predicted-dead and confirmed by mask? ─
    function automatic logic skipped_dead(input logic [$clog2(N_WARPS)-1:0] c);
        return warp_skip[c] && r_masked && (warp_mask[c] == '0);
    endfunction

    // ── Helper: how many array phases does this op need? ────
    function automatic logic [1:0] op_phase_count(input logic [4:0] op);
        case (op)
            OP_VMACC_VV, OP_VMACC_VX: return 2'd2;   // MUL then PASS_A
            default:                   return 2'd1;    // everything else: single phase
        endcase
    endfunction

    // ── Helper: map (op_type, phase_idx) → PE op_mode ───────
    function automatic logic [3:0] get_sa_op_mode(input logic [4:0] op, input logic [1:0] phase);
        case (op)
            OP_VADD_VV, OP_VADD_VX:   return SA_OP_ADD;
            OP_VSUB_VV, OP_VSUB_VX:   return SA_OP_SUB;
            OP_VMUL_VV, OP_VMUL_VX:   return SA_OP_MUL;
            OP_VMACC_VV, OP_VMACC_VX: return (phase == '0) ? SA_OP_MUL : SA_OP_PASS_A;
            OP_VAND_VV:               return SA_OP_AND;
            OP_VOR_VV:                return SA_OP_OR;
            OP_VXOR_VV:               return SA_OP_XOR;
            OP_VSLL_VV:               return SA_OP_SLL;
            OP_VSRL_VV:               return SA_OP_SRL;
            OP_VMSEQ_VV:              return SA_OP_SEQ;
            OP_VMSLT_VV:              return SA_OP_SLT;
            default:                  return SA_OP_MUL;
        endcase
    endfunction

    // ── FSM ─────────────────────────────────────────────────
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state      <= S_IDLE;
            r_op_type  <= OP_NOP;
            r_masked   <= 1'b0;
            r_vs1      <= '0;
            r_vs2      <= '0;
            r_vd_old   <= '0;
            r_scalar   <= '0;
            r_mask     <= '1;
            warp_skip  <= '0;
            phase_idx  <= '0;
            run_step   <= '0;
            result_vec <= '0;
            wb_valid   <= 1'b0;
            sa_clear   <= '0;
        end else begin
            wb_valid <= 1'b0;
            sa_clear <= '0;

            case (state)
                S_IDLE: begin
                    if (op_valid) begin
                        r_op_type  <= op_type;
                        r_masked   <= op_masked;
                        r_vs1      <= op_vs1;
                        r_vs2      <= op_vs2;
                        r_vd_old   <= op_vd_old;
                        r_scalar   <= op_scalar;
                        r_mask     <= op_masked ? op_mask : {VLMAX{1'b1}};
                        phase_idx  <= '0;
                        run_step   <= '0;
                        result_vec <= '0;
                        sa_clear   <= {N_CORES{1'b1}};
                        state      <= S_PREDICT;
                    end
                end

                S_PREDICT: begin
                    warp_skip <= r_masked ? pred_skip_in : '0;
                    state     <= S_ISSUE;
                end

                S_ISSUE: begin
                    // Copy vd_old for skipped warps
                    for (int c = 0; c < N_CORES; c++) begin
                        if (skipped_dead($clog2(N_WARPS)'(c))) begin
                            for (int lane = 0; lane < N_LANES; lane++) begin
                                automatic logic [$clog2(VLMAX)-1:0] ei =
                                    {$clog2(N_WARPS)'(c), lane[LANE_W-1:0]};
                                result_vec[ei*SEW +: SEW] <= r_vd_old[ei*SEW +: SEW];
                            end
                        end
                    end
                    // SA was already cleared in S_IDLE; by now the clear has
                    // propagated through the PE registers and sa_clear is 0.
                    phase_idx <= '0;
                    run_step  <= '0;
                    state     <= S_ARRAY_RUN;
                end

                S_ARRAY_RUN: begin
                    if (run_step == RUN_W'(RUN_STEPS - 1)) begin
                        run_step <= '0;
                        if (phase_idx + 1'b1 >= op_phase_count(r_op_type))
                            state <= S_ARRAY_WAIT;
                        else
                            phase_idx <= phase_idx + 1'b1;
                    end else begin
                        run_step <= run_step + 1'b1;
                    end
                end

                S_ARRAY_WAIT: begin
                    state <= S_COLLECT;
                end

                S_COLLECT: begin
                    for (int c = 0; c < N_CORES; c++) begin
                        for (int lane = 0; lane < N_LANES; lane++) begin
                            automatic logic [$clog2(VLMAX)-1:0] ei =
                                {$clog2(N_WARPS)'(c), lane[LANE_W-1:0]};
                            automatic logic [N_LANES-1:0] cur_mask =
                                r_masked ? warp_mask[$clog2(N_WARPS)'(c)] : {N_LANES{1'b1}};
                            if (skipped_dead($clog2(N_WARPS)'(c))) begin
                                // already filled in S_ISSUE
                            end else if (cur_mask[lane]) begin
                                result_vec[ei*SEW +: SEW] <= SEW'(sa_c_out[c][lane][lane]);
                            end else begin
                                result_vec[ei*SEW +: SEW] <= r_vd_old[ei*SEW +: SEW];
                            end
                        end
                    end
                    state <= S_POST_DP;
                end

                S_POST_DP: begin
                    state <= S_WRITEBACK;
                end

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

    // ── Systolic array feed (combinational) ─────────────────
    // Diagonal-skewed feed: lane i gets edge data on run_step i.
    // Due to systolic ripple, diagonal PE[i][i] sees it on step 2*i.
    // lane_en[i] is pulsed on step 2*i so each diagonal PE accumulates
    // exactly once per phase.
    always_comb begin
        // Defaults: quiescent
        for (int c = 0; c < N_CORES; c++) begin
            for (int i = 0; i < N_LANES; i++) begin
                sa_a_in[c][i] = '0;
                sa_b_in[c][i] = '0;
            end
            sa_lane_en[c] = '0;
        end
        sa_op_mode     = SA_OP_MUL;
        sa_unit_opt_en = 1'b1;

        if (state == S_ARRAY_RUN) begin
            sa_op_mode = get_sa_op_mode(r_op_type, phase_idx);

            for (int c = 0; c < N_CORES; c++) begin
                if (skipped_dead($clog2(N_WARPS)'(c))) begin
                    // quiescent tile — skip
                end else begin
                    // Enable row i on step 2*i (when diagonal PE[i][i] has valid data)
                    for (int lane = 0; lane < N_LANES; lane++) begin
                        if (run_step == RUN_W'(2 * lane))
                            sa_lane_en[c][lane] = r_masked ?
                                warp_mask[$clog2(N_WARPS)'(c)][lane] : 1'b1;
                    end

                    // Feed edge inputs: lane i gets data on step i
                    for (int lane = 0; lane < N_LANES; lane++) begin
                        if (run_step == RUN_W'(lane)) begin
                            automatic logic [$clog2(VLMAX)-1:0] ei =
                                {$clog2(N_WARPS)'(c), lane[LANE_W-1:0]};

                            // A operand
                            if (r_op_type == OP_VMACC_VV || r_op_type == OP_VMACC_VX) begin
                                sa_a_in[c][lane] = (phase_idx == '0) ? vs1_elem[ei] : vd_elem[ei];
                            end else begin
                                sa_a_in[c][lane] = vs1_elem[ei];
                            end

                            // B operand
                            case (r_op_type)
                                OP_VADD_VX, OP_VSUB_VX, OP_VMUL_VX:
                                    sa_b_in[c][lane] = signed'(r_scalar);
                                OP_VMACC_VX:
                                    sa_b_in[c][lane] = (phase_idx == '0) ? signed'(r_scalar) : '0;
                                OP_VMACC_VV:
                                    sa_b_in[c][lane] = (phase_idx == '0) ? vs2_elem[ei] : '0;
                                default:
                                    sa_b_in[c][lane] = vs2_elem[ei];
                            endcase
                        end
                    end
                end
            end
        end
    end

endmodule
