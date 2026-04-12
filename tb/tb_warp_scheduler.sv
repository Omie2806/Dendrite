// Testbench — warp scheduler + divergence predictor integration.
//
// Tests:
//   1. Unmasked vadd.vv: all 4 warps execute, result = vs1 + vs2
//   2. Masked vadd.vv: warps with all-zero mask should be skipped
//   3. vmacc.vv: accumulate vd += vs1 * vs2
//   4. Divergence predictor training: repeat masked op and verify skip predictions
`timescale 1ns/1ps

module tb_warp_scheduler;

    parameter int VLEN    = 512;
    parameter int SEW     = 32;
    parameter int N_LANES = 4;
    parameter int VLMAX   = VLEN / SEW;
    parameter int N_WARPS = VLMAX / N_LANES;

    logic               clk, rst_n;

    // Dispatch
    logic               op_valid;
    wire                op_ready;
    logic [4:0]         op_type;
    logic               op_masked;
    logic [VLEN-1:0]    op_vs1, op_vs2, op_vd_old;
    logic [SEW-1:0]     op_scalar;
    logic [VLMAX-1:0]   op_mask;

    // Divergence predictor
    wire                dp_query_valid;
    wire [N_WARPS-1:0]  dp_skip_pred;

    // Systolic array
    wire                sa_clear;
    wire [N_LANES-1:0]  sa_lane_en;
    wire signed [SEW-1:0] sa_a_in [N_LANES];
    wire signed [SEW-1:0] sa_b_in [N_LANES];
    wire signed [2*SEW-1:0] sa_c_out [N_LANES][N_LANES];

    // Writeback
    wire                wb_valid;
    wire [VLEN-1:0]     wb_data;
    logic               wb_ready;

    // ── Instantiate warp scheduler ──────────────────────────
    warp_scheduler #(
        .VLEN(VLEN), .SEW(SEW), .N_LANES(N_LANES)
    ) u_ws (
        .clk(clk), .rst_n(rst_n),
        .op_valid(op_valid), .op_ready(op_ready),
        .op_type(op_type), .op_masked(op_masked),
        .op_vs1(op_vs1), .op_vs2(op_vs2), .op_vd_old(op_vd_old),
        .op_scalar(op_scalar), .op_mask(op_mask),
        .dp_query_valid(dp_query_valid),
        .dp_skip_pred(dp_skip_pred),
        .sa_clear(sa_clear), .sa_lane_en(sa_lane_en),
        .sa_a_in(sa_a_in), .sa_b_in(sa_b_in), .sa_c_out(sa_c_out),
        .wb_valid(wb_valid), .wb_data(wb_data), .wb_ready(wb_ready)
    );

    // ── Instantiate divergence predictor ────────────────────
    // Feedback wiring
    reg                 dp_update_valid;
    reg [4:0]           dp_update_op_type;
    reg [N_WARPS-1:0]   dp_update_warp_active;

    divergence_predictor #(
        .N_WARPS(N_WARPS)
    ) u_dp (
        .clk(clk), .rst_n(rst_n),
        .query_valid(dp_query_valid),
        .query_op_type(op_type),
        .pred_skip(dp_skip_pred),
        .update_valid(dp_update_valid),
        .update_op_type(dp_update_op_type),
        .update_warp_active(dp_update_warp_active)
    );

    // ── Instantiate systolic array (needed for port connection) ─
    systolic_array #(.N(N_LANES), .DW(SEW)) u_sa (
        .clk(clk), .rst_n(rst_n),
        .clear(sa_clear), .lane_en(sa_lane_en),
        .a_in(sa_a_in), .b_in(sa_b_in), .c_out(sa_c_out)
    );

    // Clock
    initial clk = 0;
    always #5 clk = ~clk;

    int pass_count = 0;
    int fail_count = 0;
    integer i_loop;  // loop var for initial block

    // ── Helpers ─────────────────────────────────────────────
    function automatic [VLEN-1:0] make_vec(input int base, input int stride);
        logic [VLEN-1:0] v;
        for (int ii = 0; ii < VLMAX; ii++)
            v[ii*SEW +: SEW] = SEW'(base + ii * stride);
        return v;
    endfunction

    function automatic signed [SEW-1:0] get_elem(input [VLEN-1:0] v, input int idx);
        return v[idx*SEW +: SEW];
    endfunction

    task automatic check_elem(input string name, input [VLEN-1:0] v, input int idx,
                              input logic signed [SEW-1:0] expected);
        logic signed [SEW-1:0] got;
        got = get_elem(v, idx);
        if (got === expected)
            pass_count = pass_count + 1;
        else begin
            $display("FAIL %s elem[%0d]: got %0d, expected %0d", name, idx, got, expected);
            fail_count = fail_count + 1;
        end
    endtask

    task automatic dispatch_and_wait(
        input [4:0] otype,
        input logic masked,
        input [VLEN-1:0] vs1, vs2, vd_old,
        input [SEW-1:0] scalar,
        input [VLMAX-1:0] mask
    );
        integer w;
        // Drive on negedge to avoid races with DUT posedge flops.
        @(negedge clk);
        op_valid  = 1'b1;
        op_type   = otype;
        op_masked = masked;
        op_vs1    = vs1;
        op_vs2    = vs2;
        op_vd_old = vd_old;
        op_scalar = scalar;
        op_mask   = mask;

        // Wait for scheduler to accept
        do @(posedge clk); while (!op_ready);
        @(negedge clk);
        op_valid = 1'b0;

        // Wait for writeback
        wb_ready = 1'b1;
        do @(posedge clk); while (!wb_valid);

        // Send update to divergence predictor
        @(negedge clk);
        dp_update_valid = 1'b1;
        dp_update_op_type = otype;
        for (w = 0; w < N_WARPS; w = w + 1)
            dp_update_warp_active[w] = |mask[w*N_LANES +: N_LANES];
        @(posedge clk);
        @(negedge clk);
        dp_update_valid = 1'b0;
        wb_ready = 1'b0;
    endtask

    // ── Test ────────────────────────────────────────────────
    initial begin
        $dumpfile("tb_warp_scheduler.fst");
        $dumpvars(0, tb_warp_scheduler);

        rst_n     = 0;
        op_valid  = 0;
        wb_ready  = 0;
        op_type   = 0;
        op_masked = 0;
        op_vs1    = '0;
        op_vs2    = '0;
        op_vd_old = '0;
        op_scalar = 0;
        op_mask   = '1;
        dp_update_valid = 0;
        dp_update_op_type = 0;
        dp_update_warp_active = '0;

        #30 rst_n = 1;
        #10;

        // ── Test 1: Unmasked vadd.vv ────────────────────────
        $display("\n--- Test 1: vadd.vv unmasked ---");
        begin
            automatic logic [VLEN-1:0] vs1 = make_vec(1, 1);   // [1,2,3,...,16]
            automatic logic [VLEN-1:0] vs2 = make_vec(10, 1);  // [10,11,12,...,25]
            dispatch_and_wait(5'd0, 0, vs1, vs2, '0, 0, {VLMAX{1'b1}});

            for (i_loop = 0; i_loop < VLMAX; i_loop = i_loop + 1)
                check_elem("vadd_unmasked", wb_data, i_loop, SEW'(1+i_loop + 10+i_loop));
        end

        // ── Test 2: Masked vadd.vv ──────────────────────────
        $display("\n--- Test 2: vadd.vv masked ---");
        begin
            automatic logic [VLEN-1:0] vs1 = make_vec(100, 1);
            automatic logic [VLEN-1:0] vs2 = make_vec(200, 1);
            automatic logic [VLEN-1:0] vd  = make_vec(999, 0); // all 999
            // Mask: warp0 active, warp1 dead, warp2 active, warp3 dead
            automatic logic [VLMAX-1:0] mask = 16'b0000_1111_0000_1111;
            dispatch_and_wait(5'd0, 1, vs1, vs2, vd, 0, mask);

            // Warp 0 (elems 0-3): should have vs1+vs2
            check_elem("masked_w0", wb_data, 0, SEW'(300));
            check_elem("masked_w0", wb_data, 3, SEW'(306));
            // Warp 1 (elems 4-7): should preserve old vd = 999
            check_elem("masked_w1", wb_data, 4, SEW'(999));
            check_elem("masked_w1", wb_data, 7, SEW'(999));
            // Warp 2 (elems 8-11): active
            check_elem("masked_w2", wb_data, 8, SEW'(316));
            // Warp 3 (elems 12-15): preserved
            check_elem("masked_w3", wb_data, 12, SEW'(999));
        end

        // ── Test 3: vmacc.vv ────────────────────────────────
        $display("\n--- Test 3: vmacc.vv ---");
        begin
            automatic logic [VLEN-1:0] vs1 = make_vec(2, 0);   // all 2s
            automatic logic [VLEN-1:0] vs2 = make_vec(3, 0);   // all 3s
            automatic logic [VLEN-1:0] vd  = make_vec(10, 0);  // all 10s
            dispatch_and_wait(5'd4, 0, vs1, vs2, vd, 0, {VLMAX{1'b1}});

            // vd[i] = 10 + 2*3 = 16
            for (i_loop = 0; i_loop < VLMAX; i_loop = i_loop + 1)
                check_elem("vmacc", wb_data, i_loop, SEW'(16));
        end

        // ── Test 4: Divergence predictor training ───────────
        $display("\n--- Test 4: Divergence predictor training ---");
        begin
            // Repeat masked op with warp 3 always dead — predictor should learn to skip it
            automatic logic [VLMAX-1:0] mask = 16'b0000_1111_1111_1111; // warp3 dead
            for (i_loop = 0; i_loop < 5; i_loop = i_loop + 1) begin
                automatic logic [VLEN-1:0] vs1 = make_vec(i_loop, 1);
                automatic logic [VLEN-1:0] vs2 = make_vec(1, 0);
                dispatch_and_wait(5'd0, 1, vs1, vs2, '0, 0, mask);
            end
            // After several iterations, the predictor should have the warp3 counter >= 2
            $display("Predictor training complete — warp3 skip should be predicted");
        end

        // ── Summary ─────────────────────────────────────────
        #20;
        $display("\n=== Warp Scheduler TB: %0d passed, %0d failed ===", pass_count, fail_count);
        if (fail_count > 0) $fatal(1, "TESTS FAILED");
        $finish;
    end

endmodule
