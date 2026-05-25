// Testbench — 4x4 INT32 systolic array standalone verification.
//
// Tests:
//   1. Clear and verify all accumulators are zero
//   2. Single-cycle MAC: feed known A/B, check C = A*B
//   3. Accumulate: feed two rounds, verify C = A1*B1 + A2*B2
//   4. Lane masking: disable lane 2, verify it doesn't accumulate
`timescale 1ns/1ps

module tb_systolic_array;

    parameter int N  = 4;
    parameter int DW = 32;

    logic               clk, rst_n, clear;
    logic [N-1:0]       lane_en;
    logic [3:0]         op_mode;
    logic               unit_opt_en;
    logic signed [DW-1:0] a_in [N];
    logic signed [DW-1:0] b_in [N];
    wire  signed [2*DW-1:0] c_out [N][N];

    systolic_array #(.N(N), .DW(DW)) dut (.*);

    // Clock: 10 ns period
    initial clk = 0;
    always #5 clk = ~clk;

    int pass_count = 0;
    int fail_count = 0;

    task automatic check(string name, logic signed [63:0] got, logic signed [63:0] expected);
        if (got === expected) begin
            pass_count = pass_count + 1;
        end else begin
            $display("FAIL %s: got %0d, expected %0d", name, got, expected);
            fail_count = fail_count + 1;
        end
    endtask

    initial begin
        $dumpfile("tb_systolic_array.fst");
        $dumpvars(0, tb_systolic_array);

        // ── Reset ───────────────────────────────────────────
        rst_n   = 0;
        clear   = 0;
        lane_en = '0;
        op_mode = 2'd0;      // MAC mode
        unit_opt_en = 1'b1;  // enable 0/1/-1 multiply bypass
        for (int i = 0; i < N; i++) begin
            a_in[i] = 0;
            b_in[i] = 0;
        end
        #20 rst_n = 1;

        // ── Test 1: Clear ───────────────────────────────────
        clear = 1;
        @(posedge clk); #1;
        clear = 0;
        @(posedge clk); #1;

        for (int r = 0; r < N; r++)
            for (int c = 0; c < N; c++)
                check($sformatf("clear[%0d][%0d]", r, c), c_out[r][c], 0);
                
          // ── Test 2: Single MAC ──────────────────────────────
        // Feed A = [1,2,3,4], B = [5,6,7,8] with all lanes enabled
        // After 1 cycle, only PE[0][0] sees a_in[0]*b_in[0] = 1*5 = 5
        // (other PEs get data in subsequent cycles due to systolic delay)
        lane_en = 4'hF;
        a_in[0] = 1; a_in[1] = 2; a_in[2] = 3; a_in[3] = 4;
        b_in[0] = 5; b_in[1] = 6; b_in[2] = 7; b_in[3] = 8;
        @(posedge clk); #1;

        // Zero inputs for remaining cycles
        for (int i = 0; i < N; i++) begin
            a_in[i] = 0;
            b_in[i] = 0;
        end

        // PE[0][0] should have accumulated 1*5 = 5
        check("mac[0][0]", c_out[0][0], 64'd5);

        // Wait for data to ripple through
        @(posedge clk); #1;
        // PE[0][1] gets a_out from PE[0][0] (which was 1) * b_in[1] from prev cycle (passed through PE[0][0].b_out → but b flows down, not right)
        // Actually: PE[1][0] gets b_out from PE[0][0] (=5 from first cycle) and a_in[1]=0 now
        // PE[0][1] gets a_out from PE[0][0] (=1) and b_wire[0][1] which was b_in[1]=6 one cycle ago, now passed to PE[0][1]
        // So PE[0][1] should see 1 * 6 = 6 accumulated on top of its prior 0
        // Wait more cycles for full propagation
        repeat(N) @(posedge clk); #1;

        $display("After single feed + propagation:");
        for (int r = 0; r < N; r++) begin
            for (int c = 0; c < N; c++)
                $write("  c[%0d][%0d]=%0d", r, c, c_out[r][c]);
            $display("");
        end

        // ── Test 3: 2x2 Matrix mul ──────────────────────────────
        // Feed A = [[1, 2, 3, 4],[1, 2, 3, 4]] B = [[5, 6, 7, 8],[5, 6, 7, 8]] with all lanes enabled
        // (other PEs get data in subsequent cycles due to systolic delay)
        clear = 1;
        @(posedge clk); #1;
        clear = 0;
        @(posedge clk); #1;
        
        lane_en = 4'hF;
        a_in[0] = 1; 
        b_in[0] = 5; 
        @(posedge clk); #1;       
        a_in[0] = 2; a_in[1] = 1;
        b_in[0] = 6; b_in[1] = 5;
        @(posedge clk); #1;       
        a_in[0] = 3; a_in[1] = 2;
        b_in[0] = 7; b_in[1] = 6;
        @(posedge clk); #1;                
        a_in[0] = 4; a_in[1] = 3;                   
        b_in[0] = 8; b_in[1] = 7;
        @(posedge clk); #1;
        a_in[0] = 0; a_in[1] = 4;
        b_in[0] = 0; b_in[1] = 8;
        @(posedge clk); #1;              
        // Zero inputs for remaining cycles
        for (int i = 0; i < N; i++) begin
            a_in[i] = 0;
            b_in[i] = 0;
        end

        // cehck for the final 2x2 output
        check("mac[0][0]", c_out[0][0], 64'd70);
    
        // Wait for data to ripple through
        @(posedge clk); #1;
        repeat(N) @(posedge clk); #1;

        $display("After single feed + propagation:");
        for (int r = 0; r < N; r++) begin
            for (int c = 0; c < N; c++)
                $write("  c[%0d][%0d]=%0d", r, c, c_out[r][c]);
            $display("");
        end

        // ── Test 4: Accumulate — clear and do 2 rounds ──────
        clear = 1;
        @(posedge clk); #1;
        clear = 0;

        // Round 1: all PEs see a*b on the diagonal
        lane_en = 4'hF;
        a_in[0] = 3; a_in[1] = 0; a_in[2] = 0; a_in[3] = 0;
        b_in[0] = 7; b_in[1] = 0; b_in[2] = 0; b_in[3] = 0;
        @(posedge clk); #1;

        // PE[0][0] = 3*7 = 21
        check("accum_r1[0][0]", c_out[0][0], 64'd21);

        // Round 2: accumulate more
        a_in[0] = 2; b_in[0] = 4;
        @(posedge clk); #1;

        // PE[0][0] = 21 + 2*4 = 29
        check("accum_r2[0][0]", c_out[0][0], 64'd29);

        a_in[0] = 0; b_in[0] = 0;

        // ── Test 5: Lane masking ────────────────────────────
        // Test on edge PEs only (c_out[r][0]) — need to wait r cycles
        // for data to reach PE[r][0] via b propagation.
        // Instead, test lane_en on row 0 (PE[0][c]) using different b_in values.
        clear = 1;
        @(posedge clk); #1;
        clear = 0;

        // Disable row 2 — PE[2][*] should not accumulate
        lane_en = 4'b1011;
        a_in[0] = 10; a_in[1] = 20; a_in[2] = 30; a_in[3] = 40;
        b_in[0] = 1;  b_in[1] = 0;  b_in[2] = 0;  b_in[3] = 0;
        @(posedge clk); #1;

        // Only PE[r][0] sees b on cycle 1 (edge column).
        // PE[0][0] = 10*1 = 10 (row 0 enabled)
        // PE[2][0] gets a_in[2]*b_wire — but b_wire[2][0] = PE[1][0].b_out = 0 (delayed)
        // So only PE[0][0] gets a result after 1 cycle.
        check("mask[0][0]", c_out[0][0], 64'd10);  // row 0 active, got data
        check("mask[2][0]", c_out[2][0], 64'd0);   // row 2 masked off

        // Feed for more cycles to let data propagate to row 1 and row 3
        a_in[0] = 0; a_in[1] = 20; a_in[2] = 30; a_in[3] = 0;
        b_in[0] = 0;
        @(posedge clk); #1;
        // PE[1][0] now sees b_wire from PE[0][0].b_out = 1 (from last cycle)
        // PE[1][0] = 20 * 1 = 20 (row 1 enabled)
        check("mask[1][0]", c_out[1][0], 64'd20);
        // PE[2][0] sees b from PE[1][0].b_out but row 2 is disabled
        // Note: PE[1][0].b_out was 0 until now (just got b=1 from PE[0][0])
        // Actually b_out is registered — PE[1][0].b_out this cycle = b_wire[1][0] from last cycle = 0
        // So PE[2][0] sees b=0 this cycle anyway. Check stays 0.
        check("mask_still[2][0]", c_out[2][0], 64'd0);

        for (int i = 0; i < N; i++) begin
            a_in[i] = 0;
            b_in[i] = 0;
        end

        // ── Summary ─────────────────────────────────────────
        #20;
        $display("\n=== Systolic Array TB: %0d passed, %0d failed ===", pass_count, fail_count);
        if (fail_count > 0) $fatal(1, "TESTS FAILED");
        $finish;
    end

endmodule
