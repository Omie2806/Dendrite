// Testbench — full Dendrite coprocessor integration test.
//
// Simulates CVA6 issuing RVV instructions through the coprocessor interface.
// Tests:
//   1. vsetvli — configure VLEN/SEW, verify returned vl
//   2. vadd.vv — encode a real RVV instruction, verify result
//   3. vadd.vx — scalar broadcast add
//   4. vmacc.vv — multiply-accumulate
//   5. Masked operation — verify lane masking through full pipeline
`timescale 1ns/1ps

module tb_dendrite_top;

    parameter int VLEN  = 512;
    parameter int SEW   = 32;
    parameter int VLMAX = VLEN / SEW;
    parameter int NREGS = 8;

    logic           clk, rst_n;

    // CVA6 interface
    logic           req_valid;
    wire            req_ready;
    logic [31:0]    req_instr;
    logic [31:0]    req_rs1;

    wire            resp_valid;
    logic           resp_ready;
    wire  [63:0]    resp_data;
    wire            resp_error;

    dendrite_top #(
        .VLEN(VLEN), .SEW(SEW), .NREGS(NREGS)
    ) dut (
        .clk(clk), .rst_n(rst_n),
        .req_valid(req_valid), .req_ready(req_ready),
        .req_instr(req_instr), .req_rs1(req_rs1),
        .resp_valid(resp_valid), .resp_ready(resp_ready),
        .resp_data(resp_data), .resp_error(resp_error)
    );

    initial clk = 0;
    always #5 clk = ~clk;

    int pass_count = 0;
    int fail_count = 0;
    integer i_loop;

    // ── RVV instruction encoders ────────────────────────────

    // vsetvli rd, rs1, vtypei
    // [31]   = 0
    // [30:20] = zimm[10:0] (vtypei: vsew[5:3], vlmul[2:0])
    // [19:15] = rs1
    // [14:12] = 111
    // [11:7]  = rd
    // [6:0]   = 1010111 (OP-V)
    function automatic [31:0] enc_vsetvli(
        input [4:0] rd, input [4:0] rs1,
        input [2:0] vsew, input [2:0] vlmul
    );
        return {1'b0, 5'b0, vsew, vlmul, rs1, 3'b111, rd, 7'b1010111};
    endfunction

    // Vector arithmetic: OPIVV (funct3=000) or OPIVX (funct3=100)
    // [31:26] = funct6
    // [25]    = vm (1=unmasked, 0=masked)
    // [24:20] = vs2 / rs2
    // [19:15] = vs1 / rs1
    // [14:12] = funct3
    // [11:7]  = vd
    // [6:0]   = 1010111
    function automatic [31:0] enc_opivv(
        input [5:0] funct6, input logic vm,
        input [4:0] vs2, input [4:0] vs1, input [4:0] vd
    );
        return {funct6, vm, vs2, vs1, 3'b000, vd, 7'b1010111};
    endfunction

    function automatic [31:0] enc_opivx(
        input [5:0] funct6, input logic vm,
        input [4:0] vs2, input [4:0] rs1, input [4:0] vd
    );
        return {funct6, vm, vs2, rs1, 3'b100, vd, 7'b1010111};
    endfunction

    function automatic [31:0] enc_opmvv(
        input [5:0] funct6, input logic vm,
        input [4:0] vs2, input [4:0] vs1, input [4:0] vd
    );
        return {funct6, vm, vs2, vs1, 3'b010, vd, 7'b1010111};
    endfunction

    // ── Issue instruction and wait for response ─────────────
    task automatic issue(
        input [31:0] instr,
        input [31:0] rs1_val,
        output [63:0] result,
        output logic err
    );
        @(negedge clk);
        req_valid = 1'b1;
        req_instr = instr;
        req_rs1   = rs1_val;

        // Wait for accept
        do @(posedge clk); while (!req_ready);
        @(negedge clk);
        req_valid = 1'b0;

        // Wait for response
        resp_ready = 1'b1;
        do @(posedge clk); while (!resp_valid);
        result = resp_data;
        err    = resp_error;
        @(negedge clk);
        resp_ready = 1'b0;
    endtask

    // ── Direct VRF write (through DUT hierarchy) ────────────
    task automatic write_vreg(input int idx, input [VLEN-1:0] data);
        @(posedge clk);
        force dut.u_vrf.wr_en = 1;
        force dut.u_vrf.wr_addr = idx[$clog2(NREGS)-1:0];
        force dut.u_vrf.wr_data = data;
        @(posedge clk);
        release dut.u_vrf.wr_en;
        release dut.u_vrf.wr_addr;
        release dut.u_vrf.wr_data;
    endtask

    function automatic [VLEN-1:0] read_vreg(input int idx);
        return dut.u_vrf.vreg[idx];
    endfunction

    function automatic [VLEN-1:0] make_vec(input int base, input int stride);
        logic [VLEN-1:0] v;
        for (int ii = 0; ii < VLMAX; ii++)
            v[ii*SEW +: SEW] = SEW'(base + ii * stride);
        return v;
    endfunction

    function automatic signed [SEW-1:0] get_elem(input [VLEN-1:0] v, input int idx);
        return v[idx*SEW +: SEW];
    endfunction

    task automatic check_vreg_elem(input string name, input int vreg_idx, input int elem_idx,
                                   input logic signed [SEW-1:0] expected);
        logic [VLEN-1:0] v;
        logic signed [SEW-1:0] got;
        v = read_vreg(vreg_idx);
        got = get_elem(v, elem_idx);
        if (got === expected)
            pass_count = pass_count + 1;
        else begin
            $display("FAIL %s v%0d[%0d]: got %0d, expected %0d",
                     name, vreg_idx, elem_idx, got, expected);
            fail_count = fail_count + 1;
        end
    endtask

    // ── Main test ───────────────────────────────────────────
    logic [63:0] result;
    logic        err;
    logic [31:0] instr;
    logic signed [SEW-1:0] expected;

    initial begin
        $dumpfile("tb_dendrite_top.fst");
        $dumpvars(0, tb_dendrite_top);

        rst_n      = 0;
        req_valid  = 0;
        resp_ready = 0;
        req_instr  = 0;
        req_rs1    = 0;

        #30 rst_n = 1;
        #10;

        // ── Test 1: vsetvli ─────────────────────────────────
        $display("\n--- Test 1: vsetvli (SEW=32, LMUL=1, AVL=16) ---");
        begin
            instr = enc_vsetvli(5'd1, 5'd2, 3'b010, 3'b000);
            // rs1 = AVL = 16
            issue(instr, 32'd16, result, err);
            if (result[SEW-1:0] == SEW'(16) && !err) begin
                $display("PASS vsetvli: vl=%0d", result);
                pass_count = pass_count + 1;
            end else begin
                $display("FAIL vsetvli: vl=%0d, err=%0b", result, err);
                fail_count = fail_count + 1;
            end
        end

        // ── Test 2: vadd.vv ─────────────────────────────────
        $display("\n--- Test 2: vadd.vv v3, v1, v2 ---");
        begin
            // Pre-load v1 and v2
            write_vreg(1, make_vec(1, 1));    // v1 = [1,2,3,...,16]
            write_vreg(2, make_vec(100, 10)); // v2 = [100,110,...,250]
            #10;

            // vadd.vv v3, v1, v2 (unmasked)
            // funct6=000000, vm=1, vs2=2, vs1=1, vd=3
            instr = enc_opivv(6'b000000, 1'b1, 5'd2, 5'd1, 5'd3);
            issue(instr, 32'd0, result, err);

            // v3[i] = v1[i] + v2[i]
            for (i_loop = 0; i_loop < VLMAX; i_loop = i_loop + 1)
                check_vreg_elem("vadd.vv", 3, i_loop, SEW'((1+i_loop) + (100 + i_loop*10)));
        end

        // ── Test 3: vadd.vx ─────────────────────────────────
        $display("\n--- Test 3: vadd.vx v4, v1, x (scalar=42) ---");
        begin
            // vadd.vx v4, v1, rs1  — funct6=000000, vm=1, vs2=1, rs1(vs1 field)=1, vd=4
            // For .vx, funct3=100, scalar comes from req_rs1
            instr = enc_opivx(6'b000000, 1'b1, 5'd1, 5'd1, 5'd4);
            issue(instr, 32'd42, result, err);

            // v4[i] = v1[i] + 42
            for (i_loop = 0; i_loop < VLMAX; i_loop = i_loop + 1)
                check_vreg_elem("vadd.vx", 4, i_loop, SEW'((1+i_loop) + 42));
        end

        // ── Test 4: vmacc.vv ────────────────────────────────
        $display("\n--- Test 4: vmacc.vv v5, v1, v2 ---");
        begin
            write_vreg(5, make_vec(1000, 0));  // v5 = all 1000s
            #10;

            // vmacc.vv v5, v1, v2 — funct6=101101
            instr = enc_opmvv(6'b101101, 1'b1, 5'd2, 5'd1, 5'd5);
            issue(instr, 32'd0, result, err);

            // v5[i] = 1000 + v1[i] * v2[i]
            for (i_loop = 0; i_loop < VLMAX; i_loop = i_loop + 1) begin
                expected = SEW'(1000 + (1+i_loop) * (100 + i_loop*10));
                check_vreg_elem("vmacc.vv", 5, i_loop, expected);
            end
        end

        // ── Test 5: Masked vadd.vv ──────────────────────────
        $display("\n--- Test 5: vadd.vv v6, v1, v2, v0.t (partial mask) ---");
        begin
            // Set v0 mask: lanes 0-3 active, 4-7 inactive, 8-11 active, 12-15 inactive
            write_vreg(0, {{(VLEN-VLMAX){1'b0}}, 16'b0000_1111_0000_1111});
            write_vreg(6, make_vec(777, 0));  // v6 = all 777s (old values)
            #10;

            // vadd.vv v6, v1, v2, v0.t — vm=0 means masked
            instr = enc_opivv(6'b000000, 1'b0, 5'd2, 5'd1, 5'd6);
            issue(instr, 32'd0, result, err);

            // Active lanes: v6[i] = v1[i] + v2[i]
            check_vreg_elem("masked_active", 6, 0, SEW'(1 + 100));
            check_vreg_elem("masked_active", 6, 3, SEW'(4 + 130));
            // Inactive lanes: v6[i] preserved = 777
            check_vreg_elem("masked_inactive", 6, 4, SEW'(777));
            check_vreg_elem("masked_inactive", 6, 7, SEW'(777));
            // Active warp 2
            check_vreg_elem("masked_active_w2", 6, 8, SEW'(9 + 180));
            // Inactive warp 3
            check_vreg_elem("masked_inactive_w3", 6, 12, SEW'(777));
        end

        // ── Summary ─────────────────────────────────────────
        #50;
        $display("\n=== Dendrite Top TB: %0d passed, %0d failed ===", pass_count, fail_count);
        if (fail_count > 0) $fatal(1, "TESTS FAILED");
        $finish;
    end

endmodule
