// Coprocessor interface — decodes RVV instructions from CVA6 and drives Dendrite.
//
// CVA6 dispatches vector instructions via a req/resp handshake (modeled on
// the CORE-V X-Interface issue channel).  This module:
//   1. Accepts the 32-bit RVV instruction + scalar operands from CVA6
//   2. Decodes it into an internal op_type
//   3. Manages vtype/vl CSR state (vsetvli/vsetivli)
//   4. Reads operands from the vector register file
//   5. Dispatches to the warp scheduler
//   6. Writes results back to the vector register file
//   7. Returns scalar results (vsetvli) to CVA6
//
// RVV instruction format (OP-V, opcode=1010111):
//   [31:26] funct6
//   [25]    vm        (1=unmasked, 0=masked v0.t)
//   [24:20] vs2
//   [19:15] vs1/rs1
//   [14:12] funct3    (000=OPIVV, 001=OPFVV, 010=OPMVV,
//                       011=OPIVI, 100=OPIVX, 101=OPFVF, 110=OPMVX, 111=OPCFG)
//   [11:7]  vd/rd
//   [6:0]   1010111
//
// vsetvli encoding:
//   [31]    0
//   [30:20] zimm[10:0]  →  [22:20]=vlmul, [25:23]=vsew, [26]=vta, [27]=vma
//   [19:15] rs1 (AVL source register)
//   [14:12] 111 (OPCFG)
//   [11:7]  rd
//   [6:0]   1010111
`default_nettype none

module coprocessor_if #(
    parameter int VLEN   = 512,
    parameter int SEW    = 32,
    parameter int VLMAX  = VLEN / SEW,   // 16
    parameter int NREGS  = 8
)(
    input  wire                 clk,
    input  wire                 rst_n,

    // ── CVA6 issue interface ────────────────────────────────
    input  wire                 req_valid,
    output reg                  req_ready,
    input  wire [31:0]          req_instr,       // full RVV instruction
    input  wire [31:0]          req_rs1,         // scalar x[rs1] lower 32 bits

    // ── CVA6 result interface ───────────────────────────────
    output reg                  resp_valid,
    input  wire                 resp_ready,
    output reg  [63:0]          resp_data,       // scalar result (vsetvli → new vl)
    output reg                  resp_error,

    // ── Vector register file ports ──────────────────────────
    output reg  [$clog2(NREGS)-1:0] vrf_rs1_addr,
    input  wire [VLEN-1:0]          vrf_rs1_data,
    output reg  [$clog2(NREGS)-1:0] vrf_rs2_addr,
    input  wire [VLEN-1:0]          vrf_rs2_data,
    output reg  [$clog2(NREGS)-1:0] vrf_rd_rd_addr,
    input  wire [VLEN-1:0]          vrf_rd_rd_data,
    output reg                      vrf_wr_en,
    output reg  [$clog2(NREGS)-1:0] vrf_wr_addr,
    output reg  [VLEN-1:0]          vrf_wr_data,
    input  wire [VLMAX-1:0]         vrf_mask_v0,

    // ── Warp scheduler dispatch ─────────────────────────────
    output reg                  ws_op_valid,
    input  wire                 ws_op_ready,
    output reg  [4:0]           ws_op_type,
    output reg                  ws_op_masked,
    output reg  [VLEN-1:0]      ws_op_vs1,
    output reg  [VLEN-1:0]      ws_op_vs2,
    output reg  [VLEN-1:0]      ws_op_vd_old,
    output reg  [SEW-1:0]       ws_op_scalar,
    output reg  [VLMAX-1:0]     ws_op_mask,

    // ── Warp scheduler writeback ────────────────────────────
    input  wire                 ws_wb_valid,
    output reg                  ws_wb_ready,
    input  wire [VLEN-1:0]      ws_wb_data
);

    // ── RVV major opcode ────────────────────────────────────
    localparam [6:0] OPC_OP_V = 7'b1010111;

    // ── RVV funct3 categories ───────────────────────────────
    localparam [2:0] F3_OPIVV = 3'b000;     // integer vector-vector
    localparam [2:0] F3_OPMVV = 3'b010;     // multiply  vector-vector
    localparam [2:0] F3_OPIVX = 3'b100;     // integer vector-scalar
    localparam [2:0] F3_OPMVX = 3'b110;     // multiply  vector-scalar
    localparam [2:0] F3_OPCFG = 3'b111;     // vsetvli / vsetivli

    // ── RVV funct6 codes ────────────────────────────────────
    // OPIVV / OPIVX operations (funct3=000/100)
    localparam [5:0] F6_VADD   = 6'b000000;
    localparam [5:0] F6_VSUB   = 6'b000010;
    localparam [5:0] F6_VAND   = 6'b001001;
    localparam [5:0] F6_VOR    = 6'b001010;
    localparam [5:0] F6_VXOR   = 6'b001011;
    localparam [5:0] F6_VMSEQ  = 6'b011000;
    localparam [5:0] F6_VMSLT  = 6'b011011;
    localparam [5:0] F6_VSLL   = 6'b100101;
    localparam [5:0] F6_VSRL   = 6'b101000;
    // OPMVV / OPMVX operations (funct3=010/110) — same funct6 space, different funct3
    localparam [5:0] F6_VMUL   = 6'b100101;  // same encoding as VSLL, disambiguated by funct3
    localparam [5:0] F6_VMACC  = 6'b101101;
    localparam [5:0] F6_VMADD  = 6'b101001;

    // ── Internal op codes (match warp_scheduler.sv) ─────────
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

    // ── RVV CSR state ───────────────────────────────────────
    reg [SEW-1:0] vl;           // vector length (active elements)
    reg [2:0]     vsew_r;       // selected element width encoding
    reg [2:0]     vlmul_r;      // length multiplier encoding
    reg           vill_r;       // illegal config flag

    // ── FSM ─────────────────────────────────────────────────
    typedef enum logic [2:0] {
        S_IDLE,
        S_DECODE,
        S_READ_VRF,
        S_DISPATCH,
        S_WAIT_WB,
        S_WRITE_VRF,
        S_RESP
    } state_t;

    state_t state;

    // ── Latched instruction fields ──────────────────────────
    // Decomposed from req_instr to avoid unused-bit warnings
    reg [6:0]   r_opcode;       // [6:0]
    reg [2:0]   r_funct3;       // [14:12]
    reg [5:0]   r_funct6;       // [31:26]
    reg         r_vm;           // [25]
    reg [2:0]   r_vsew_bits;    // [25:23] = zimm[5:3] for vsetvli
    reg [4:0]   r_vs2_raw;      // [24:20]
    reg [4:0]   r_vs1_raw;      // [19:15]
    reg [4:0]   r_vd_raw;       // [11:7]
    reg [31:0]  r_rs1;          // scalar operand (lower 32 bits of req_rs1)
    reg [4:0]   r_op_type;
    reg         r_masked;
    reg         r_is_vsetvl;
    reg         r_needs_vd_read;
    reg         r_error;        // latched error from decode
    reg [2:0]   r_vd_addr, r_vs1_addr, r_vs2_addr;

    reg [VLEN-1:0] r_wb_data;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= S_IDLE;
            req_ready    <= 1'b1;
            resp_valid   <= 1'b0;
            resp_data    <= '0;
            resp_error   <= 1'b0;
            vl           <= SEW'(VLMAX);
            vsew_r       <= 3'b010;     // SEW=32
            vlmul_r      <= 3'b000;     // LMUL=1
            vill_r       <= 1'b0;
            ws_op_valid  <= 1'b0;
            ws_wb_ready  <= 1'b0;
            vrf_wr_en    <= 1'b0;
            r_opcode     <= '0;
            r_funct3     <= '0;
            r_funct6     <= '0;
            r_vm         <= 1'b1;
            r_vsew_bits  <= 3'b010;
            r_vs2_raw    <= '0;
            r_vs1_raw    <= '0;
            r_vd_raw     <= '0;
            r_rs1        <= '0;
            r_op_type    <= OP_NOP;
            r_masked     <= 1'b0;
            r_is_vsetvl  <= 1'b0;
            r_needs_vd_read <= 1'b0;
            r_error      <= 1'b0;
            r_vd_addr    <= '0;
            r_vs1_addr   <= '0;
            r_vs2_addr   <= '0;
            r_wb_data    <= '0;
        end else begin
            vrf_wr_en   <= 1'b0;
            resp_valid  <= 1'b0;
            ws_op_valid <= 1'b0;
            ws_wb_ready <= 1'b0;

            case (state)
                S_IDLE: begin
                    req_ready <= 1'b1;
                    if (req_valid && req_ready) begin
                        r_opcode   <= req_instr[6:0];
                        r_funct3   <= req_instr[14:12];
                        r_funct6   <= req_instr[31:26];
                        r_vm       <= req_instr[25];
                        r_vsew_bits <= req_instr[25:23];
                        r_vs2_raw  <= req_instr[24:20];
                        r_vs1_raw  <= req_instr[19:15];
                        r_vd_raw   <= req_instr[11:7];
                        r_rs1      <= req_rs1[31:0];
                        req_ready  <= 1'b0;
                        state      <= S_DECODE;
                    end
                end

                S_DECODE: begin
                    r_masked   <= !r_vm;
                    r_vd_addr  <= r_vd_raw[2:0];
                    r_vs1_addr <= r_vs1_raw[2:0];
                    r_vs2_addr <= r_vs2_raw[2:0];
                    r_is_vsetvl     <= 1'b0;
                    r_needs_vd_read <= 1'b0;

                    if (r_opcode != OPC_OP_V) begin
                        r_error <= 1'b1;
                        state <= S_RESP;
                    end else if (r_funct3 != F3_OPCFG &&
                                 (|r_vd_raw[4:3] || |r_vs1_raw[4:3] || |r_vs2_raw[4:3])) begin
                        // Reject out-of-range register indices (8-reg file) for arithmetic ops.
                        // vsetvli reuses these fields as immediates, not register indices.
                        r_error <= 1'b1;
                        state <= S_RESP;
                    end else begin
                        case (r_funct3)
                            // ── vsetvli / vsetivli ──────────────
                            F3_OPCFG: begin
                                r_is_vsetvl <= 1'b1;
                                // zimm[2:0] = vlmul → vs2_raw (instr[22:20])
                                // zimm[5:3] = vsew  → {vm, vs2_raw[2:1]} = instr[25:23]
                                vlmul_r <= r_vs2_raw[2:0];
                                vsew_r  <= r_vsew_bits;     // zimm[5:3] = instr[25:23]
                                if (r_rs1 > 32'(VLMAX))
                                    vl <= SEW'(VLMAX);
                                else
                                    vl <= r_rs1[SEW-1:0];
                                vill_r <= (r_vsew_bits != 3'b010);
                                state  <= S_RESP;
                            end

                            // ── Arithmetic ops — reject if vtype is illegal
                            F3_OPIVV: begin
                                automatic logic [4:0] op = decode_opivv(r_funct6);
                                if (vill_r || op == OP_NOP) begin
                                    r_error <= 1'b1;
                                    state <= S_RESP;
                                end else begin
                                    r_op_type <= op;
                                    // Masked operations preserve old vd on masked-off lanes.
                                    r_needs_vd_read <= !r_vm;
                                    state <= S_READ_VRF;
                                end
                            end

                            F3_OPIVX: begin
                                automatic logic [4:0] op = decode_opivx(r_funct6);
                                if (vill_r || op == OP_NOP) begin
                                    r_error <= 1'b1;
                                    state <= S_RESP;
                                end else begin
                                    r_op_type <= op;
                                    // Masked operations preserve old vd on masked-off lanes.
                                    r_needs_vd_read <= !r_vm;
                                    state <= S_READ_VRF;
                                end
                            end

                            F3_OPMVV: begin
                                automatic logic [4:0] op = decode_opmvv(r_funct6);
                                if (vill_r || op == OP_NOP) begin
                                    r_error <= 1'b1;
                                    state <= S_RESP;
                                end else begin
                                    r_op_type <= op;
                                    r_needs_vd_read <= is_accum_opm(r_funct6) || !r_vm;
                                    state <= S_READ_VRF;
                                end
                            end

                            F3_OPMVX: begin
                                automatic logic [4:0] op = decode_opmvx(r_funct6);
                                if (vill_r || op == OP_NOP) begin
                                    r_error <= 1'b1;
                                    state <= S_RESP;
                                end else begin
                                    r_op_type <= op;
                                    r_needs_vd_read <= is_accum_opm(r_funct6) || !r_vm;
                                    state <= S_READ_VRF;
                                end
                            end

                            default: begin
                                r_error <= 1'b1;
                                state <= S_RESP;
                            end
                        endcase
                    end
                end

                S_READ_VRF: begin
                    vrf_rs1_addr   <= r_vs1_addr;
                    vrf_rs2_addr   <= r_vs2_addr;
                    vrf_rd_rd_addr <= r_vd_addr;
                    state <= S_DISPATCH;
                end

                S_DISPATCH: begin
                    ws_op_type   <= r_op_type;
                    ws_op_masked <= r_masked;
                    ws_op_vs1    <= vrf_rs1_data;
                    ws_op_vs2    <= vrf_rs2_data;
                    ws_op_vd_old <= r_needs_vd_read ? vrf_rd_rd_data : '0;
                    ws_op_scalar <= r_rs1[SEW-1:0];
                    ws_op_mask   <= vrf_mask_v0;
                    ws_op_valid  <= 1'b1;   // hold high until accepted

                    if (ws_op_valid && ws_op_ready) begin
                        ws_op_valid <= 1'b0;
                        state <= S_WAIT_WB;
                    end
                end

                S_WAIT_WB: begin
                    ws_wb_ready <= 1'b1;
                    if (ws_wb_valid) begin
                        r_wb_data <= ws_wb_data;
                        state <= S_WRITE_VRF;
                    end
                end

                S_WRITE_VRF: begin
                    vrf_wr_en   <= 1'b1;
                    vrf_wr_addr <= r_vd_addr;
                    vrf_wr_data <= r_wb_data;
                    state <= S_RESP;
                end

                S_RESP: begin
                    resp_valid <= 1'b1;
                    resp_data  <= r_is_vsetvl ?
                        {vill_r, 23'b0, 2'b0, vsew_r, vlmul_r, vl} : '0;
                    resp_error <= r_error;
                    if (resp_ready) begin
                        r_error <= 1'b0;
                        state <= S_IDLE;
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

    // ── Decode: OPIVV (funct3=000) — integer vector-vector ──
    function automatic [4:0] decode_opivv(input [5:0] f6);
        case (f6)
            F6_VADD:  return OP_VADD_VV;
            F6_VSUB:  return OP_VSUB_VV;
            F6_VAND:  return OP_VAND_VV;
            F6_VOR:   return OP_VOR_VV;
            F6_VXOR:  return OP_VXOR_VV;
            F6_VMSEQ: return OP_VMSEQ_VV;
            F6_VMSLT: return OP_VMSLT_VV;
            F6_VSLL:  return OP_VSLL_VV;
            F6_VSRL:  return OP_VSRL_VV;
            default:  return OP_NOP;
        endcase
    endfunction

    // ── Decode: OPIVX (funct3=100) — integer vector-scalar ──
    function automatic [4:0] decode_opivx(input [5:0] f6);
        case (f6)
            F6_VADD: return OP_VADD_VX;
            F6_VSUB: return OP_VSUB_VX;
            default: return OP_NOP;
        endcase
    endfunction

    // ── Decode: OPMVV (funct3=010) — multiply vector-vector ─
    function automatic [4:0] decode_opmvv(input [5:0] f6);
        case (f6)
            F6_VMUL:  return OP_VMUL_VV;
            F6_VMACC: return OP_VMACC_VV;
            default:  return OP_NOP;
        endcase
    endfunction

    // ── Decode: OPMVX (funct3=110) — multiply vector-scalar ─
    function automatic [4:0] decode_opmvx(input [5:0] f6);
        case (f6)
            F6_VMUL:  return OP_VMUL_VX;
            F6_VMACC: return OP_VMACC_VX;
            default:  return OP_NOP;
        endcase
    endfunction

    // ── Accumulate check (for OPMVV/OPMVX) ──────────────────
    function automatic logic is_accum_opm(input [5:0] f6);
        return (f6 == F6_VMACC) || (f6 == F6_VMADD);
    endfunction

endmodule
