//im writing this testbench to fimiliarize myself with such large register files 
//and understand how the register accesses will work  
module tb_vector_file;
    parameter VLEN  = 512;
    parameter NREGS = 8;
    parameter SEW   = 32;
    parameter VLMAX = VLEN/SEW;
    parameter MASK_BITS = VLMAX;

    logic                     clk;
    logic                     rst_n;
    logic [$clog2(NREGS)-1:0] rs1_addr;
    logic [VLEN-1:0]          rs1_data;
    logic [$clog2(NREGS)-1:0] rs2_addr;
    logic [VLEN-1:0]          rs2_data;
    logic [$clog2(NREGS)-1:0] rd_rd_addr;
    logic [VLEN-1:0]          rd_rd_data;
    logic                     wr_en;
    logic [$clog2(NREGS)-1:0] wr_addr;
    logic [VLEN-1:0]          wr_data;
    logic [MASK_BITS-1:0]     mask_v0;    

    vector_regfile #(
        .VLEN(VLEN),
        .NREGS(NREGS),
        .SEW(SEW),
        .VLMAX(VLMAX),
        .MASK_BITS(MASK_BITS)
    ) dut(
        .clk(clk),
        .rst_n(rst_n),
        .rs1_addr(rs1_addr),
        .rs1_data(rs1_data),
        .rs2_addr(rs2_addr),
        .rs2_data(rs2_data),
        .rd_rd_addr(rd_rd_addr),
        .rd_rd_data(rd_rd_data),
        .wr_en(wr_en),
        .wr_addr(wr_addr),
        .wr_data(wr_data),
        .mask_v0(mask_v0)
    );

    initial clk = 0;
    always #5 clk = ~clk;

    initial begin
        rst_n = 0;
        repeat(2)@(posedge clk);
        rst_n = 1;
        @(posedge clk);

//test 1 simple write and read test
        wr_en = 1;
        wr_addr = 3'b001;
        wr_data = 32'h1111_1111;
        repeat(2)@(posedge clk);
        wr_en = 0;
        rs1_addr = 3'b001;
        @(posedge clk);

//test 2 do multiple large vector writes and reads 
        wr_en = 1;
        wr_addr = 3'b010;
        wr_data = 64'h3124_9008_ABCD_1234;
        @(posedge clk);
        rs1_addr = 3'b010;
        rs2_addr = 3'b001; // reading the data written in the 1st test
        @(posedge clk);
        $finish;
        //hence the combinational reads work as expected instant reads when addresses arrives
        //rd will work the same way hence not testing it
    end
endmodule