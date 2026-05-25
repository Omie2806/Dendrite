wont the read and write buses for vrf be too wide? but i guess thats the compromise we make for fast matmul
but the register files itself would occupy a lot of area.
asynchronous reads wouldnt allow bram utilization on a fpga but ig its okay for now

systollic array:
To do actual matrix multiplication, the subsequent rows and columns should arrive a cycle late
for example, in a 2x2 matrix, a[1] and b[1] arrive a cycle later than a[0] and b[0];  
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

so the input martices are A = [[1 ,2 ,3 ,4], [1, 2, 3, 4]];    B = [[5, 6, 7, 8], [5, 6, 7, 8]];

Hence                     AxB = [[70, 70], [70, 70]];