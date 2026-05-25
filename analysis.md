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

hence
  c[0][0]=70  c[0][1]=70  c[0][2]=0  c[0][3]=0
  c[1][0]=70  c[1][1]=70  c[1][2]=0  c[1][3]=0
  c[2][0]=0  c[2][1]=0  c[2][2]=0  c[2][3]=0
  c[3][0]=0  c[3][1]=0  c[3][2]=0  c[3][3]=0

similarly for 4x4 tests A = [[1, 2, 3, 4],[2, 3, 4, 5],[3, 4, 5, 6],[4 ,5, 6, 7]] = B

hence  
  c[0][0]=30  c[0][1]=40  c[0][2]=50  c[0][3]=60
  c[1][0]=40  c[1][1]=54  c[1][2]=68  c[1][3]=82
  c[2][0]=50  c[2][1]=68  c[2][2]=86  c[2][3]=104
  c[3][0]=60  c[3][1]=82  c[3][2]=104  c[3][3]=126