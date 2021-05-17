module fpl(cs, re, we, d, out, clk, rst);
   input 	  cs;
   input 	  we;
   input 	  re;

   input [7:0] 	  d;
   output [18:13] out;
   input 	  clk;
   input 	  rst;

   reg [18:13] 	  latch;

   assign out[18:13] = latch[18:13];

   assign d[7:0] = (cs && re) ? 8'h46 : 8'bzzzzzzzz;

   always @(posedge clk)
     begin
	if (rst)
	  latch[18:13] <= 6'b000000;
	else if (cs && we)
	  latch[18:13] <= d[5:0];
     end

endmodule // fpl
