module fpl(cs, nwe, d, out, clk);
   input 	  cs;
   input 	  nwe;
   input [7:0] 	  d;
   output [18:13] out;
   input 	  clk;

   reg [18:13] 	  latch;

   assign out[18:13] = latch[18:13];
   
   always @(posedge clk)
     begin
	if (cs && !nwe)
	  latch[18:13] <= d[5:0];
     end   

endmodule // fpl
