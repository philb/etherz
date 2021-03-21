module ide(a, d, cs, clk, irq, rnw, reset);
   input [13:2] a;
   inout [7:0] 	d;
   input 	cs;
   input 	clk;
   output 	irq;

   reg 		host_irq_requested; 	
   reg 		bsy; 		
 
   reg [7:0] command;
   reg [7:0] lba0;
   reg [7:0] lba1;
   reg [7:0] lba2;
   reg [7:0] lba3;
   reg [7:0] sector_count;

   reg [8:0] sector_ram_ra;

   wire [7:0] sector_ram_dout, sector_ram_din;
   
   SB_RAM512x8 sector_ram (.RDATA (sector_ram_dout), 
			   .RADDR (sector_ram_ra), 
			   .RCLK (clk), 
			   .RCLKE (1'b1),
			   .RE (sector_ram_re),
			   .WDATA (sector_ram_din), 
			   .WCLK (clk), 
			   .WCLKE (1'b1),
			   .WE (sector_ram_we), 
			   .WADDR (sector_ram_wa));

   parameter IDLE = 2'b00;
   parameter R1 = 2'b01;
   parameter R2 = 2'b10;

   assign sector_ram_re = (cs && rnw && ...);
   assign sector_ram_we = (cs && !rnw && host_state == R1);
   
   reg [1:0]  host_state;   
   
   always @(posedge clk)
     begin
	if (reset)
	  begin
	     lba0 <= 8'b00000001;
	     lba1 <= 8'b00000001;
	     lba2 <= 8'b00000001;
	     lba3 <= 8'b00000001;
	     sector_count <= 8'b00000001;
	     command <= 8'b00000001;
	     bsy <= 1'b0;
	     sector_ram_ra <= 9'b00000000;
	     sector_ram_wa <= 9'b00000000;
	     host_state <= IDLE;	     
	  end
	else
	  begin
	     if (cs)
	       begin
		  
	       end
	     else
	       begin
		  host_state <= IDLE;		  
	       end
	  end	
     end
   

endmodule // taskfile
