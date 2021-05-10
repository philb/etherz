module ide(a, d, cs1, cs2, clk, irq, rnw, reset);
   input [13:2] a;
   inout [7:0] 	d;
   input 	cs;
   input 	clk;
   output 	irq;

   reg 		host_irq_requested;
   reg 		bsy;
   reg [6:0] 	stat_reg;

   reg [7:0] command;
   reg [7:0] lba0;
   reg [7:0] lba1;
   reg [7:0] lba2;
   reg [7:0] lba3;
   reg [7:0] sector_count;

   reg [7:0] sector_ram_a;
   wire [15:0] sector_ram_d;

   wire [7:0]  cs1_dout;
   wire [7:0]  cs2_dout;

   reg [7:0]   hidata;

   wire [7:0]  status;
   assign status[7:0] = { bsy, stat_reg[6:0] };

   assign cs1_dout = bsy ? status :
		     (a[4:2] == 3'b000) ? sector_ram_d :
		     (a[4:2] == 3'b001) ? error_reg :
		     (a[4:2] == 3'b010) ? sector_count :
		     (a[4:2] == 3'b011) ? lba0 :
		     (a[4:2] == 3'b100) ? lba1 :
		     (a[4:2] == 3'b101) ? lba2 :
		     (a[4:2] == 3'b110) ? lba3 :
		     status;
   assign cs2_dout = bsy ? 8'hb5 : hidata[7:0];
   assign d = (!cs1 && rnw) ? (cs1_dout) : ((!cs2 & rnw) ? (cs2_dout) : 8'bzzzzzzzz);

   SB_RAM256x16 sector_ram (.RDATA (sector_ram_d),
			   .RADDR (sector_ram_a),
			   .RCLK (clk),
			   .RCLKE (1'b1),
			   .RE (sector_ram_re),
			   .WDATA (sector_ram_d),
			   .WCLK (clk),
			   .WCLKE (1'b1),
			   .WE (sector_ram_we),
			   .WADDR (sector_ram_a));

   parameter IDLE = 2'b00;
   parameter R1 = 2'b01;
   parameter R2 = 2'b10;

   assign sector_ram_re = (cs && rnw && ...);
   assign sector_ram_we = (cs && !rnw && host_state == R1);

   reg [1:0]   host_state;

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
	     sector_ram_a <= 8'b00000000;
	     host_state <= IDLE;
	     hidata <= 8'h50;
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
