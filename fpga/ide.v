module ide(a, d, cs1, cs2, clk, irq, re, we, reset, mcu_ss1, mcu_ss2, mcu_sclk, mcu_mosi, mcu_miso, mcu_irq);
   input [13:2] a;
   inout [7:0] 	d;
   input 	cs1, cs2;
   input 	clk;
   input 	re, we;
   input 	reset;

   output 	irq;

   input 	mcu_ss1, mcu_ss2;
   input 	mcu_sclk;
   input 	mcu_mosi;
   output 	mcu_miso;
   output 	mcu_irq;

   reg 		host_irq_requested;
   reg 		bsy;
   reg [5:0] 	stat_reg;
   reg 		drq;

   reg [7:0] 	command;
   reg [7:0] 	lba0;
   reg [7:0] 	lba1;
   reg [7:0] 	lba2;
   reg [7:0] 	lba3;
   reg [7:0] 	sector_count;
   reg [7:0] 	features;
   reg [7:0] 	error;

   reg [7:0] 	sector_ram_a;
   wire [15:0] 	sector_ram_din, sector_ram_dout;

   wire [7:0] 	cs1_dout;
   wire [7:0] 	cs2_dout;

   reg [7:0] 	hidata;

   wire [7:0] 	status;
   assign status[7:0] = { bsy, stat_reg[5:3], drq, stat_reg[2:0] };

   reg [2:0] 	reg_a;

   reg 		wrote_command;
   reg 		mcu_irq;
   reg 		write_cmd_pending;

   assign cs1_dout = bsy ? status :
		     (a[4:2] == 3'b000) ? sector_ram_dout[7:0] :
		     (a[4:2] == 3'b001) ? error :
		     (a[4:2] == 3'b010) ? sector_count :
		     (a[4:2] == 3'b011) ? lba0 :
		     (a[4:2] == 3'b100) ? lba1 :
		     (a[4:2] == 3'b101) ? lba2 :
		     (a[4:2] == 3'b110) ? lba3 :
		     status;
   assign cs2_dout = bsy ? 8'hb5 : hidata[7:0];
   assign d = (cs1 && re) ? (cs1_dout) : ((cs2 & re) ? (cs2_dout) : 8'bzzzzzzzz);

   SB_RAM256x16 sector_ram (.RDATA (sector_ram_dout),
			   .RADDR (sector_ram_a),
			   .RCLK (clk),
			   .RCLKE (1'b1),
			   .RE (sector_ram_re),
			   .WDATA (sector_ram_din),
			   .WCLK (clk),
			   .WCLKE (1'b1),
			   .WE (sector_ram_we),
			   .WADDR (sector_ram_a));

   // host interface state
   parameter IDLE = 3'b00;
   parameter R2 = 3'b010;
   parameter W2 = 3'b100;

   wire [15:0] 	mcu_d;

   assign sector_ram_din[15:0] = (host_state == IDLE && we && cs1 && a[4:2] == 3'b000) ? { hidata[7:0], d[7:0] } : mcu_d[15:0];
   assign sector_ram_re = 1'b1;
   assign sector_ram_we = (host_state == IDLE && we && cs1 && a[4:2] == 3'b000);

   reg [2:0]   host_state;

   always @(posedge clk or posedge reset)
     begin
	if (reset)
	  begin
	     lba0 <= 8'b00000001;
	     lba1 <= 8'b00000010;
	     lba2 <= 8'b00000011;
	     lba3 <= 8'b00000100;
	     sector_count <= 8'b00000101;
	     command <= 8'b00000000;
	     bsy <= 1'b0;
	     drq <= 1'b0;
	     stat_reg <= 6'b000000;
	     error <= 8'b00000000;
	     features <= 8'b00000000;
	     sector_ram_a <= 8'b00000000;
	     host_state <= IDLE;
	     hidata <= 8'h50;
	     wrote_command <= 1'b0;
	     mcu_irq <= 1'b0;
	     write_cmd_pending <= 1'b0;
	  end
	else
	  begin
	     case (host_state)
	       IDLE:
		 if (cs1 && !bsy && (re || we))
		   begin
		      wrote_command <= (we && (a[4:2] == 3'b111));
		      host_state <= re ? R2 : W2;
		      if (re)
			begin
			   if (reg_a[2:0] == 3'b000)
			     begin
				hidata[7:0] <= sector_ram_dout[15:8];
				sector_ram_a <= sector_ram_a + 1;
			     end
			end
		      else
			begin
			   // If we are writing the data register, we assert WE into the RAM during this cycle
			   case (a[4:2])
			     3'b000:
			       begin
				  if (sector_ram_a == 8'hff)
				    begin
				       if (write_cmd_pending)
					 begin
					    bsy <= 1'b1;
					    mcu_irq <= 1'b1;
					 end
				       drq <= 1'b0;
				       write_cmd_pending <= 1'b0;
				    end
				  else
				    begin
				       sector_ram_a <= sector_ram_a + 1;
				    end
			       end
			     3'b001:
			       features <= d[7:0];
			     3'b010:
			       sector_count <= d[7:0];
			     3'b011:
			       lba0 <= d[7:0];
			     3'b100:
			       lba1 <= d[7:0];
			     3'b101:
			       lba2 <= d[7:0];
			     3'b110:
			       lba3 <= d[7:0];
			     3'b111:
			       begin
				  command <= d[7:0];
				  wrote_command <= 1'b1;
			       end
			   endcase
			end
		   end

	       R2:
		 begin
		    if (!cs1)
		      begin
			 host_state <= IDLE;
		      end
		 end

	       W2:
		 begin
		    if (!cs1)
		      begin
			 host_state <= IDLE;
			 if (wrote_command)
			   begin
			      sector_ram_a <= 8'h00;
			      case (command)
				8'h50 /* Format */,
				8'h30 /* Write sectors */,
				8'h31 /* Write sector no retry */:
				  begin
				     // Class 2 commands that require data from the host.
				     // Assert DRQ and wait for the host to fill the buffer.
				     drq <= 1'b1;
				     write_cmd_pending <= 1'b1;
				  end
				8'he8 /* Write buffer */,
				8'he4 /* Read buffer */:
				  begin
				     // We can service this command entirely in the FPGA
				     drq <= 1'b1;
				  end
				default:
				  begin
				     // Other commands require attention from MCU first
				     bsy <= 1'b1;
				     mcu_irq <= 1'b1;
				  end
			      endcase
			   end
		      end
		 end
	     endcase
	  end
     end


endmodule // taskfile
