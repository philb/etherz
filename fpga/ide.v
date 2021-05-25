module ide(a, d, cs1, cs2, clk, host_irq_out, re, we, reset, mcu_ss1, mcu_ss2, mcu_sclk, mcu_mosi, mcu_miso, mcu_irq, ide_obs);
   input [13:2] a;
   inout [7:0] 	d;
   input 	cs1, cs2;
   input 	clk;
   input 	re, we;
   input 	reset;

   output 	host_irq_out;

   input 	mcu_ss1, mcu_ss2;
   input 	mcu_sclk;
   input 	mcu_mosi;
   output 	mcu_miso;
   output 	mcu_irq;
   output 	ide_obs;

   reg 		host_irq;
   reg 		bsy;
   reg [5:0] 	stat_reg;
   reg 		drq;

   reg [7:0] 	command;
   reg [7:0] 	lba0;
   reg [7:0] 	lba1;
   reg [7:0] 	lba2;
   reg [7:0] 	dhr;
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

   wire [7:0] 	sector_ram_wa;

   reg 		wrote_command;
   reg 		read_data;
   reg 		mcu_irq;
   reg 		write_cmd_pending;

   reg [15:0] 	mcu_data_latch;
   reg 		mcu_data_latch_full;
   reg 		mcu_data_latch_full_2;
   reg 		mcu_data_latch_full_3;
   reg [8:0] 	mcu_data_latch_address;

   wire [3:0] 	reg_a;
   assign reg_a[3:0] = { a[11], a[4:2] };

   reg 		host_irq_en;
   assign host_irq_out = host_irq /*&& host_irq_en*/;

   wire [7:0] 	drive_addr;
   assign drive_addr[7:0] = { 1'bz, 6'b111111, 1'b0 };

   assign cs1_dout = bsy ? status :
		     (reg_a == 4'b0000) ? sector_ram_dout[7:0] :
		     (reg_a == 4'b0001) ? error :
		     (reg_a == 4'b0010) ? sector_count :
		     (reg_a == 4'b0011) ? lba0 :
		     (reg_a == 4'b0100) ? lba1 :
		     (reg_a == 4'b0101) ? lba2 :
		     (reg_a == 4'b0110) ? dhr :
		     (reg_a == 4'b1111) ? drive_addr :
		     (reg_a == 4'b1000) ? command :
		     (reg_a == 4'b1001) ? sector_ram_a :
		     (reg_a == 4'b1010) ? { 6'b000000, wrote_command, write_cmd_pending } :
		     status;
   assign cs2_dout = hidata[7:0];
   assign d = (cs1 && re) ? (cs1_dout) : ((cs2 && re) ? (cs2_dout) : 8'bzzzzzzzz);

   SB_RAM256x16 sector_ram (.RDATA (sector_ram_dout),
			   .RADDR (sector_ram_a),
			   .RCLK (clk),
			   .RCLKE (1'b1),
			   .RE (sector_ram_re),
			   .WDATA (sector_ram_din),
			   .WCLK (clk),
			   .WCLKE (1'b1),
			   .WE (sector_ram_we),
			   .WADDR (sector_ram_wa));

   // host interface state
   parameter IDLE = 3'b00;
   parameter R2 = 3'b010;
   parameter W2 = 3'b100;
   parameter W3 = 3'b101;

   wire 	mcu_writing_sector;
   assign mcu_writing_sector = (mcu_data_latch_full_2 && !mcu_data_latch_full_3 && bsy && !mcu_data_latch_address[8]);

   assign sector_ram_din[15:0] = (!bsy) ? { hidata[7:0], d[7:0] } : mcu_data_latch[15:0];
   assign sector_ram_re = (host_state == IDLE) || bsy;
   assign sector_ram_we = (host_state == IDLE && we && cs1 && reg_a == 4'b0000 && !bsy) || mcu_writing_sector;

   assign sector_ram_wa = mcu_writing_sector ? mcu_data_latch_address[7:0] : sector_ram_a;

   reg 		mcu_data_req;
   reg 		mcu_data_req_2;
   reg 		mcu_data_req_3;

   reg [2:0] 	host_state;
   reg 		more_reads_pending;

   always @(posedge clk or posedge reset)
     begin
	if (reset)
	  begin
	     lba0 <= 8'b00000001;
	     lba1 <= 8'b00000000;
	     lba2 <= 8'b00000000;
	     dhr <= 8'b10100000;
	     sector_count <= 8'b00000001;
	     command <= 8'b00000000;
	     bsy <= 1'b0;
	     drq <= 1'b0;
	     stat_reg <= 6'b100000;
	     error <= 8'b00000000;
	     features <= 8'b00000000;
	     sector_ram_a <= 8'b00000000;
	     host_state <= IDLE;
	     hidata <= 8'h50;
	     wrote_command <= 1'b0;
	     read_data <= 1'b0;
	     mcu_irq <= 1'b0;
	     host_irq <= 1'b0;
	     host_irq_en <= 1'b0;
	     write_cmd_pending <= 1'b0;
	     more_reads_pending <= 1'b0;
	     mcu_data_latch_full_2 <= 1'b0;
	     mcu_data_latch_full_3 <= 1'b0;
	     mcu_data_req_2 <= 1'b0;
	     mcu_data_req_3 <= 1'b0;
	  end
	else
	  begin
	     mcu_data_latch_full_2 <= mcu_data_latch_full;
	     mcu_data_latch_full_3 <= mcu_data_latch_full_2;

	     if (mcu_data_latch_full_2 && !mcu_data_latch_full_3 && bsy && mcu_data_latch_address[8])
	       begin
		  case (mcu_data_latch_address[7:0])
		    8'b00000000:
		      { lba0, lba1 } <= mcu_data_latch;
		    8'b00000001:
		      { lba2, dhr } <= mcu_data_latch;
		    8'b00000010:
		      { sector_count, error } <= mcu_data_latch;
		    8'b00000011:
		      begin
			 { more_reads_pending, write_cmd_pending, stat_reg, drq, host_irq } <= mcu_data_latch[9:0];
			 bsy <= 1'b0;
			 mcu_irq <= 1'b0;
			 wrote_command <= 1'b0;
			 sector_ram_a <= 8'b00000000;
		      end
		  endcase
	       end // if (mcu_data_latch_full_2 && !mcu_data_latch_full_3 && bsy && mcu_data_latch_address[8])

	     mcu_data_req_2 <= mcu_data_req;
	     mcu_data_req_3 <= mcu_data_req_2;
	     if (mcu_data_req_2 && !mcu_data_req_3)
	       begin
		  sector_ram_a <= sector_ram_a + 1;
	       end

	     case (host_state)
	       IDLE:
		 if (cs1 && !bsy && (re || we))
		   begin
		      wrote_command <= (we && (reg_a == 4'b0111));
		      read_data <= (re && (reg_a == 4'b0000));
		      host_state <= re ? R2 : W2;
		      if (re)
			begin
			   if (reg_a == 4'b0111)
			     begin
				// Reading status (not alt status) clears irq
				host_irq <= 1'b0;
			     end
			end
		      else
			begin
			   // If we are writing the data register, we assert WE into the RAM during this cycle
			   case (reg_a)
			     4'b0000:
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
				  sector_ram_a <= sector_ram_a + 1;
			       end
			     4'b0001:
			       features <= d[7:0];
			     4'b0010:
			       sector_count <= d[7:0];
			     4'b0011:
			       lba0 <= d[7:0];
			     4'b0100:
			       lba1 <= d[7:0];
			     4'b0101:
			       lba2 <= d[7:0];
			     4'b0110:
			       dhr <= d[7:0];
			     4'b0111:
			       begin
				  if (dhr[4] == 0)
				    begin
				       command <= d[7:0];
				       wrote_command <= 1'b1;
				    end
			       end
			     4'b1110:
			       host_irq_en <= !d[1];
			   endcase
			end
		   end // if (cs1 && !bsy && (re || we))
	       else if (cs2 && !bsy && we)
		 begin
		    hidata <= d[7:0];
		    host_state <= W3;
 		 end

	       R2:
		 begin
		    if (!re)
		      begin
			 if (read_data)
			   begin
			      hidata[7:0] <= sector_ram_dout[15:8];
			      if (sector_ram_a == 8'hff)
				begin
				   drq <= 1'b0;
				   mcu_irq <= more_reads_pending;
				   bsy <= more_reads_pending;
				end
			      sector_ram_a <= sector_ram_a + 1;
			   end // if (read_data)

			 host_state <= IDLE;
		      end
		 end

	       W3:
		 begin
		    if (!we)
		      begin
			 host_state <= IDLE;
		      end
		 end

	       W2:
		 begin
		    if (!we)
		      begin
			 host_state <= IDLE;
			 if (wrote_command)
			   begin
			      sector_ram_a <= 8'h00;
			      host_irq <= 1'b0;	// write command clears intrq
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
				     drq <= 1'b0;	// In case previous command was interrupted
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

   wire [63:0] mcu_status_reg;
   assign mcu_status_reg = { bsy, wrote_command, drq, host_irq, 4'b0011, command, lba0, lba1, lba2, dhr, sector_count, features };
   reg [5:0]   mcu_status_bit;
   reg [15:0]  mcu_data_in;
   reg [3:0]   mcu_data_bit;
   reg [3:0]   mcu_data_out_bit;

   reg [3:0]   mcu_big_state;
   parameter MCU_START = 4'b0001;
   parameter MCU_SECTOR_WRITE = 4'b0010;
   parameter MCU_COMMAND_WRITE = 4'b0100;
   parameter MCU_NOTHING = 4'b1000;

   reg [7:0]   mcu_data_out;
   reg [7:0]   mcu_word_count;

   wire [15:0] sector_ram_dout_swizzle;
   assign sector_ram_dout_swizzle[15:0] = { sector_ram_dout[7:0], sector_ram_dout[15:8] };

   assign mcu_miso = !mcu_ss1 ? mcu_status_reg[63 - mcu_status_bit] : !mcu_ss2 ? sector_ram_dout_swizzle[15 - mcu_data_out_bit] : 1'bz;

   always @(negedge mcu_sclk or posedge mcu_ss1 or posedge reset)
     begin
	if (mcu_ss1 || reset)
	  mcu_status_bit <= 6'h3f;
	else
	  mcu_status_bit <= mcu_status_bit + 1;
     end

   reg first_word;

   always @(negedge mcu_sclk or posedge mcu_ss2 or posedge reset)
     begin
	if (mcu_ss2 || reset)
	  begin
	     mcu_data_out_bit <= 4'hf;
	     mcu_data_req <= 1'b0;
	     first_word <= 1'b1;
	  end
	else
	  begin
	     mcu_data_out_bit <= mcu_data_out_bit + 1;
	     first_word <= 1'b0;
	     mcu_data_req <= (mcu_data_out_bit == 4'hf && !first_word) ? 1 : 0;
	  end
     end

   always @(posedge mcu_sclk or posedge mcu_ss2 or posedge reset)
     begin
	if (mcu_ss2 || reset)
	  begin
	     mcu_data_bit <= 4'h0;
	     mcu_big_state <= MCU_START;
	     mcu_data_latch_full <= 1'b0;
	     mcu_data_latch <= 16'h5555;
	     mcu_data_in <= 16'h0000;
	     mcu_word_count <= 8'h00;
	  end
	else
	  begin
	     mcu_data_in <= { mcu_data_in[14:0], mcu_mosi };

	     if (mcu_data_bit == 7)
	       begin
		  case (mcu_big_state)
		    MCU_START:
		      // Bits [7:5], actually [6:4] here because they're pipelined one cycle:
		      // B7=1 sector data write
		      // B6=1 command file write
		      begin
			 if (mcu_data_in[6])
			   mcu_big_state <= MCU_SECTOR_WRITE;
			 else
			   if (mcu_data_in[5])
			     mcu_big_state <= MCU_COMMAND_WRITE;
			   else
			     mcu_big_state <= MCU_NOTHING;
			 mcu_word_count <= 8'h00;
			 mcu_data_bit <= 4'h0;
		      end // case: MCU_START
		    default:
		      mcu_data_bit <= mcu_data_bit + 1;
		  endcase
	       end
	     else if (mcu_data_bit == 4'hf)
	       begin
		  mcu_data_bit <= mcu_data_bit + 1;
		  case (mcu_big_state)
		    MCU_SECTOR_WRITE:
		      begin
			 // Swizzle byte order here to compensate for weird big-endian SPI
			 mcu_data_latch <= { mcu_data_in[6:0], mcu_mosi, mcu_data_in[14:7] };
			 mcu_data_latch_full <= 1'b1;
			 mcu_data_latch_address <= { 1'b0, mcu_word_count };   // sector data in latch
			 mcu_word_count <= mcu_word_count + 1;
			 if (mcu_word_count == 255)
			   mcu_big_state <= MCU_COMMAND_WRITE;
		      end

		    MCU_COMMAND_WRITE:
		      begin
			 mcu_data_latch <= { mcu_data_in[14:0], mcu_mosi };
			 mcu_data_latch_full <= 1'b1;
			 mcu_data_latch_address <= { 1'b1, mcu_word_count };   // command file data in latch
			 mcu_word_count <= mcu_word_count + 1;
			 if (mcu_word_count == 3)
			   mcu_big_state <= MCU_NOTHING;
		      end

		  endcase
	       end // if (mcu_data_bit == 4'hf)
	     else
	       begin
		  mcu_data_latch_full <= 1'b0;
		  mcu_data_bit <= mcu_data_bit + 1;
	       end
	  end
     end

   assign ide_obs = (mcu_big_state[2]);

endmodule // taskfile
