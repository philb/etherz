module top(A, nWE, nRE, nIOC_SEL, nECONET_FIQ, REF8M, RnW,
	   D, SERIAL_CTS, IRQ, SERIAL_RTS, nOE_ADDR, SERIAL_RI, SERIAL_DCD, nOE_DATAIN, nOE_DATAOUT, BL,
	   FIQ, nETH_CS, nETH_IRQ, ETH_CMD, nETH_WE, nETH_RE, RST, nIO_RESET, nMCU_RESET, MCU_IRQ,
	   nECONET_SEL, nFLASH_CE, IOGT, FPGA_DEBUG3, nMS1, SERIAL_DSR, SERIAL_DTR, FPGA_CLK, FPGA_SS2,
	   FLASH_A, SERIAL_RXD, SERIAL_TXD, FPGA_CLK2, SERIAL_INVALID, FPGA_DEBUG1, FPGA_DEBUG2,
	   nFPGA_SS, FPGA_SCK, FPGA_SDI, FPGA_SDO);

   input [13:2]   A;
   input 	  nWE;
   input 	  nRE;
   input 	  nIOC_SEL;
   input 	  nECONET_FIQ;
   input 	  REF8M;
   input 	  RnW;
   inout [7:0] 	  D;
   input 	  SERIAL_CTS;
   output 	  IRQ;
   output 	  SERIAL_RTS;
   output 	  nOE_ADDR;
   input 	  SERIAL_RI;
   input 	  SERIAL_DCD;
   output 	  nOE_DATAIN;
   output 	  nOE_DATAOUT;
   output 	  BL;
   output 	  FIQ;
   output 	  nETH_CS;
   output 	  nETH_IRQ;
   output 	  ETH_CMD;
   output 	  nETH_WE;
   output 	  nETH_RE;
   input 	  RST;
   output 	  nIO_RESET;
   output 	  nMCU_RESET;
   output 	  MCU_IRQ;
   output 	  nECONET_SEL;
   output 	  nFLASH_CE;
   output 	  IOGT;
   output 	  FPGA_DEBUG3;
   input 	  nMS1;
   input 	  SERIAL_DSR;
   output 	  SERIAL_DTR;
   input 	  FPGA_CLK;
   input 	  FPGA_SS2;
   output [18:13] FLASH_A;
   input 	  SERIAL_RXD;
   output 	  SERIAL_TXD;
   input 	  FPGA_CLK2;
   input 	  SERIAL_INVALID;
   output 	  FPGA_DEBUG1;
   output 	  FPGA_DEBUG2;
   input 	  nFPGA_SS;
   input 	  FPGA_SCK;
   input 	  FPGA_SDI;
   output 	  FPGA_SDO;

   reg [5:0] 	  reset_counter;
   wire 	  soft_reset;
   wire 	  ext_reset;
   wire 	  initial_reset;

   // main clock
   wire 	  clk;
   assign clk = FPGA_CLK;

   assign initial_reset = !(&reset_counter);
   always @(posedge clk)
     begin
	if (initial_reset)
	  reset_counter <= reset_counter + 1;
     end

   assign ext_reset = !RST;
   assign soft_reset = 1'b0;

   assign reset = ext_reset || initial_reset || soft_reset;

   wire 	  rom_cs, econet_cs, ethernet_cs, ide_cs, ide2_cs, interrupt_cs, fpl_cs, uart_cs;
   wire 	  ide_irq, uart_tx_irq, uart_rx_irq;

   decode decode_(A, rom_cs, econet_cs, ethernet_cs, ide_cs, ide2_cs, interrupt_cs, fpl_cs, uart_cs);

   // externally visible chip selects
   assign nFLASH_CE = !(rom_cs && !nIOC_SEL);
   assign nECONET_SEL = !(econet_cs && !nIOC_SEL);
   assign nETH_CS = !(ethernet_cs && !nIOC_SEL);
   assign ETH_CMD = A[9];
   assign nETH_RE = nRE;
   assign nETH_WE = nWE;

   reg re, we;
   reg re2, we2;
   always @(posedge clk)
     begin
	if (reset)
	  begin
	     re <= 1'b0;
	     re2 <= 1'b0;
	     we <= 1'b0;
	     we2 <= 1'b0;
	  end
	else
	  begin
	     re2 <= !nRE;
	     we2 <= !nWE;
	     re <= re2;
	     we <= we2;
	  end
     end

   fpl fpl_(fpl_cs && !nIOC_SEL, we, D, FLASH_A[18:13], clk, reset);
   interrupts interrupts_(IRQ, FIQ, !nECONET_FIQ, !nETH_IRQ, ide_irq, uart_tx_irq, uart_rx_irq, D, A, interrupt_cs && IOC_SEL, re, we, reset);

   ide ide_(A, D, ide_cs && !nIOC_SEL, ide2_cs && !nIOC_SEL, clk, ide_irq, re, we, reset, nFPGA_SS, FPGA_SS2, FPGA_SCK, FPGA_SDI, FPGA_SDO, MCU_IRQ);

   assign nIO_RESET = 1'b1;

   assign nOE_DATAIN = nIOC_SEL || nWE;
   assign nOE_DATAOUT = nIOC_SEL || nRE;
   assign nOE_ADDR = 1'b0;

endmodule // top
