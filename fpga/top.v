module top(A, nWE, nIOC_SEL, nECONET_FIQ, REF8M, RnW, 
	   D, SERIAL_CTS, IRQ, SERIAL_RTS, nOE_ADDR, SERIAL_RI, SERIAL_DCD, nOE_DATAIN, nOE_DATAOUT, BL, 
	   FIQ, nETH_CS, nETH_IRQ, ETH_CMD, nETH_WE, nETH_RE, RST, nIO_RESET, nMCU_RESET, nMCU_IRQ,
	   nECONET_SEL, nFLASH_CE, IOGT, FPGA_DEBUG3, nMS1, SERIAL_DSR, SERIAL_DTR, FPGA_CLK, FPGA_SS2,
	   FLASH_A, SERIAL_RXD, SERIAL_TXD, FPGA_CLK2, SERIAL_INVALID, FPGA_DEBUG1, FPGA_DEBUG2,
	   nFPGA_SS, FPGA_SCK, FPGA_SDI, FPGA_SDO);

   input [13:2]   A;
   input 	  nWE;
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
   output 	  nMCU_IRQ;
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

   // main clock
   wire 	  clk;
   assign clk = FPGA_CLK;   
   
   wire 	  rom_cs, econet_cs, ethernet_cs, ide_cs, interrupt_cs, fpl_cs, uart_cs;
   wire 	  ide_irq, uart_tx_irq, uart_rx_irq;	  

   decode decode_(A, rom_cs, econet_cs, ethernet_cs, ide_cs, interrupt_cs, fpl_cs, uart_cs);
   
   // externally visible chip selects
   assign nFLASH_CE = !(rom_cs && !IOC_SEL);
   assign nECONET_SEL = !(econet_cs && !IOC_SEL);
   assign nETH_CS = !(ethernet_cs && !IOC_SEL);
   assign ETH_CMD = A[9];
   assign nETH_RE = nRE;
   assign nETH_WE = nWE;
   
   fpl fpl_(fpl_cs && IOC_SEL, nWE, D, FLASH_A, clk);
   interrupts interrupts_(IRQ, FIQ, !nECONET_FIQ, !nETH_IRQ, ide_irq, uart_tx_irq, uart_rx_irq, D, A, interrupt_cs && IOC_SEL, !nRE, !nWE);
           
endmodule // top
