module interrupts(irq, fiq, econet_fiq, ethernet_irq, ide_irq, uart_tx_irq, uart_rx_irq, D, A, cs, re, we);
   input econet_fiq;
   input ethernet_irq;
   input ide_irq;
   input uart_rx_irq;
   input uart_tx_irq;
   
   output irq;
   output fiq;
   
   inout [7:0]  D;
   input [13:0] A;   
   input 	cs;
   input 	re;
   input 	we;

   assign irq = (ethernet_irq || ide_irq || uart_tx_irq || uart_rx_irq);
   assign fiq = econet_fiq;

   wire [7:0] status;
   assign status[7:0] = { econet_fiq, ethernet_irq, ide_irq, uart_tx_irq, uart_rx_irq, 1'b0, fiq, irq };

   assign D[7:0] = (cs && re) ? status[7:0] : 8'bzzzzzzzz;
   	  
endmodule // interrupts
