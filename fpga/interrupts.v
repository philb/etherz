module interrupts(irq, fiq, econet_fiq, ethernet_irq, ide_irq, uart_tx_irq, uart_rx_irq, D, A, cs, re, we, reset);
   input econet_fiq;
   input ethernet_irq;
   input ide_irq;
   input uart_rx_irq;
   input uart_tx_irq;

   output irq;
   output fiq;

   inout [7:0]  D;
   input [13:2] A;
   input 	cs;
   input 	re;
   input 	we;
   input 	reset;

   assign irq = ((ethernet_irq && mask[6]) || (ide_irq && mask[5]) || (uart_tx_irq && mask[4]) || (uart_rx_irq && mask[3])) || mask[2];
   assign fiq = econet_fiq && mask[7];

   wire [7:0] status;
   assign status[7:0] = { econet_fiq, ethernet_irq, ide_irq, uart_tx_irq, uart_rx_irq, 1'b1, fiq, irq };

   wire       mask_cs;
   wire       status_cs;
   wire       soft_reset_cs;
   wire	      ide_mask_cs;

   assign mask_cs = cs && (A[3:2] == 2'b01);
   assign status_cs = cs && (A[3:2] == 2'b00);
   assign ide_mask_cs = cs && (A[3:2] == 2'b10);
   assign soft_reset_cs = cs && (A[3:2] == 2'b11);

   reg [7:2]  mask;
   always @(negedge we or posedge reset)
     begin
	if (reset)
	  begin
	     mask[7:2] <= 6'b000000;
	  end
	else
	  begin
	     if (mask_cs)
	       mask[7:2] <= D[7:2];
	     else if (ide_mask_cs)
	       mask[5] <= D[5];
	  end
     end

   assign D[7:0] = (status_cs && re) ? status : (mask_cs && re) ? { mask[7:2], 2'b00 } : 8'bzzzzzzzz;

endmodule // interrupts
