module interrupts(irq, fiq, econet_fiq, ethernet_irq, ide_irq, uart_tx_irq, uart_rx_irq, D, A, cs, re, we, reset);
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
   input 	reset;

   assign irq = ((ethernet_irq && mask[6]) || (ide_irq && mask[5]) || (uart_tx_irq && mask[4]) || (uart_rx_irq && mask[3]));
   assign fiq = econet_fiq && mask[7];

   wire [7:0] status;
   assign status[7:0] = { econet_fiq, ethernet_irq, ide_irq, uart_tx_irq, uart_rx_irq, 1'b0, fiq, irq };

   wire       mask_cs;
   wire       status_cs;
   wire       soft_reset_cs;

   assign mask_cs = cs && (A[3:2] == 2'b01);
   assign status_cs = cs && (A[3:2] == 2'b00);
   assign soft_reset_cs = cs && (A[3:2] == 2'b11);

   reg [7:3]  mask;
   always @(negedge (mask_cs && we) or posedge reset)
     begin
	if (reset)
	  begin
	     mask[7:3] <= 5'b11111;
	  end
	else
	  begin
	     mask[7:3] <= D[7:3];
	  end
     end

   assign D[7:0] = (status_cs && re) ? status[7:0] : (mask_cs && re) ? { mask[7:3], 3'b000 } : 8'bzzzzzzzz;

endmodule // interrupts
