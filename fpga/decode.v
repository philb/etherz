// A13 A12 A11 A10
// 0   x   x   x    ROM
// 1   0   0   0    Econet
// 1   0   0   1    Ethernet (A9=CMD)
// 1   0   1   0    IDE
// 1   0   1   1    Interrupt status
// 1   1   0   0    Flash page latch
// 1   1   0   1    UART

module decode(a, rom_cs, econet_cs, ethernet_cs, ide_cs, interrupt_cs, fpl_cs, uart_cs);
   input [13:2] a;
   output 	rom_cs;
   output 	econet_cs;
   output 	ethernet_cs;
   output 	ide_cs;
   output 	interrupt_cs;
   output 	fpl_cs;
   output 	uart_cs;

   assign rom_cs = (a[13] == 1'b0);
   assign econet_cs = (a[13:10] == 4'b1000);
   assign ethernet_cs = (a[13:10] == 4'b1001);
   assign ide_cs = (a[13:10] == 4'b1010);
   assign interrupt_cs = (a[13:10] == 4'b1011);
   assign fpl_cs = (a[13:10] == 4'b1100);
   assign uart_cs = (a[13:10] == 4'b1110);   
   
endmodule // decode
