// Ian Stocks hardware addresses from
// https://stardot.org.uk/forums/viewtopic.php?f=16&t=15205&
//+0x0000 - 0x1FFF ROM window (2 KB)
//+0x2000 ROM page register
//+0x2400 IDE reg 0
//+0x2404 IDE reg 1
//+0x2408 IDE reg 2
//+0x240C IDE reg 3
//+0x2410 IDE reg 4
//+0x2414 IDE reg 5
//+0x2408 IDE reg 6
//+0x241C IDE reg 7
//+0x2800 high byte r/w

// A13 A12 A11 A10
// 0   x   x   x    ROM
// 1   0   0   0    Econet
// 1   0   x   1    IDE cmd file
// 1   0   1   0    IDE high byte
// 1   1   0   0    Flash page latch
// 1   1   0   1    UART
// 1   1   1   0    Ethernet (A9=CMD)
// 1   1   1   1    Interrupt status and mask

module decode(a, rom_cs, econet_cs, ethernet_cs, ide_cs, ide2_cs, interrupt_cs, fpl_cs, uart_cs);
   input [13:2] a;
   output 	rom_cs;
   output 	econet_cs;
   output 	ethernet_cs;
   output 	ide_cs;
   output	ide2_cs;
   output 	interrupt_cs;
   output 	fpl_cs;
   output 	uart_cs;

   assign rom_cs =            (a[13] == 1'b0);
   assign econet_cs =         (a[13:10] == 4'b1000);
   assign ide_cs =            (a[13:10] == 4'b1001 || a[13:10] == 4'b1011);
   assign ide2_cs =           (a[13:10] == 4'b1010);
   assign fpl_cs =            (a[13:10] == 4'b1100);
   assign uart_cs =           (a[13:10] == 4'b1101);
   assign ethernet_cs =       (a[13:10] == 4'b1110);
   assign interrupt_cs =      (a[13:10] == 4'b1111);
   
endmodule // decode
