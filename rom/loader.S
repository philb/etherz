@       Input/Output     Comments	
@       ------------     --------
@
@ R0    Write/Read data  Treated as a byte
@ R1    Address          Must be preserved
@ R2-R3                  May be used
@ R4-R9                  Must be preserved
@ R10                    May be used
@ R11   Hardware         Hardware base address, must be preserved
@ R12                    Private, must be preserved
@ R13   sp               Stack pointer (FD), must be preserved
@ R14                    Return address; use BICS pc, lr, #V_bit 
@ R15                    PC

	#define V_bit	1<<28
	#define page_shift	11
	#define fpl_addr	0x3000
	
	b	read_byte
	b	write_byte
	b	reset
	bics	pc, lr, #V_bit

read_byte:
	@ mask off hardware address from combined address and clear speed bits
	ldr	r3, =0b00000011111001111111000000000000
	and	r10, r11, r3
	@ select fast cycles
	orr	r10, r10, #0b000000000000100000000000000000000
	cmp 	r1, #(128 * 1024) - (1 << page_shift)
	bhs	out_of_range
	mov	r2, r1, lsr #page_shift
	add	r2, r2, #1
	mov	r3, #fpl_addr
	strb	r2, [r10, r3]
	bic	r2, r1, #(0x7f << page_shift)
	ldrb	r0, [r10, r2, lsl #2]
	bics	pc, lr, #V_bit

	.ltorg

out_of_range:
	adr	r0, 1f
	orrs	pc, lr, #V_bit
1:	.word	0x584
	.asciz	"Address out of range"
	.align

write_byte:
	adr	r0, 1f
	orrs	pc, lr, #V_bit
1:	.word	0x580
	.asciz	"Not writeable"
	.align

reset:
	ldr	r3, =0b00000011111111111111000000000000
	and	r10, r11, r3
	add	r3, r10, #fpl_addr
	mov	r2, #0
	strb	r2, [r3]
	bics	pc, lr, #V_bit
