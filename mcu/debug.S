#if 1 //def USE_SWD_DEBUG

	.thumb_func
debug_put_digit:
	add	r0, #'0'
	cmp	r0, #10 + '0'
	blt	1f
	add	r0, #'a' - 10 - '0'
1:

	.globl	debug_putc
	.thumb_func
debug_putc:
	push 	{r0}
	mov 	r0, #3
	mov	r1, sp
	bkpt	0xab
	pop	{r0}
	bx	lr

	.globl	debug_put_hex
	.thumb_func
debug_put_hex:
	push	{lr}
	mov	r2, #255
	and	r0, r2
	mov	r2, r0
	asr	r0, #4
	bl	debug_put_digit
	mov	r0, r2
	mov	r2, #15
	and	r0, r2
	bl	debug_put_digit
	pop	{pc}

	.globl debug_put_hex32
	.thumb_func
debug_put_hex32:
	push	{lr}
	mov	r3, r0
	lsr	r0, #24
	bl	debug_put_hex
	mov	r0, r3
	lsr	r0, #16
	bl	debug_put_hex
	mov	r0, r3
	lsr	r0, #8
	bl	debug_put_hex
	mov	r0, r3
	bl	debug_put_hex
	pop	{pc}

	.globl	debug_puts
	.thumb_func
debug_puts:
	push	{r4, lr}
	mov	r4, r0
1:
	ldrb	r0, [r4]
	cmp	r0, #0
	beq	2f
	bl	debug_putc
	add	r4, #1
	b	1b
2:
	pop	{r4, pc}

	.globl	HardFault_Handler
	.thumb_func
HardFault_Handler:
	adr	r0, hfstr
	bl	debug_puts
	ldr	r0, [sp, #0x14]
	bl	debug_put_hex32
	mov	r0, #' '
	bl	debug_putc
	ldr	r0, [sp, #0x18]
	bl	debug_put_hex32
	mov	r0, #' '
	bl	debug_putc
	ldr	r0, [sp, #0x1C]
	bl	debug_put_hex32
1:
	b	1b

	.align
hfstr:
	.asciz	"HardFault "
	.align

#endif
