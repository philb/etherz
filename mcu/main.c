#include "asf.h"
#include "conf_board.h"
#include "spi.h"
#include "pdc.h"
#include "ioport.h"
#include "sd_mmc.h"
#include "delay.h"
#include "wdt.h"
#include "pio.h"
#include <string.h>
#include <stdarg.h>

#define VERBOSE 0
#define SECTORS 64
#define CYLINDERS 4096
#define VERSION "0.2"

static uint8_t fpga_image[] = {
#include "fpga_image.h"
};

/* Chip select. */
#define SPI_CHIP_SEL 0

/* Clock polarity. */
#define SPI_CLK_POLARITY 1

/* Clock phase. */
#define SPI_CLK_PHASE 0

/* Delay before SPCK. */
#define SPI_DLYBS 0

/* Delay between consecutive transfers. */
#define SPI_DLYBCT 0

/* SPI clock setting (Hz). */
static uint32_t gs_ul_spi_clock = 9*1000*1000;

#define FPGA_CRESET_GPIO            (PIO_PA19_IDX)
#define FPGA_CDONE_GPIO             (PIO_PA21_IDX)
#define FPGA_SS_GPIO		    (PIO_PA11_IDX)
#define FPGA_SS2_GPIO		    (PIO_PA10_IDX)

#define FPGA_IRQ_GPIO		    (PIO_PA5_IDX)

extern void debug_puts(const char *);
extern void debug_putc(char);
extern void debug_put_hex(uint8_t);

static int capacity_mb;
static char card_psn[4];
static char card_pnm[7];

struct command_file {
	uint8_t sector_count;
	uint8_t sector_nr;
	uint8_t cyl_low;
	uint8_t cyl_high;
	uint8_t dhr;
};

#define TX_BUFFER_SIZE	1024

static char tx_buffer[TX_BUFFER_SIZE];
static volatile off_t tx_buffer_write, tx_buffer_read;
static volatile bool uart_tx_active;

#define NEXTP(x) ((x+1) % TX_BUFFER_SIZE)

void UART1_Handler(void)
{
	UART1->UART_THR = tx_buffer[tx_buffer_read];
	tx_buffer_read = NEXTP(tx_buffer_read);
	if (tx_buffer_write == tx_buffer_read) {
		uart_disable_interrupt(UART1, UART_IER_TXRDY);
		uart_tx_active = false;
	}
}

static void uart_kick(void)
{
	uart_enable_interrupt(UART1, UART_IER_TXRDY);
	uart_tx_active = true;
}

static void uart_writec(char c)
{
	while (NEXTP(tx_buffer_write) == tx_buffer_read)
		/* Buffer full, wait */
		;
	tx_buffer[tx_buffer_write] = c;
	tx_buffer_write = NEXTP(tx_buffer_write);
	cpu_irq_disable();
	if (!uart_tx_active)
		uart_kick();
	cpu_irq_enable();
}

static void uart_write_buffer(const char *buf, size_t length)
{
	while (length--)
		uart_writec(*(buf++));
}

static void uart_printf(const char *fmt, ...)
{
	va_list ap;
	va_start(ap, fmt);
	size_t sz = 256;
	char buf[sz];
	size_t n = vsnprintf(buf, sz, fmt, ap);
	uart_write_buffer(buf, n);
	va_end(ap);
}

static void load_fpga(void)
{
	ioport_set_pin_mode(FPGA_CRESET_GPIO, 0);
	ioport_set_pin_dir(FPGA_CRESET_GPIO, IOPORT_DIR_OUTPUT);
	ioport_enable_pin(FPGA_CRESET_GPIO);
	ioport_set_pin_level(FPGA_CRESET_GPIO, IOPORT_PIN_LEVEL_LOW);

	ioport_set_pin_mode(FPGA_CDONE_GPIO, IOPORT_MODE_PULLUP);
	ioport_set_pin_dir(FPGA_CDONE_GPIO, IOPORT_DIR_INPUT);
	ioport_enable_pin(FPGA_CDONE_GPIO);

	ioport_set_pin_mode(FPGA_SS_GPIO, 0);
	ioport_set_pin_dir(FPGA_SS_GPIO, IOPORT_DIR_OUTPUT);
	ioport_enable_pin(FPGA_SS_GPIO);
	ioport_set_pin_level(FPGA_SS_GPIO, IOPORT_PIN_LEVEL_LOW);

	ioport_set_pin_mode(PIO_PA12_IDX, IOPORT_MODE_MUX_A);
	ioport_disable_pin(PIO_PA12_IDX);
	ioport_set_pin_mode(PIO_PA13_IDX, IOPORT_MODE_MUX_A);
	ioport_disable_pin(PIO_PA13_IDX);
	ioport_set_pin_mode(PIO_PA14_IDX, IOPORT_MODE_MUX_A);
	ioport_disable_pin(PIO_PA14_IDX);

	Pdc *pdc = spi_get_pdc_base(SPI);

	pmc_enable_periph_clk(ID_SPI);

	spi_disable(SPI);
	spi_reset(SPI);
	spi_set_lastxfer(SPI);
	spi_set_master_mode(SPI);
	spi_disable_mode_fault_detect(SPI);
	spi_disable_tx_on_rx_empty(SPI);
	spi_set_peripheral_chip_select_value(SPI, SPI_CHIP_SEL);
	spi_set_clock_polarity(SPI, SPI_CHIP_SEL, SPI_CLK_POLARITY);
	spi_set_clock_phase(SPI, SPI_CHIP_SEL, SPI_CLK_PHASE);
	spi_set_bits_per_transfer(SPI, SPI_CHIP_SEL,
			SPI_CSR_BITS_8_BIT);
	spi_set_baudrate_div(SPI, SPI_CHIP_SEL,
			(sysclk_get_peripheral_hz() / gs_ul_spi_clock));
	spi_set_transfer_delay(SPI, SPI_CHIP_SEL, SPI_DLYBS,
			SPI_DLYBCT);
	spi_enable(SPI);

	pdc_disable_transfer(pdc, PERIPH_PTCR_RXTDIS | PERIPH_PTCR_TXTDIS);

	ioport_set_pin_level(FPGA_CRESET_GPIO, IOPORT_PIN_LEVEL_HIGH);

	delay_ms(1);

	pdc_packet_t pdc_spi_packet;
	pdc_spi_packet.ul_addr = (uint32_t)&fpga_image[0];
	pdc_spi_packet.ul_size = sizeof(fpga_image);
	pdc_tx_init(pdc, &pdc_spi_packet, NULL);

	/* Enable the TX PDC transfer requests */
	pdc_enable_transfer(pdc, PERIPH_PTCR_TXTEN);

	/* Waiting transfer done*/
	while((spi_read_status(SPI) & SPI_SR_ENDTX) == 0);

	/* Disable the RX and TX PDC transfer requests */
	pdc_disable_transfer(pdc, PERIPH_PTCR_RXTDIS | PERIPH_PTCR_TXTDIS);

	delay_ms(1);

	if (ioport_get_pin_level(FPGA_CDONE_GPIO) == 0)
	{
		debug_puts("No CDONE\n");
		for (;;);
	}

	pdc_spi_packet.ul_addr = (uint32_t)&fpga_image[0];
	pdc_spi_packet.ul_size = 8;
	pdc_tx_init(pdc, &pdc_spi_packet, NULL);

	/* Enable the TX PDC transfer requests */
	pdc_enable_transfer(pdc, PERIPH_PTCR_TXTEN);

	/* Waiting transfer done*/
	while((spi_read_status(SPI) & SPI_SR_ENDTX) == 0);

	/* Disable the RX and TX PDC transfer requests */
	pdc_disable_transfer(pdc, PERIPH_PTCR_RXTDIS | PERIPH_PTCR_TXTDIS);

	ioport_set_pin_level(FPGA_SS_GPIO, IOPORT_PIN_LEVEL_HIGH);

	// Enable clock to FPGA
	ioport_set_pin_mode(PIO_PA6_IDX, IOPORT_MODE_MUX_B);
	ioport_disable_pin(PIO_PA6_IDX);
	pmc_disable_pck(0);
	// MCK = 18.432MHz * 12 = 110.592MHz
	// PCK = MCK/4 = 27.64MHz
	// Fmax for PCK output = 46MHz
	pmc_switch_pck_to_mck(0, 2);
	pmc_enable_pck(0);
}

static void setup_emmc(void)
{
	ioport_set_pin_mode(PIO_PA26_IDX, IOPORT_MODE_MUX_C);
	ioport_disable_pin(PIO_PA26_IDX);
	ioport_set_pin_mode(PIO_PA27_IDX, IOPORT_MODE_MUX_C);
	ioport_disable_pin(PIO_PA27_IDX);
	ioport_set_pin_mode(PIO_PA28_IDX, IOPORT_MODE_MUX_C | IOPORT_MODE_PULLUP);
	ioport_disable_pin(PIO_PA28_IDX);
	ioport_set_pin_mode(PIO_PA29_IDX, IOPORT_MODE_MUX_C);
	ioport_disable_pin(PIO_PA29_IDX);
	ioport_set_pin_mode(PIO_PA30_IDX, IOPORT_MODE_MUX_C | IOPORT_MODE_PULLUP);
	ioport_disable_pin(PIO_PA30_IDX);
	ioport_set_pin_mode(PIO_PA31_IDX, IOPORT_MODE_MUX_C);
	ioport_disable_pin(PIO_PA31_IDX);

	sd_mmc_init();
	uint8_t r = sd_mmc_check(0);

	capacity_mb = sd_mmc_get_capacity(0) / 1024;
	uint8_t *cid = sd_mmc_get_cid(0);

	memcpy(card_psn, cid+10, 4);
	memcpy(card_pnm, cid+3, 6);
	card_pnm[6] = 0;
}

static void get_command_file(uint8_t *rbuf)
{
	Pdc *pdc = spi_get_pdc_base(SPI);

	uint8_t tbuf[8];
	memset(tbuf, 0, sizeof(tbuf));
	memset(rbuf, 0xff, 8);

	pio_disable_interrupt(PIOA, PIO_PA5);

	ioport_set_pin_level(FPGA_SS_GPIO, IOPORT_PIN_LEVEL_LOW);

	spi_get(SPI);

	pdc_packet_t pdc_spi_packet;

	pdc_spi_packet.ul_addr = (uint32_t)rbuf;
	pdc_spi_packet.ul_size = 8;
	pdc_rx_init(pdc, &pdc_spi_packet, NULL);

	pdc_spi_packet.ul_addr = (uint32_t)tbuf;
	pdc_spi_packet.ul_size = sizeof(tbuf);
	pdc_tx_init(pdc, &pdc_spi_packet, NULL);

	/* Enable the RX and TX PDC transfer requests */
	pdc_enable_transfer(pdc, PERIPH_PTCR_RXTEN | PERIPH_PTCR_TXTEN);

	/* Waiting transfer done*/
	while (pdc_read_rx_counter(pdc) != 0);

	/* Disable the RX and TX PDC transfer requests */
	pdc_disable_transfer(pdc, PERIPH_PTCR_RXTDIS | PERIPH_PTCR_TXTDIS);

	ioport_set_pin_level(FPGA_SS_GPIO, IOPORT_PIN_LEVEL_HIGH);
}

static int debug_put(void volatile *x, char c)
{
	debug_putc(c);
	return 0;
}

static void finish_ide_command(void)
{
#if VERBOSE
	uint8_t rbuf[8];
	get_command_file(rbuf);
	uart_printf(" : %02x %02x %02x %02x %02x %02x %02x %02x\r\n",
		    rbuf[0], rbuf[1], rbuf[2], rbuf[3],
		    rbuf[4], rbuf[5], rbuf[6], rbuf[7]);
#else
	uart_printf("\r\n");
#endif
	pio_enable_interrupt(PIOA, PIO_PA5);
}

static uint8_t sector_buffer[512];

#define HOST_IRQ_BIT	0x1
#define DRQ_BIT		0x2
#define ERR_BIT		0x4
#define DRDY_BIT	0x80

static void fill_status_buffer(uint8_t *tbuf, struct command_file *cf, bool drq, bool error, bool more_writes, bool more_reads)
{
	// not swizzled, so byte 0 ends up in mcu_data[15:8]
	tbuf[0] = cf->sector_nr;
	tbuf[1] = cf->cyl_low;
	tbuf[2] = cf->cyl_high;
	tbuf[3] = cf->dhr;
	tbuf[4] = cf->sector_count;
	tbuf[5] = error;
	tbuf[6] = (more_writes ? 1 : 0) | (more_reads ? 2 : 0);
	tbuf[7] = DRDY_BIT | HOST_IRQ_BIT | (drq ? DRQ_BIT : 0) | (error ? ERR_BIT : 0);
#if VERBOSE
	uart_printf("-> [%02x %02x %02x %02x %02x %02x %02x %02x] %d/%d/%d %s%s%s",
		    tbuf[0], tbuf[1], tbuf[2], tbuf[3], tbuf[4], tbuf[5], tbuf[6], tbuf[7],
		    cf->sector_nr, (cf->cyl_low | (cf->cyl_high << 8)), cf->dhr & 15,
		    drq ? "DRQ " : "", error ? "ERR " : "", (more_reads | more_writes) ? "more " : "");
#endif
}

static void return_status(struct command_file *cf, bool drq, bool error, bool more_writes, bool more_reads)
{
	Pdc *pdc = spi_get_pdc_base(SPI);

	uint8_t tbuf[9];
	memset(tbuf, 0, sizeof(tbuf));

	tbuf[0] = 0x40;
	fill_status_buffer(tbuf + 1, cf, drq, error, more_writes, more_reads);

	pdc_packet_t pdc_spi_packet;
	pdc_spi_packet.ul_addr = (uint32_t)tbuf;
	pdc_spi_packet.ul_size = sizeof(tbuf);
	pdc_tx_init(pdc, &pdc_spi_packet, NULL);

	ioport_set_pin_level(FPGA_SS2_GPIO, IOPORT_PIN_LEVEL_LOW);

	/* Enable the RX and TX PDC transfer requests */
	pdc_enable_transfer(pdc, PERIPH_PTCR_TXTEN);

	/* Waiting transfer done*/
	while ((pdc_read_tx_counter(pdc) != 0) || (spi_is_tx_empty(SPI) != 0));

	delay_ms(1);

	/* Disable the RX and TX PDC transfer requests */
	pdc_disable_transfer(pdc, PERIPH_PTCR_TXTDIS);
	ioport_set_pin_level(FPGA_SS2_GPIO, IOPORT_PIN_LEVEL_HIGH);
}

static void fill_sector_buffer(void)
{
	Pdc *pdc = spi_get_pdc_base(SPI);

	uint8_t tbuf[1+sizeof(sector_buffer)];
	memset(tbuf, 0, sizeof(tbuf));
	memset(sector_buffer, 0xff, sizeof(sector_buffer));

	pdc_packet_t pdc_spi_packet;
	pdc_spi_packet.ul_addr = (uint32_t)sector_buffer;
	pdc_spi_packet.ul_size = sizeof(sector_buffer);
	pdc_rx_init(pdc, &pdc_spi_packet, NULL);

	pdc_spi_packet.ul_addr = (uint32_t)tbuf;
	pdc_spi_packet.ul_size = sizeof(tbuf);
	pdc_tx_init(pdc, &pdc_spi_packet, NULL);

	ioport_set_pin_level(FPGA_SS2_GPIO, IOPORT_PIN_LEVEL_LOW);

	/* Enable the RX and TX PDC transfer requests */
	pdc_enable_transfer(pdc, PERIPH_PTCR_RXTEN | PERIPH_PTCR_TXTEN);

	/* Waiting transfer done */
	while (pdc_read_rx_counter(pdc) != 0);

	/* Disable the RX and TX PDC transfer requests */
	pdc_disable_transfer(pdc, PERIPH_PTCR_RXTDIS | PERIPH_PTCR_TXTDIS);
	ioport_set_pin_level(FPGA_SS2_GPIO, IOPORT_PIN_LEVEL_HIGH);
}

static void return_sector_buffer(struct command_file *cf, int error, int more_reads)
{
	Pdc *pdc = spi_get_pdc_base(SPI);

	uint8_t tbuf[512+9], rbuf[512+9];
	memset(tbuf, 0, sizeof(tbuf));
	memset(rbuf, 0xff, sizeof(rbuf));

	tbuf[0] = 0xc0;
	memcpy(tbuf + 1, sector_buffer, 512);
	fill_status_buffer(tbuf + 512 + 1, cf, true, error, false, more_reads);

	pdc_packet_t pdc_spi_packet;
	pdc_spi_packet.ul_addr = (uint32_t)rbuf;
	pdc_spi_packet.ul_size = sizeof(rbuf);
	pdc_rx_init(pdc, &pdc_spi_packet, NULL);

	pdc_spi_packet.ul_addr = (uint32_t)tbuf;
	pdc_spi_packet.ul_size = sizeof(tbuf);
	pdc_tx_init(pdc, &pdc_spi_packet, NULL);

	ioport_set_pin_level(FPGA_SS2_GPIO, IOPORT_PIN_LEVEL_LOW);

	/* Enable the RX and TX PDC transfer requests */
	pdc_enable_transfer(pdc, PERIPH_PTCR_RXTEN | PERIPH_PTCR_TXTEN);

	/* Waiting transfer done*/
	while (pdc_read_rx_counter(pdc) != 0);

	/* Disable the RX and TX PDC transfer requests */
	pdc_disable_transfer(pdc, PERIPH_PTCR_RXTDIS | PERIPH_PTCR_TXTDIS);
	ioport_set_pin_level(FPGA_SS2_GPIO, IOPORT_PIN_LEVEL_HIGH);
}

static void swizzle_string(uint8_t *buf, const char *in, int size)
{
	memset(buf, ' ', size);
	memcpy(buf, in, strlen(in));
	for (int i = 0; i < size; i += 2) {
		uint8_t a = buf[i];
		buf[i] = buf[i+1];
		buf[i+1] = a;
	}
}

static void unpack_command(struct command_file *cf, uint8_t *cmd)
{
	cf->sector_nr = cmd[2];
	cf->cyl_low = cmd[3];
	cf->cyl_high = cmd[4];
	cf->dhr = cmd[5];
	cf->sector_count = cmd[6];
}

static uint32_t get_sector(struct command_file *cf)
{
	return (cf->sector_nr - 1) + SECTORS * ((cf->dhr & 0xf) | (cf->cyl_low << 4) | (cf->cyl_high << 12));
}

static void update_sector_address(struct command_file *cf, uint32_t new_addr)
{
	int track = (new_addr / SECTORS);
	cf->sector_nr = (new_addr % SECTORS) + 1;
	cf->dhr = (cf->dhr & 0xf0) | (track & 0xf);
	cf->cyl_low = (track & 0xff0) >> 4;
	cf->cyl_high = (track & 0xff000) >> 12;
}

static int get_sector_count(struct command_file *cf)
{
	int n = cf->sector_count;
	if (n == 0)
		n = 256;
	return n;
}

static void start_ide_command(uint8_t *cmd)
{
	/*
	  cmd[1] = opcode
	  cmd[2-5] = address
	  cmd[6] = sector count
	  cmd[7] = features
	*/

#if VERBOSE
	uart_printf("[%02x %02x] ", cmd[0], cmd[1]);
#endif

	struct command_file cf;
	unpack_command(&cf, cmd);

	uint32_t sector_address = get_sector(&cf);
	int sector_count = get_sector_count(&cf);

	if (cmd[1] == 0xec) {
		// Identify drive
		memset(sector_buffer, 0, sizeof(sector_buffer));
		sector_buffer[0<<1] = 0x80;
		sector_buffer[1<<1] = CYLINDERS & 0xff;
		sector_buffer[(1<<1)+1] = CYLINDERS >> 8;
		sector_buffer[3<<1] = 16;
		sector_buffer[6<<1] = SECTORS;
		char s[20];
		sprintf(s, "EtherZ eMMC %s", card_pnm);
		swizzle_string(sector_buffer+(27<<1), s, 20);
		swizzle_string(sector_buffer+(23<<1), VERSION, 8);
		sprintf(s, "%02x%02x%02x%02x", card_psn[0], card_psn[1], card_psn[2], card_psn[3]);
		swizzle_string(sector_buffer+(10<<1), s, 20);
		return_sector_buffer(&cf, 0, false);
		finish_ide_command();
		return;
	}

	if (cmd[1] == 0x30 || cmd[1] == 0x31) {
		// Write sectors
		fill_sector_buffer();
		uart_printf("WS %d@%x ", sector_count, sector_address);
		sd_mmc_err_t err;
		err = sd_mmc_init_write_blocks(0, sector_address, 1);
		if (err == SD_MMC_OK)
			err = sd_mmc_start_write_blocks(sector_buffer, 1);
		if (err == SD_MMC_OK)
			err = sd_mmc_wait_end_of_write_blocks(0);
#if 0
		debug_putc('E');
		debug_put_hex(err);
		debug_putc('\n');
#endif
		bool more = (sector_count > 1);
		update_sector_address(&cf, ++sector_address);
		cf.sector_count--;
		return_status(&cf, more, 0, more, false);
		finish_ide_command();
		return;
	}

	if (cmd[1] == 0x20) {
		// Read sectors from the card
		uart_printf("RS %d@%x ", sector_count, sector_address);
		sd_mmc_err_t err;
		err = sd_mmc_init_read_blocks(0, sector_address, 1);
		if (err == SD_MMC_OK)
			err = sd_mmc_start_read_blocks(sector_buffer, 1);
		if (err == SD_MMC_OK)
			err = sd_mmc_wait_end_of_read_blocks(0);
#if 0
		debug_putc('E');
		debug_put_hex(err);
#endif
#if 0
		debug_putc('=');
		for (int i = 0; i < 4; i++)
			debug_put_hex(sector_buffer[i]);
		debug_putc('\n');
#endif
		bool more = (sector_count > 1);
		update_sector_address(&cf, ++sector_address);
		cf.sector_count--;
		return_sector_buffer(&cf, 0, more);
		finish_ide_command();
		return;
	}

	if (((cmd[1] & 0xf0) == 0x10) || ((cmd[1] & 0xf0) == 0x70)) {
		// recalibrate or seek
		return_status(&cf, false, 0, false, false);
		finish_ide_command();
		return;
	}

	if (cmd[1] == 0x91) {
		// initialise drive parameters
		return_status(&cf, false, 0, false, false);
		finish_ide_command();
		return;
	}

	uart_printf("Unknown cmd %02x %02x %02x %02x %02x %02x %02x %02x\r\n",
		    cmd[0], cmd[1], cmd[2], cmd[3], cmd[4], cmd[5], cmd[6], cmd[7]);
	return_status(&cf, false, 1 << 2, false, false);
	finish_ide_command();
}

static void fpga_irq_handler(uint32_t id, uint32_t mask)
{
	uint8_t rbuf[8];
	get_command_file(rbuf);
	start_ide_command(rbuf);
}

static void board_uart_init(void)
{
	Uart *uart = UART1;
	struct sam_uart_opt opt = {
	ul_mck: (BOARD_FREQ_MAINCK_XTAL * CONFIG_PLL0_MUL) / (CONFIG_PLL0_DIV * 2),
	ul_baudrate: 115200,
	ul_mode: UART_MR_PAR_NO
	};
	pmc_enable_periph_clk(ID_UART1);
	uart_init(uart, &opt);
	ioport_set_pin_mode(PIO_PB2_IDX, IOPORT_MODE_MUX_A);
	ioport_disable_pin(PIO_PB2_IDX);
	ioport_set_pin_mode(PIO_PB3_IDX, IOPORT_MODE_MUX_A);
	ioport_disable_pin(PIO_PB3_IDX);
}

int main(void)
{
	ptr_put = debug_put;

	sysclk_init();
	ioport_init();

	board_uart_init();

	load_fpga();

	setup_emmc();

	ioport_set_pin_mode(FPGA_IRQ_GPIO, 0);
	ioport_set_pin_dir(FPGA_IRQ_GPIO, IOPORT_DIR_INPUT);
	ioport_enable_pin(FPGA_IRQ_GPIO);

	ioport_set_pin_mode(FPGA_SS2_GPIO, 0);
	ioport_set_pin_dir(FPGA_SS2_GPIO, IOPORT_DIR_OUTPUT);
	ioport_enable_pin(FPGA_SS2_GPIO);
	ioport_set_pin_level(FPGA_SS2_GPIO, IOPORT_PIN_LEVEL_HIGH);

	NVIC_SetPriority(PIOA_IRQn, 4);
	NVIC_SetPriority(UART1_IRQn, 2);

	NVIC_EnableIRQ(PIOA_IRQn);
	pio_handler_set(PIOA, ID_PIOA, PIO_PA5, PIO_IT_RISE_EDGE, fpga_irq_handler);
	pio_enable_interrupt(PIOA, PIO_PA5);
	ioport_enable_pin(PIO_PA5_IDX);
	wdt_disable(WDT);

	NVIC_EnableIRQ(UART1_IRQn);
	uart_printf("Ready\r\n");

	for (;;)
		;
}
