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
	debug_puts("SD ");
	debug_put_hex(r);
	debug_putc('\n');

	capacity_mb = sd_mmc_get_capacity(0) / 1024;
	uint8_t *cid = sd_mmc_get_cid(0);

	memcpy(card_psn, cid+10, 4);
	memcpy(card_pnm, cid+3, 6);
	card_pnm[6] = 0;
}

static int debug_put(void volatile *x, char c)
{
	debug_putc(c);
	return 0;
}

static void finish_ide_command(void)
{
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
#if 0
	debug_putc('R');
	debug_put_hex(tbuf[4]);
	debug_put_hex(tbuf[7]);
	debug_putc('\n');
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
	return cf->sector_nr | (cf->cyl_low << 8) | (cf->cyl_high << 16) | ((cf->dhr & 0xf) << 24);
}

static void update_sector_address(struct command_file *cf, uint32_t new_addr)
{
	cf->sector_nr = (new_addr & 0xff);
	cf->cyl_low = (new_addr & 0xff00) >> 8;
	cf->cyl_high = (new_addr & 0xff0000) >> 16;
	cf->dhr = cf->dhr & 0xf0 | (new_addr & 0x0f000000) >> 24;
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

#if 0
	debug_puts("Cmd ");
	debug_put_hex(cmd[0]);
	debug_putc(' ');
	debug_put_hex(cmd[1]);
	debug_putc('\n');
#endif

	struct command_file cf;
	unpack_command(&cf, cmd);

	uint32_t sector_address = get_sector(&cf);
	int sector_count = get_sector_count(&cf);

	if (cmd[1] == 0xec) {
		// Identify drive
		memset(sector_buffer, 0, sizeof(sector_buffer));
		sector_buffer[0<<1] = 0x80;
		sector_buffer[1<<1] = 255;
		sector_buffer[3<<1] = 16;
		sector_buffer[6<<1] = 255;
		char s[20];
		sprintf(s, "EtherZ eMMC %s", card_pnm);
		swizzle_string(sector_buffer+(27<<1), s, 20);
		swizzle_string(sector_buffer+(23<<1), "0.1", 8);
		sprintf(s, "%02x%02x%02x%02x", card_psn[0], card_psn[1], card_psn[2], card_psn[3]);
		swizzle_string(sector_buffer+(10<<1), s, 20);
		return_sector_buffer(&cf, 0, false);
		finish_ide_command();
		return;
	}

	if (cmd[1] == 0x30 || cmd[1] == 0x31) {
		// Write sectors
		fill_sector_buffer();
#if 0
		debug_putc('W');
		debug_put_hex(sector_count);
		debug_putc('@');
		debug_put_hex(sector_address >> 24);
		debug_put_hex(sector_address >> 16);
		debug_put_hex(sector_address >> 8);
		debug_put_hex(sector_address >> 0);
		debug_putc('=');
		for (int i = 0; i < 16; i++)
			debug_put_hex(sector_buffer[i]);
#endif
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
#if 0
		debug_puts("RS ");
		debug_put_hex(sector_count);
		debug_putc('@');
		debug_put_hex((sector_address >> 24) & 0xff);
		debug_put_hex((sector_address >> 16) & 0xff);
		debug_put_hex((sector_address >> 8) & 0xff);
		debug_put_hex(sector_address & 0xff);
#endif
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

	debug_puts("Cmd ");
	debug_put_hex(cmd[0]);
	debug_putc(' ');
	debug_put_hex(cmd[1]);
	debug_putc(' ');
	debug_put_hex(cmd[2]);
	debug_putc(' ');
	debug_put_hex(cmd[3]);
	debug_putc(' ');
	debug_put_hex(cmd[4]);
	debug_putc(' ');
	debug_put_hex(cmd[5]);
	debug_putc(' ');
	debug_put_hex(cmd[6]);
	debug_putc(' ');
	debug_put_hex(cmd[7]);
	debug_putc('\n');
	return_status(&cf, false, 1 << 2, false, false);
	finish_ide_command();
}

static void fpga_irq_handler(uint32_t id, uint32_t mask)
{
	Pdc *pdc = spi_get_pdc_base(SPI);

	uint8_t tbuf[8], rbuf[8];
	memset(tbuf, 0, sizeof(tbuf));
	memset(rbuf, 0xff, sizeof(rbuf));

	pio_disable_interrupt(PIOA, PIO_PA5);

	ioport_set_pin_level(FPGA_SS_GPIO, IOPORT_PIN_LEVEL_LOW);

	spi_get(SPI);

	pdc_packet_t pdc_spi_packet;

	pdc_spi_packet.ul_addr = (uint32_t)rbuf;
	pdc_spi_packet.ul_size = sizeof(rbuf);
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

#if 0
	for (int i = 0; i < sizeof(rbuf); i++) {
		debug_put_hex(rbuf[i]);
		debug_putc(' ');
	}
	debug_putc('\n');
#endif

	start_ide_command(rbuf);
}

int main(void)
{
	ptr_put = debug_put;

	sysclk_init();
	ioport_init();
	load_fpga();

	setup_emmc();

	ioport_set_pin_mode(FPGA_IRQ_GPIO, 0);
	ioport_set_pin_dir(FPGA_IRQ_GPIO, IOPORT_DIR_INPUT);
	ioport_enable_pin(FPGA_IRQ_GPIO);

	ioport_set_pin_mode(FPGA_SS2_GPIO, 0);
	ioport_set_pin_dir(FPGA_SS2_GPIO, IOPORT_DIR_OUTPUT);
	ioport_enable_pin(FPGA_SS2_GPIO);
	ioport_set_pin_level(FPGA_SS2_GPIO, IOPORT_PIN_LEVEL_HIGH);

	NVIC_EnableIRQ(PIOA_IRQn);
	pio_handler_set(PIOA, ID_PIOA, PIO_PA5, PIO_IT_RISE_EDGE, fpga_irq_handler);
	pio_enable_interrupt(PIOA, PIO_PA5);
	ioport_enable_pin(PIO_PA5_IDX);
	wdt_disable(WDT);

	for (;;)
		;
}
