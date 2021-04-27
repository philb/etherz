#include "asf.h"
#include "conf_board.h"
#include "spi.h"
#include "pdc.h"
#include "ioport.h"
#include "sd_mmc.h"
#include "delay.h"
#include "wdt.h"

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
static uint32_t gs_ul_spi_clock = 2500000;

#define FPGA_CRESET_GPIO            (PIO_PA19_IDX)
#define FPGA_CDONE_GPIO             (PIO_PA21_IDX)
#define FPGA_SS_GPIO		    (PIO_PA11_IDX)

extern void debug_puts(const char *);
extern void debug_putc(char);

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

	debug_puts("CDONE ");
	debug_putc(ioport_get_pin_level(FPGA_CDONE_GPIO) ? '1' : '0');
	debug_putc('\n');

	pdc_spi_packet.ul_addr = (uint32_t)&fpga_image[0];
	pdc_spi_packet.ul_size = 8;
	pdc_tx_init(pdc, &pdc_spi_packet, NULL);

	/* Enable the TX PDC transfer requests */
	pdc_enable_transfer(pdc, PERIPH_PTCR_TXTEN);

	/* Waiting transfer done*/
	while((spi_read_status(SPI) & SPI_SR_ENDTX) == 0);

	/* Disable the RX and TX PDC transfer requests */
	pdc_disable_transfer(pdc, PERIPH_PTCR_RXTDIS | PERIPH_PTCR_TXTDIS);

	// Enable clock to FPGA
	ioport_set_pin_mode(PIO_PA6_IDX, IOPORT_MODE_MUX_B);
	ioport_disable_pin(PIO_PA6_IDX);
	pmc_disable_pck(0);
	pmc_switch_pck_to_mainck(0, 0);
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
	sd_mmc_check(0);

	printf("eMMC %dMB\n", sd_mmc_get_capacity(0) / 1024);
}

static int debug_put(void volatile *x, char c)
{
	debug_putc(c);
	return 0;
}

int main(void)
{
	ptr_put = debug_put;

	sysclk_init();
	ioport_init();
	debug_puts("Hello\n");
	load_fpga();

	setup_emmc();

	wdt_disable(WDT);

	for (;;)
		;
}
