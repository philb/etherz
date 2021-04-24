#ifndef BOARD_H
#define BOARD_H

#include <conf_board.h>

#define BOARD_FREQ_SLCK_XTAL        (32768UL)
#define BOARD_FREQ_SLCK_BYPASS      (32768UL)

#define BOARD_FREQ_MAINCK_XTAL      (18432000U)
#define BOARD_FREQ_MAINCK_BYPASS    (18432000U)

#define BOARD_OSC_STARTUP_US        (15625UL)

/*! \brief This function initializes the board target resources
 *
 * This function should be called to ensure proper initialization of the target
 * board hardware connected to the part.
 */
extern void board_init(void);

#endif
