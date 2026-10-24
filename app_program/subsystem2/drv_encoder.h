/*
 *  drv_encoder.h
 *  Subsystem 2 -- two quadrature encoders counted by PIO0 (SM0: GP4/GP5,
 *  SM1: GP16/GP17), x4, in hardware.  Register-level.  Owner: Buddy 2.
 *
 *  The program is the Raspberry Pi pico-examples quadrature_encoder.pio
 *  (BSD-3-Clause), 24 instructions that must sit at offset 0 because the
 *  first 16 are a jump table indexed by the pin state.  The SDK radio
 *  driver tries PIO1 first and loads its 6-instruction program at the top
 *  of the block, so PIO0 offset 0 stays free (Week 6 Section 8.2).
 */

#ifndef DRV_ENCODER_H
#define DRV_ENCODER_H

#include <stdint.h>
#include "../app.h"

#define DRV_ENC_LEFT        0u      /* PIO0 SM0 */
#define DRV_ENC_RIGHT       1u      /* PIO0 SM1 */

/* Returns E_OBJ if a PIO0 state machine 0/1 is already running (boot check). */
IMPORT ER      drv_encoder_init(void);

/* Current count (signed, wraps at 2^32).  A few microseconds. */
IMPORT int32_t drv_encoder_read(uint32_t which);

#endif /* DRV_ENCODER_H */
