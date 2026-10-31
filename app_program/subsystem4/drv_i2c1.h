/*
 *  drv_i2c1.h
 *  Subsystem 4 -- I2C1 master on GP2 (SDA) / GP3 (SCL), 400 kHz, polled.
 *  Register-level: the kernel I2C driver stays off (DEVCNF_USE_IIC = 0,
 *  its default pins are the left motor's GP8/GP9).  ImuTask is the only
 *  user (Week 6 Section 3), on core 0.  Owner: Buddy 4.
 *
 *  A 7-byte read takes ~0.25 ms on the bus; every call is bounded by a
 *  2 ms timeout and returns E_IO on a NACK / abort and E_TMOUT on a hang.
 */

#ifndef DRV_I2C1_H
#define DRV_I2C1_H

#include <stdint.h>
#include "../app.h"

IMPORT ER drv_i2c1_init(void);          /* also used to recover the bus */
IMPORT ER drv_i2c1_write_reg(uint8_t addr7, uint8_t reg, uint8_t value);
IMPORT ER drv_i2c1_read(uint8_t addr7, uint8_t reg, uint8_t * p_buf, uint8_t n);
/* E_OK if a device answers (ACK) at addr7; E_IO on NACK, E_TMOUT if the bus is stuck. */
IMPORT ER drv_i2c1_probe(uint8_t addr7);

#endif /* DRV_I2C1_H */
