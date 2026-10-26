/*
 *  drv_lsm303.h
 *  Subsystem 4 -- GY-511 (LSM303DLHC) accelerometer + magnetometer on
 *  I2C1.  Owner: Buddy 4.  Datasheet: lsm303dlhc.pdf (project files).
 *
 *  Accelerometer (0x19): 100 Hz, +-4 g, high resolution, INT1 = high-g on
 *  any axis, high-pass filtered (gravity removed), threshold IMU_INT1_THS.
 *  Magnetometer (0x1E): 75 Hz, +-1.3 gauss, continuous.
 */

#ifndef DRV_LSM303_H
#define DRV_LSM303_H

#include <stdint.h>
#include "../app.h"

/* Configure both sensors; E_OBJ if the identity check fails (IRA_REG_M
   must read 0x48, Week 6 Section 5). */
IMPORT ER drv_lsm303_init(void);

/* Acceleration in mg, raw sensor axes X, Y, Z. */
IMPORT ER drv_lsm303_read_accel(int32_t acc_mg[3]);

/* Magnetic field, raw counts, sensor axes X, Y, Z (the chip sends X, Z, Y). */
IMPORT ER drv_lsm303_read_mag(int16_t mag[3]);
/* Result of the last drv_lsm303_init(), reported as LOG 53:
     bits 31:24 stage: 0 ok, 1 magnetometer (0x1E) not answering,
                2 unknown identity, 3 accelerometer (0x19) not answering,
                4 a configuration write failed
     bits 23:16 -er of the failing I2C call (0 if none)
     bits 15:8  IRA_REG_M byte read (0x48 on an LSM303DLHC)
     bits 7:0   WHO_AM_I_M byte read (0x40 on an LSM303AGR)            */
IMPORT uint32_t drv_lsm303_diag(void);

#endif /* DRV_LSM303_H */
