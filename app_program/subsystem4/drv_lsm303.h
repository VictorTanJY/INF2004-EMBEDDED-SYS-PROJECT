/* LSM303D accelerometer + magnetometer, shared I2C address 0x1E.
 * Acceleration: 100 Hz, +/-4 g. Magnetic: 50 Hz, +/-4 gauss.
 * INT1: active-high, non-latched, high-pass filtered high-g event.
 */
#ifndef DRV_LSM303_H
#define DRV_LSM303_H

#include <stdint.h>
#include "../app.h"

/* E_OBJ for an unexpected identity; bus/configuration errors propagate. */
IMPORT ER drv_lsm303_init(void);

/* Acceleration in mg, raw sensor axes X, Y, Z. */
IMPORT ER drv_lsm303_read_accel(int32_t acc_mg[3]);

/* Magnetic raw counts, raw sensor axes X, Y, Z. */
IMPORT ER drv_lsm303_read_mag(int16_t mag[3]);

/* Result of the last drv_lsm303_init():
 * bits 31..24: stage:
 *             0 = success
 *             1 = identity read failed
 *             2 = unexpected identity
 *             4 = configuration failed
 * bits 23..16: negated error code, low byte
 * bits 15..8:  observed WHO_AM_I value (expected 0x49)
 * bits 7..0:   reserved
 */
IMPORT uint32_t drv_lsm303_diag(void);

#endif /* DRV_LSM303_H */