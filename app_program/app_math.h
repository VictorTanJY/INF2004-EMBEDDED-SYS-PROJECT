/*
 *  app_math.h
 *  Integer trigonometry and square root shared by subsystems 2, 4 and 5.
 *
 *  Why not <math.h>: the template links no libm (adding -lm would change
 *  the shared makefile), and the Cortex-M0+ has no FPU, so float sinf()/
 *  atan2f() would be slow soft-float anyway.  These functions use 32/64-bit
 *  integers only.  Pure C (host-testable): testing/shared/host/.
 *
 *  Angles are in centidegrees (cdeg, 1/100 degree), positive = counter-
 *  clockwise (left / nose up).  Trig results are Q15 (32767 = 1.0).
 *  Accuracy (checked by the host tests over the full range):
 *    app_atan2_cdeg   |error| <= 1 cdeg (0.01 deg)
 *    app_sin_q15      |error| <= 2 LSB (6e-5) over -360..+360 deg
 */

#ifndef APP_MATH_H
#define APP_MATH_H

#include <stdint.h>

#define APP_Q15_ONE         32768
#define APP_CDEG_90         9000
#define APP_CDEG_180        18000
#define APP_CDEG_360        36000

/* floor(sqrt(x)) */
uint32_t app_isqrt32(uint32_t x);
uint32_t app_isqrt64(uint64_t x);

/* atan2(y, x) in cdeg, -18000..18000.  atan2(0, 0) = 0. */
int32_t app_atan2_cdeg(int32_t y, int32_t x);

/* sin / cos of an angle in cdeg (any value), Q15. */
int32_t app_sin_q15(int32_t cdeg);
int32_t app_cos_q15(int32_t cdeg);

/* Wrap an angle into -18000 .. +17999 cdeg. */
int32_t app_wrap_cdeg(int32_t cdeg);

/* value * q15 / 32768, rounded toward zero, without 32-bit overflow. */
int32_t app_mul_q15(int32_t value, int32_t q15);

#endif /* APP_MATH_H */
