/** @file app_types.h
 *
 *  @brief Small pure helpers shared by every subsystem.  Changed only by team
 *  agreement.
 *
 *  Pure C: no kernel, SDK or register access, so the same header is used
 *  by the firmware and by the host unit tests (testing/subsystemN/host/).
 *  Kernel-facing definitions are in app.h.
 *
 *  Barr-C 2018 rule 1.2/5.x: fixed-width types only; functions instead of
 *  function-like macros.
 */

#ifndef APP_TYPES_H
#define APP_TYPES_H

#include <stdint.h>
#include <stdbool.h>

/** Barr-C 5.2: floating point, if ever used, is declared with this type.
   The firmware itself is integer / fixed point (Cortex-M0+, no FPU). */
typedef float  float32_t;
typedef double float64_t; /* host unit tests only */

/** Absolute value (value must not be INT32_MIN). */
static inline int32_t app_types_abs_i32 (int32_t value)
{
    return (value < 0) ? -value : value;
}

/** value limited to low..high. */
static inline int32_t app_types_clamp_i32 (int32_t value, int32_t low,
                                           int32_t high)
{
    int32_t result = value;

    if (result < low)
    {
        result = low;
    }
    if (result > high)
    {
        result = high;
    }
    return result;
}

/** The smaller of lhs and rhs. */
static inline int32_t app_types_min_i32 (int32_t lhs, int32_t rhs)
{
    return (lhs < rhs) ? lhs : rhs;
}

/** The larger of lhs and rhs. */
static inline int32_t app_types_max_i32 (int32_t lhs, int32_t rhs)
{
    return (lhs > rhs) ? lhs : rhs;
}

/** Time comparisons that survive the 32-bit millisecond wrap. */
static inline uint32_t app_types_elapsed_ms (uint32_t now_ms, uint32_t since_ms)
{
    return (now_ms - since_ms);
}

#endif /* APP_TYPES_H */

/*** end of file ***/
