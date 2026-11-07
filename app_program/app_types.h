/*
 *  app_types.h
 *  Small pure helpers shared by every subsystem.  Changed only by team
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

/* Barr-C 5.2: floating point, if ever used, is declared with this type.
   The firmware itself is integer / fixed point (Cortex-M0+, no FPU). */
typedef float float32_t;

static inline int32_t app_abs_i32(int32_t value)
{
    return (value < 0) ? -value : value;
}

static inline int32_t app_clamp_i32(int32_t value, int32_t lo, int32_t hi)
{
    int32_t result = value;

    if (result < lo)
    {
        result = lo;
    }
    if (result > hi)
    {
        result = hi;
    }
    return result;
}

static inline int32_t app_min_i32(int32_t a, int32_t b)
{
    return (a < b) ? a : b;
}

static inline int32_t app_max_i32(int32_t a, int32_t b)
{
    return (a > b) ? a : b;
}

/* Time comparisons that survive the 32-bit millisecond wrap. */
static inline uint32_t app_elapsed_ms(uint32_t now_ms, uint32_t since_ms)
{
    return (uint32_t)(now_ms - since_ms);
}

#endif /* APP_TYPES_H */
