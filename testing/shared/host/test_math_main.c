/** @file test_math_main.c
 *
 *  @brief Host unit tests for the shared integer maths (app_program/app_math.c)
 *  and the minimum-bit packing helpers (app_program/app_pack.h, SH-H05).
 *  Build and run:  make -C testing/shared/host
 */

#include "test_math_main.h"
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <inttypes.h>
#include <stdbool.h>
#include "app_types.h"
#include "app_math.h"
#include "app_pack.h"

#define T_PAST_RANGE     3 /* values tried outside the field range */
#define T_RANDOM_FIELDS  100u
#define T_SIN_RANGE_CDEG 72000 /* +-720 deg */
#define T_ATAN_RANGE     3000  /* +-3000 on each axis */
#define T_ISQRT_MAX      70000u

/* NOTE: a macro rather than a function only so that a failure reports the
   caller's file and line (__FILE__, __LINE__). */
#define CHECK(id, cond) (check_result((id), (cond), __FILE__, __LINE__))

static int32_t g_pass;
static int32_t g_fail;

static void test_pack(void);
static void check_result(char const * p_id, bool b_ok, char const * p_file,
                         int32_t line);
static void test_pack_h05(void);
static void test_pack_h05_part2(void);
static void test_trig(int32_t * p_worst_sin, float64_t * p_worst_atan);
static void test_isqrt(void);
static void test_wrap(void);

/** Run the shared host tests (app_pack, app_math).  Exit status 0 if all
    passed. */
int main (void)
{
    int32_t   worst_sin  = 0;
    float64_t worst_atan = 0.0;

    test_pack();
    test_trig(&worst_sin, &worst_atan);
    test_isqrt();
    test_wrap();
    printf("worst sin/cos error %d LSB, worst atan2 error %.1f cdeg\n",
           worst_sin, worst_atan);
    printf("host tests: %d passed, %d failed\n", g_pass, g_fail);
    return (0 == g_fail) ? 0 : 1;
}

/** SH-H01..H03: sine, cosine, atan2 and the integer square roots;
 *p_worst_sin / *p_worst_atan = the largest errors seen. */
static void test_trig (int32_t * p_worst_sin, float64_t * p_worst_atan)
{
    int32_t val_a;
    /* SH-H01: sin/cos over -720..720 deg in 0.07 deg steps */
    for (val_a = -T_SIN_RANGE_CDEG; val_a <= T_SIN_RANGE_CDEG; val_a += 7)
    {
        /* Casts (int32_t)lround(sin(...)): a Q15 sine: -32768..32768;
           (float64_t)val_a: exact: a 32-bit integer fits the 53-bit mantissa of
           a double. */
        int32_t ref =
            (int32_t)lround(sin(((float64_t)val_a * M_PI) / 18000.0) * 32768.0);
        int32_t got = app_math_sin_q15(val_a);
        int32_t diff;

        if (ref > 32767)
        {
            ref = 32767;
        }
        diff = abs(got - ref);
        if (diff > *p_worst_sin)
        {
            *p_worst_sin = diff;
        }
        /* Casts (int32_t)lround(cos(...)): a Q15 cosine: -32768..32768;
           (float64_t)val_a: exact: a 32-bit integer fits the 53-bit mantissa of
           a double. */
        ref =
            (int32_t)lround(cos(((float64_t)val_a * M_PI) / 18000.0) * 32768.0);
        if (ref > 32767)
        {
            ref = 32767;
        }
        diff = abs(app_math_cos_q15(val_a) - ref);
        if (diff > *p_worst_sin)
        {
            *p_worst_sin = diff;
        }
    }
    CHECK("SH-H01", *p_worst_sin <= 2);

    /* SH-H02: atan2 on a grid of vectors, all quadrants */
    {
        int32_t x_start;
        int32_t y_start;

        for (x_start = -T_ATAN_RANGE; x_start <= T_ATAN_RANGE; x_start += 37)
        {
            for (y_start = -T_ATAN_RANGE; y_start <= T_ATAN_RANGE;
                 y_start += 41)
            {
                /* Casts (float64_t)y_start, (float64_t)x_start,
                   (float64_t)app_math_atan2_cdeg(...): exact: a 32-bit integer
                   fits the 53-bit mantissa of a double. */
                float64_t ref =
                    (atan2((float64_t)y_start, (float64_t)x_start) * 18000.0)
                    / M_PI;
                float64_t diff = fabs(
                    (float64_t)app_math_atan2_cdeg(y_start, x_start) - ref);

                if (diff > 18000.0)
                {
                    diff = 36000.0 - diff;
                } /* +-180 seam */
                if (diff > *p_worst_atan)
                {
                    *p_worst_atan = diff;
                }
            }
        }
        CHECK("SH-H02", *p_worst_atan <= 1.0);
        CHECK("SH-H02", 0 == app_math_atan2_cdeg(0, 0));
        CHECK("SH-H02", 9000 == app_math_atan2_cdeg(1, 0));
        CHECK("SH-H02", 18000 == app_math_atan2_cdeg(0, -1));
        CHECK("SH-H02", -9000 == app_math_atan2_cdeg(-1, 0));
        CHECK("SH-H02",
              abs(app_math_atan2_cdeg(2147483647, -2147483647 - 1) - 13500)
                  <= 10);
    }
}

/** SH-H03: integer square roots, exhaustive near squares. */
static void test_isqrt (void)
{
    uint32_t x_val;

    for (x_val = 0u; x_val < T_ISQRT_MAX; x_val++)
    {
        /* Cast (uint64_t)x_val: every uint32_t value fits. */
        uint64_t square = (uint64_t)x_val * x_val;

        if ((app_math_isqrt64(square) != x_val)
            || ((x_val > 0u)
                && (app_math_isqrt64(square - 1u) != (x_val - 1u))))
        {
            CHECK("SH-H03", 0);
            break;
        }
        /* Cast (uint32_t)square: square <= 0xFFFFFFFF on this branch (tested in
           the same statement). */
        if ((square <= 0xFFFFFFFFu)
            && (app_math_isqrt32((uint32_t)square) != x_val))
        {
            CHECK("SH-H03", 0);
            break;
        }
    }
    CHECK("SH-H03", 65535u == app_math_isqrt32(0xFFFFFFFFu));
    CHECK("SH-H03", 0xFFFFFFFFu == app_math_isqrt64(0xFFFFFFFFFFFFFFFFull));
}

/** SH-H04: angle wrap and Q15 multiply. */
static void test_wrap (void)
{
    /* SH-H04: wrap and Q15 multiply */
    CHECK("SH-H04", -18000 == app_math_wrap_cdeg(18000));
    CHECK("SH-H04", 17999 == app_math_wrap_cdeg(-18001));
    CHECK("SH-H04", 4500 == app_math_wrap_cdeg(72000 + 4500));
    CHECK("SH-H04", 1000000000 == app_math_mul_q15(2000000000, 16384));
}

/** SH-H05: app_pack.h, every width the firmware uses */
static void test_pack (void)
{
    test_pack_h05();
    test_pack_h05_part2();
}

/** Checks SH-H05. */
static void test_pack_h05 (void)
{
    static uint8_t const widths[] = {2u,  3u,  4u,  5u,  6u,  7u,  8u, 9u,
                                     10u, 11u, 12u, 16u, 20u, 24u, 32u};
    uint32_t             idx;
    bool                 b_ok = true;

    for (idx = 0u; idx < sizeof(widths); idx++)
    {
        /* Cast (int64_t)(UINT64_C(1) << (width - ...)): width <= 32, so 2^31 at
           most. */
        uint8_t const width = widths[idx];
        int64_t const half  = (int64_t)(UINT64_C(1) << (width - 1u));
        int64_t       value;

        /* signed: exact inside the range, saturated outside it */
        for (value = -half - T_PAST_RANGE; value <= (half + T_PAST_RANGE);
             value += (width <= 12u) ? 1 : ((half / 7) + 1))
        {
            /* Cast (int32_t)((value < INT32_MIN) ? ...): clamped to
               INT32_MIN..INT32_MAX in the same expression. */
            int32_t const input =
                (int32_t)((value < INT32_MIN)
                              ? INT32_MIN
                              : ((value > INT32_MAX) ? INT32_MAX : value));
            int64_t const expected =
                (input < -half) ? -half
                                : ((input > (half - 1)) ? (half - 1) : input);
            uint32_t const factor = app_pack_off_s(input, width);

            if ((factor > app_pack_mask(width))
                || (app_pack_unoff_s(factor, width) != expected))
            {
                b_ok = false;
            }
        }
        /* unsigned: saturates at the field maximum */
        if ((app_pack_sat_u(app_pack_mask(width), width)
             != app_pack_mask(width))
            || ((width < 32u)
                && (app_pack_sat_u(app_pack_mask(width) + 1u, width)
                    != app_pack_mask(width))))
        {
            b_ok = false;
        }
        /* wrapping counters: modular difference within +-half */
        if ((width >= 8u)
            && ((app_pack_wrap_diff(2u, app_pack_mask(width) - 1u, width) != 4)
                || (app_pack_wrap_diff(app_pack_mask(width) - 1u, 2u, width)
                    != -4)))
        {
            b_ok = false;
        }
    }
    CHECK("SH-H05", b_ok);
    CHECK("SH-H05", (0u == app_pack_off_s(INT32_MIN, 32u))
                        && (0xFFFFFFFFu == app_pack_off_s(INT32_MAX, 32u)));
    CHECK("SH-H05", (INT32_MIN == app_pack_unoff_s(0u, 32u))
                        && (INT32_MAX == app_pack_unoff_s(0xFFFFFFFFu, 32u)));

    /* fields in a word: set one without touching its neighbours */
    {
        uint32_t bit_width = 0xFFFFFFFFu;

        bit_width = app_pack_set(bit_width, 5u, 7u, 0u);
        CHECK("SH-H05", (0xFFFFF01Fu == bit_width)
                            && (0u == app_pack_get(bit_width, 5u, 7u))
                            && (0x1Fu == app_pack_get(bit_width, 0u, 5u)));
        bit_width = app_pack_set(bit_width, 5u, 7u, 0x55u);
        CHECK("SH-H05", (0x55u == app_pack_get(bit_width, 5u, 7u))
                            && (0xFFFFFu == app_pack_get(bit_width, 12u, 20u)));
        CHECK("SH-H05", (0x12345678u == app_pack_set(0u, 0u, 32u, 0x12345678u))
                            && (1u == app_pack_get(0x80000000u, 31u, 1u)));
    }
}

/** Checks SH-H05. */
static void test_pack_h05_part2 (void)
{
    uint8_t           buf[64];
    uint32_t          vals[100];
    uint8_t           bit_widths[100];
    app_pack_writer_t writer;
    app_pack_reader_t reader;
    uint32_t          count = 0u;
    uint32_t          kdx;
    int32_t           ret = 1;

    /* streams: random widths and values round-trip; overflow is reported */
    srand(77);
    app_pack_writer_init(&writer, buf, sizeof(buf));
    while (count < T_RANDOM_FIELDS)
    {
        /* Casts (uint8_t)(1 + (rand() % 32)): 1..32; (uint32_t)rand(): rand()
           is not negative; (uint32_t)writer.bits: every uint16_t value fits. */
        bit_widths[count] = (uint8_t)(1 + (rand() % 32));
        vals[count]       = (((uint32_t)rand() << 16) ^ (uint32_t)rand())
                            & app_pack_mask(bit_widths[count]);
        if (((uint32_t)writer.bits + bit_widths[count]) > (sizeof(buf) * 8u))
        {
            break;
        }
        app_pack_put(&writer, vals[count], bit_widths[count]);
        count++;
    }
    app_pack_reader_init(&reader, buf, app_pack_writer_len(&writer));
    for (kdx = 0u; kdx < count; kdx++)
    {
        if (app_pack_take(&reader, bit_widths[kdx]) != vals[kdx])
        {
            ret = 0;
        }
    }
    CHECK("SH-H05", ret && !reader.b_short && (count > 10u));
    app_pack_put(&writer, 0xFFFFFFFFu, 32u);
    app_pack_put(&writer, 0u, 33u);
    CHECK("SH-H05", writer.b_overflow && (0u == app_pack_writer_len(&writer)));
}

/*!
 * @brief Count one check; print a FAIL line when it failed.
 * @param[in] p_id Test id, as in the subsystem's TEST_LOG.md.
 * @param[in] b_ok Whether the check passed.
 * @param[in] p_file Source file of the check.
 * @param[in] line Source line of the check.
 */
static void check_result (char const * p_id, bool b_ok, char const * p_file,
                          int32_t line)
{
    if (b_ok)
    {
        g_pass++;
    }
    else
    {
        g_fail++;
        printf("FAIL %s  (%s:%" PRId32 ")\n", p_id, p_file, line);
    }
}

/*** end of file ***/
