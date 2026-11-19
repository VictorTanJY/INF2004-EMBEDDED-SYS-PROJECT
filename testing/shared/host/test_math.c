/*
 *  test_math.c
 *  Host unit tests for the shared integer maths (app_program/app_math.c)
 *  and the minimum-bit packing helpers (app_program/app_pack.h, SH-H05).
 *  Build and run:  make -C testing/shared/host
 */

#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include "app_math.h"
#include "app_pack.h"

static int g_pass;
static int g_fail;

#define CHECK(id, cond) \
    do { if (cond) { g_pass++; } else { g_fail++; \
        printf("FAIL %s  (%s:%d)  %s\n", id, __FILE__, __LINE__, #cond); } } while (0)

/* SH-H05: app_pack.h, every width the firmware uses */
static void test_pack(void)
{
    static uint8_t const widths[] = { 2u, 3u, 4u, 5u, 6u, 7u, 8u, 9u, 10u, 11u, 12u, 16u, 20u, 24u, 32u };
    unsigned idx;
    int      b_ok = 1;

    for (idx = 0u; idx < sizeof(widths); idx++)
    {
        uint8_t const width    = widths[idx];
        int64_t const half = (int64_t)1 << (width - 1u);
        int64_t       value;

        /* signed: exact inside the range, saturated outside it */
        for (value = -half - 3; value <= half + 3; value += (width <= 12u) ? 1 : (half / 7 + 1))
        {
            int32_t const input  = (int32_t)((value < INT32_MIN) ? INT32_MIN : ((value > INT32_MAX) ? INT32_MAX : value));
            int64_t const exp = (input < -half) ? -half : ((input > half - 1) ? half - 1 : input);
            uint32_t const factor  = pk_off_s(input, width);

            if ((factor > pk_mask(width)) || (pk_unoff_s(factor, width) != exp)) { b_ok = 0; }
        }
        /* unsigned: saturates at the field maximum */
        if ((pk_sat_u(pk_mask(width), width) != pk_mask(width)) || ((width < 32u) && (pk_sat_u(pk_mask(width) + 1u, width) != pk_mask(width)))) { b_ok = 0; }
        /* wrapping counters: modular difference within +-half */
        if ((width >= 8u) && ((pk_wrap_diff(2u, pk_mask(width) - 1u, width) != 4) || (pk_wrap_diff(pk_mask(width) - 1u, 2u, width) != -4))) { b_ok = 0; }
    }
    CHECK("SH-H05", b_ok);
    CHECK("SH-H05", pk_off_s(INT32_MIN, 32u) == 0u && pk_off_s(INT32_MAX, 32u) == 0xFFFFFFFFu);
    CHECK("SH-H05", pk_unoff_s(0u, 32u) == INT32_MIN && pk_unoff_s(0xFFFFFFFFu, 32u) == INT32_MAX);

    /* fields in a word: set one without touching its neighbours */
    {
        uint32_t bit_width = 0xFFFFFFFFu;

        bit_width = pk_set(bit_width, 5u, 7u, 0u);
        CHECK("SH-H05", bit_width == 0xFFFFF01Fu && pk_get(bit_width, 5u, 7u) == 0u && pk_get(bit_width, 0u, 5u) == 0x1Fu);
        bit_width = pk_set(bit_width, 5u, 7u, 0x55u);
        CHECK("SH-H05", pk_get(bit_width, 5u, 7u) == 0x55u && pk_get(bit_width, 12u, 20u) == 0xFFFFFu);
        CHECK("SH-H05", pk_set(0u, 0u, 32u, 0x12345678u) == 0x12345678u && pk_get(0x80000000u, 31u, 1u) == 1u);
    }

    /* streams: random widths and values round-trip; overflow is reported */
    {
        uint8_t     buf[64];
        uint32_t    vals[100];
        uint8_t     bit_widths[100];
        pk_writer_t writer;
        pk_reader_t reader;
        unsigned    count = 0u;
        unsigned    kdx;
        int         ret = 1;

        srand(77);
        pk_writer_init(&writer, buf, sizeof(buf));
        while (count < 100u)
        {
            bit_widths[count]   = (uint8_t)(1 + (rand() % 32));
            vals[count] = ((uint32_t)rand() << 16 ^ (uint32_t)rand()) & pk_mask(bit_widths[count]);
            if ((unsigned)writer.bits + bit_widths[count] > sizeof(buf) * 8u) { break; }
            pk_put(&writer, vals[count], bit_widths[count]);
            count++;
        }
        pk_reader_init(&reader, buf, pk_writer_len(&writer));
        for (kdx = 0u; kdx < count; kdx++)
        {
            if (pk_take(&reader, bit_widths[kdx]) != vals[kdx]) { ret = 0; }
        }
        CHECK("SH-H05", ret && !reader.b_short && count > 10u);
        pk_put(&writer, 0xFFFFFFFFu, 32u);
        pk_put(&writer, 0u, 33u);
        CHECK("SH-H05", writer.b_overflow && pk_writer_len(&writer) == 0u);
    }
}

int main(void)
{
    int32_t val_a;
    int     worst_sin = 0;
    double  worst_atan = 0.0;
    uint32_t x_val;

    test_pack();

    /* SH-H01: sin/cos over -720..720 deg in 0.07 deg steps */
    for (val_a = -72000; val_a <= 72000; val_a += 7)
    {
        int ref = (int)lround(sin((double)val_a * M_PI / 18000.0) * 32768.0);
        int got = app_sin_q15(val_a);
        int diff;

        if (ref > 32767) { ref = 32767; }
        diff = abs(got - ref);
        if (diff > worst_sin) { worst_sin = diff; }
        ref = (int)lround(cos((double)val_a * M_PI / 18000.0) * 32768.0);
        if (ref > 32767) { ref = 32767; }
        diff = abs(app_cos_q15(val_a) - ref);
        if (diff > worst_sin) { worst_sin = diff; }
    }
    CHECK("SH-H01", worst_sin <= 2);

    /* SH-H02: atan2 on a grid of vectors, all quadrants */
    {
        int32_t x_start;
        int32_t y_start;

        for (x_start = -3000; x_start <= 3000; x_start += 37)
        {
            for (y_start = -3000; y_start <= 3000; y_start += 41)
            {
                double ref = atan2((double)y_start, (double)x_start) * 18000.0 / M_PI;
                double diff   = fabs((double)app_atan2_cdeg(y_start, x_start) - ref);

                if (diff > 18000.0) { diff = 36000.0 - diff; }   /* +-180 seam */
                if (diff > worst_atan) { worst_atan = diff; }
            }
        }
        CHECK("SH-H02", worst_atan <= 1.0);
        CHECK("SH-H02", app_atan2_cdeg(0, 0) == 0);
        CHECK("SH-H02", app_atan2_cdeg(1, 0) == 9000);
        CHECK("SH-H02", app_atan2_cdeg(0, -1) == 18000);
        CHECK("SH-H02", app_atan2_cdeg(-1, 0) == -9000);
        CHECK("SH-H02", abs(app_atan2_cdeg(2147483647, -2147483647 - 1) - 13500) <= 10);
    }

    /* SH-H03: integer square roots, exhaustive near squares */
    for (x_val = 0u; x_val < 70000u; x_val++)
    {
        uint64_t square = (uint64_t)x_val * x_val;

        if ((app_isqrt64(square) != x_val) || ((x_val > 0u) && (app_isqrt64(square - 1u) != (x_val - 1u))))
        {
            CHECK("SH-H03", 0);
            break;
        }
        if ((square <= 0xFFFFFFFFu) && (app_isqrt32((uint32_t)square) != x_val))
        {
            CHECK("SH-H03", 0);
            break;
        }
    }
    CHECK("SH-H03", app_isqrt32(0xFFFFFFFFu) == 65535u);
    CHECK("SH-H03", app_isqrt64(0xFFFFFFFFFFFFFFFFull) == 0xFFFFFFFFu);

    /* SH-H04: wrap and Q15 multiply */
    CHECK("SH-H04", app_wrap_cdeg(18000) == -18000);
    CHECK("SH-H04", app_wrap_cdeg(-18001) == 17999);
    CHECK("SH-H04", app_wrap_cdeg(72000 + 4500) == 4500);
    CHECK("SH-H04", app_mul_q15(2000000000, 16384) == 1000000000);

    printf("worst sin/cos error %d LSB, worst atan2 error %.1f cdeg\n", worst_sin, worst_atan);
    printf("host tests: %d passed, %d failed\n", g_pass, g_fail);
    return (g_fail == 0) ? 0 : 1;
}
