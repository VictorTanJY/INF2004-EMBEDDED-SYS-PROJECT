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
    unsigned i;
    int      ok = 1;

    for (i = 0u; i < sizeof(widths); i++)
    {
        uint8_t const w    = widths[i];
        int64_t const half = (int64_t)1 << (w - 1u);
        int64_t       v;

        /* signed: exact inside the range, saturated outside it */
        for (v = -half - 3; v <= half + 3; v += (w <= 12u) ? 1 : (half / 7 + 1))
        {
            int32_t const in  = (int32_t)((v < INT32_MIN) ? INT32_MIN : ((v > INT32_MAX) ? INT32_MAX : v));
            int64_t const exp = (in < -half) ? -half : ((in > half - 1) ? half - 1 : in);
            uint32_t const f  = pk_off_s(in, w);

            if ((f > pk_mask(w)) || (pk_unoff_s(f, w) != exp)) { ok = 0; }
        }
        /* unsigned: saturates at the field maximum */
        if ((pk_sat_u(pk_mask(w), w) != pk_mask(w)) || ((w < 32u) && (pk_sat_u(pk_mask(w) + 1u, w) != pk_mask(w)))) { ok = 0; }
        /* wrapping counters: modular difference within +-half */
        if ((w >= 8u) && ((pk_wrap_diff(2u, pk_mask(w) - 1u, w) != 4) || (pk_wrap_diff(pk_mask(w) - 1u, 2u, w) != -4))) { ok = 0; }
    }
    CHECK("SH-H05", ok);
    CHECK("SH-H05", pk_off_s(INT32_MIN, 32u) == 0u && pk_off_s(INT32_MAX, 32u) == 0xFFFFFFFFu);
    CHECK("SH-H05", pk_unoff_s(0u, 32u) == INT32_MIN && pk_unoff_s(0xFFFFFFFFu, 32u) == INT32_MAX);

    /* fields in a word: set one without touching its neighbours */
    {
        uint32_t wd = 0xFFFFFFFFu;

        wd = pk_set(wd, 5u, 7u, 0u);
        CHECK("SH-H05", wd == 0xFFFFF01Fu && pk_get(wd, 5u, 7u) == 0u && pk_get(wd, 0u, 5u) == 0x1Fu);
        wd = pk_set(wd, 5u, 7u, 0x55u);
        CHECK("SH-H05", pk_get(wd, 5u, 7u) == 0x55u && pk_get(wd, 12u, 20u) == 0xFFFFFu);
        CHECK("SH-H05", pk_set(0u, 0u, 32u, 0x12345678u) == 0x12345678u && pk_get(0x80000000u, 31u, 1u) == 1u);
    }

    /* streams: random widths and values round-trip; overflow is reported */
    {
        uint8_t     buf[64];
        uint32_t    vals[100];
        uint8_t     ws[100];
        pk_writer_t wr;
        pk_reader_t rd;
        unsigned    n = 0u;
        unsigned    k;
        int         rt = 1;

        srand(77);
        pk_writer_init(&wr, buf, sizeof(buf));
        while (n < 100u)
        {
            ws[n]   = (uint8_t)(1 + (rand() % 32));
            vals[n] = ((uint32_t)rand() << 16 ^ (uint32_t)rand()) & pk_mask(ws[n]);
            if ((unsigned)wr.bits + ws[n] > sizeof(buf) * 8u) { break; }
            pk_put(&wr, vals[n], ws[n]);
            n++;
        }
        pk_reader_init(&rd, buf, pk_writer_len(&wr));
        for (k = 0u; k < n; k++)
        {
            if (pk_take(&rd, ws[k]) != vals[k]) { rt = 0; }
        }
        CHECK("SH-H05", rt && !rd.b_short && n > 10u);
        pk_put(&wr, 0xFFFFFFFFu, 32u);
        pk_put(&wr, 0u, 33u);
        CHECK("SH-H05", wr.b_overflow && pk_writer_len(&wr) == 0u);
    }
}

int main(void)
{
    int32_t a;
    int     worst_sin = 0;
    double  worst_atan = 0.0;
    uint32_t x;

    test_pack();

    /* SH-H01: sin/cos over -720..720 deg in 0.07 deg steps */
    for (a = -72000; a <= 72000; a += 7)
    {
        int ref = (int)lround(sin((double)a * M_PI / 18000.0) * 32768.0);
        int got = app_sin_q15(a);
        int d;

        if (ref > 32767) { ref = 32767; }
        d = abs(got - ref);
        if (d > worst_sin) { worst_sin = d; }
        ref = (int)lround(cos((double)a * M_PI / 18000.0) * 32768.0);
        if (ref > 32767) { ref = 32767; }
        d = abs(app_cos_q15(a) - ref);
        if (d > worst_sin) { worst_sin = d; }
    }
    CHECK("SH-H01", worst_sin <= 2);

    /* SH-H02: atan2 on a grid of vectors, all quadrants */
    {
        int32_t xs;
        int32_t ys;

        for (xs = -3000; xs <= 3000; xs += 37)
        {
            for (ys = -3000; ys <= 3000; ys += 41)
            {
                double ref = atan2((double)ys, (double)xs) * 18000.0 / M_PI;
                double d   = fabs((double)app_atan2_cdeg(ys, xs) - ref);

                if (d > 18000.0) { d = 36000.0 - d; }   /* +-180 seam */
                if (d > worst_atan) { worst_atan = d; }
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
    for (x = 0u; x < 70000u; x++)
    {
        uint64_t sq = (uint64_t)x * x;

        if ((app_isqrt64(sq) != x) || ((x > 0u) && (app_isqrt64(sq - 1u) != (x - 1u))))
        {
            CHECK("SH-H03", 0);
            break;
        }
        if ((sq <= 0xFFFFFFFFu) && (app_isqrt32((uint32_t)sq) != x))
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
