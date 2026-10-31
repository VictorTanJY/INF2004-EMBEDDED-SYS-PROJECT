/*
 *  test_math.c
 *  Host unit tests for the shared integer maths (app_program/app_math.c).
 *  Build and run:  make -C testing/shared/host
 */

#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include "app_math.h"

static int g_pass;
static int g_fail;

#define CHECK(id, cond) \
    do { if (cond) { g_pass++; } else { g_fail++; \
        printf("FAIL %s  (%s:%d)  %s\n", id, __FILE__, __LINE__, #cond); } } while (0)

int main(void)
{
    int32_t a;
    int     worst_sin = 0;
    double  worst_atan = 0.0;
    uint32_t x;

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
