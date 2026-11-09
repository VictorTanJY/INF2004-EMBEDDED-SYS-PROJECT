/*
 *  app_math.c
 *  Integer trigonometry and square root.  See app_math.h.
 */

#include "app_math.h"

uint32_t app_isqrt64(uint64_t x)
{
    uint64_t result = 0u;
    uint64_t bit    = (uint64_t)1u << 62;

    while (bit > x)
    {
        bit >>= 2;
    }
    while (bit != 0u)
    {
        if (x >= (result + bit))
        {
            x      -= result + bit;
            result  = (result >> 1) + bit;
        }
        else
        {
            result >>= 1;
        }
        bit >>= 2;
    }
    return (uint32_t)result;
}

uint32_t app_isqrt32(uint32_t x)
{
    uint32_t result = 0u;
    uint32_t bit    = (uint32_t)1u << 30;

    while (bit > x)
    {
        bit >>= 2;
    }
    while (bit != 0u)
    {
        if (x >= (result + bit))
        {
            x      -= result + bit;
            result  = (result >> 1) + bit;
        }
        else
        {
            result >>= 1;
        }
        bit >>= 2;
    }
    return result;
}

/*
 * atan(z) for 0 <= z <= 1, z in Q30, result in cdeg.  Abramowitz & Stegun
 * 4.4.49: atan(z) = z (a1 + a3 z^2 + a5 z^4 + a7 z^6 + a9 z^8), |err| <= 1e-5 rad.
 */
static int32_t atan_unit_cdeg(int64_t z)
{
    int64_t const pi_q30 = 3373259426;          /* pi * 2^30 */
    int64_t const z2     = (z * z) >> 30;
    int64_t       p;

    p = 22371518;                               /*  0.0208351 */
    p = -91410863 + ((z2 * p) >> 30);           /* -0.0851330 */
    p = 193424926 + ((z2 * p) >> 30);           /*  0.1801410 */
    p = -354656388 + ((z2 * p) >> 30);          /* -0.3302995 */
    p = 1073597943 + ((z2 * p) >> 30);          /*  0.9998660 */
    p = (z * p) >> 30;                          /* atan(z), Q30 rad, >= 0 */

    /* rad -> cdeg: x 18000 / pi, rounded */
    return (int32_t)(((p * 18000) + (pi_q30 / 2)) / pi_q30);
}

int32_t app_atan2_cdeg(int32_t y, int32_t x)
{
    uint32_t ax = (x < 0) ? (0u - (uint32_t)x) : (uint32_t)x;
    uint32_t ay = (y < 0) ? (0u - (uint32_t)y) : (uint32_t)y;
    int32_t  angle;

    if ((ax == 0u) && (ay == 0u))
    {
        return 0;
    }
    if (ax >= ay)
    {
        angle = atan_unit_cdeg((int64_t)(((uint64_t)ay << 30) / ax));
    }
    else
    {
        angle = APP_CDEG_90 - atan_unit_cdeg((int64_t)(((uint64_t)ax << 30) / ay));
    }
    if (x < 0)
    {
        angle = APP_CDEG_180 - angle;
    }
    if (y < 0)
    {
        angle = -angle;
    }
    return angle;
}

int32_t app_wrap_cdeg(int32_t cdeg)
{
    int32_t a = cdeg % APP_CDEG_360;

    if (a >= APP_CDEG_180)
    {
        a -= APP_CDEG_360;
    }
    else if (a < -APP_CDEG_180)
    {
        a += APP_CDEG_360;
    }
    else
    {
        /* already in range */
    }
    return a;
}

/*
 * sin(t) by its Taylor series to t^9 on |t| <= pi/2, in Q30.
 * Truncation error < (pi/2)^11 / 11! = 3.6e-6.
 */
int32_t app_sin_q15(int32_t cdeg)
{
    int32_t a    = app_wrap_cdeg(cdeg);         /* -180 .. 180 deg */
    int64_t t;
    int64_t t2;
    int64_t s;

    /* Fold into -90 .. 90 deg: sin(180 - a) = sin(a). */
    if (a > APP_CDEG_90)
    {
        a = APP_CDEG_180 - a;
    }
    else if (a < -APP_CDEG_90)
    {
        a = -APP_CDEG_180 - a;
    }
    else
    {
        /* already folded */
    }

    /* t = a * pi / 18000 in Q30: pi / 18000 * 2^30 = 187406.71 */
    t  = ((int64_t)a * 18740671) / 100;
    t2 = (t * t) >> 30;

    /* Horner: t (1 - t2/6 (1 - t2/20 (1 - t2/42 (1 - t2/72)))) */
    s = ((int64_t)1 << 30) - (t2 / 72);
    s = ((int64_t)1 << 30) - (((t2 * s) >> 30) / 42);
    s = ((int64_t)1 << 30) - (((t2 * s) >> 30) / 20);
    s = ((int64_t)1 << 30) - (((t2 * s) >> 30) / 6);
    s = (t * s) >> 30;                          /* Q30 */

    s = (s + ((s >= 0) ? 16384 : -16384)) / 32768;  /* Q15, rounded */
    if (s > 32767)
    {
        s = 32767;
    }
    if (s < -32768)
    {
        s = -32768;
    }
    return (int32_t)s;
}

int32_t app_cos_q15(int32_t cdeg)
{
    return app_sin_q15(app_wrap_cdeg(cdeg) + APP_CDEG_90);
}

int32_t app_mul_q15(int32_t value, int32_t q15)
{
    return (int32_t)(((int64_t)value * (int64_t)q15) / 32768);
}
