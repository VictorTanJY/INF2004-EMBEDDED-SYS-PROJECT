/*
 *  app_math.c
 *  Integer trigonometry and square root.  See app_math.h.
 */

#include "app_math.h"

uint32_t app_isqrt64(uint64_t value)
{
    uint64_t result = 0u;
    uint64_t bit    = (uint64_t)1u << 62;

    while (bit > value)
    {
        bit >>= 2;
    }
    while (bit != 0u)
    {
        if (value >= (result + bit))
        {
            value      -= result + bit;
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

uint32_t app_isqrt32(uint32_t value)
{
    uint32_t result = 0u;
    uint32_t bit    = (uint32_t)1u << 30;

    while (bit > value)
    {
        bit >>= 2;
    }
    while (bit != 0u)
    {
        if (value >= (result + bit))
        {
            value      -= result + bit;
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
static int32_t atan_unit_cdeg(int64_t ratio)
{
    int64_t const pi_q30 = 3373259426;          /* pi * 2^30 */
    int64_t const ratio_sq     = (ratio * ratio) >> 30;
    int64_t       poly;

    poly = 22371518;                               /*  0.0208351 */
    poly = -91410863 + ((ratio_sq * poly) >> 30);           /* -0.0851330 */
    poly = 193424926 + ((ratio_sq * poly) >> 30);           /*  0.1801410 */
    poly = -354656388 + ((ratio_sq * poly) >> 30);          /* -0.3302995 */
    poly = 1073597943 + ((ratio_sq * poly) >> 30);          /*  0.9998660 */
    poly = (ratio * poly) >> 30;                          /* atan(z), Q30 rad, >= 0 */

    /* rad -> cdeg: x 18000 / pi, rounded */
    return (int32_t)(((poly * 18000) + (pi_q30 / 2)) / pi_q30);
}

int32_t app_atan2_cdeg(int32_t y_val, int32_t x_val)
{
    uint32_t abs_x = (x_val < 0) ? (0u - (uint32_t)x_val) : (uint32_t)x_val;
    uint32_t abs_y = (y_val < 0) ? (0u - (uint32_t)y_val) : (uint32_t)y_val;
    int32_t  angle;

    if ((abs_x == 0u) && (abs_y == 0u))
    {
        return 0;
    }
    if (abs_x >= abs_y)
    {
        angle = atan_unit_cdeg((int64_t)(((uint64_t)abs_y << 30) / abs_x));
    }
    else
    {
        angle = APP_CDEG_90 - atan_unit_cdeg((int64_t)(((uint64_t)abs_x << 30) / abs_y));
    }
    if (x_val < 0)
    {
        angle = APP_CDEG_180 - angle;
    }
    if (y_val < 0)
    {
        angle = -angle;
    }
    return angle;
}

int32_t app_wrap_cdeg(int32_t cdeg)
{
    int32_t angle = cdeg % APP_CDEG_360;

    if (angle >= APP_CDEG_180)
    {
        angle -= APP_CDEG_360;
    }
    else if (angle < -APP_CDEG_180)
    {
        angle += APP_CDEG_360;
    }
    else
    {
        /* already in range */
    }
    return angle;
}

/*
 * sin(t) by its Taylor series to t^9 on |t| <= pi/2, in Q30.
 * Truncation error < (pi/2)^11 / 11! = 3.6e-6.
 */
int32_t app_sin_q15(int32_t cdeg)
{
    int32_t angle    = app_wrap_cdeg(cdeg);         /* -180 .. 180 deg */
    int64_t term;
    int64_t term_sq;
    int64_t sine;

    /* Fold into -90 .. 90 deg: sin(180 - a) = sin(a). */
    if (angle > APP_CDEG_90)
    {
        angle = APP_CDEG_180 - angle;
    }
    else if (angle < -APP_CDEG_90)
    {
        angle = -APP_CDEG_180 - angle;
    }
    else
    {
        /* already folded */
    }

    /* t = a * pi / 18000 in Q30: pi / 18000 * 2^30 = 187406.71 */
    term  = ((int64_t)angle * 18740671) / 100;
    term_sq = (term * term) >> 30;

    /* Horner: t (1 - t2/6 (1 - t2/20 (1 - t2/42 (1 - t2/72)))) */
    sine = ((int64_t)1 << 30) - (term_sq / 72);
    sine = ((int64_t)1 << 30) - (((term_sq * sine) >> 30) / 42);
    sine = ((int64_t)1 << 30) - (((term_sq * sine) >> 30) / 20);
    sine = ((int64_t)1 << 30) - (((term_sq * sine) >> 30) / 6);
    sine = (term * sine) >> 30;                          /* Q30 */

    sine = (sine + ((sine >= 0) ? 16384 : -16384)) / 32768;  /* Q15, rounded */
    if (sine > 32767)
    {
        sine = 32767;
    }
    if (sine < -32768)
    {
        sine = -32768;
    }
    return (int32_t)sine;
}

int32_t app_cos_q15(int32_t cdeg)
{
    return app_sin_q15(app_wrap_cdeg(cdeg) + APP_CDEG_90);
}

int32_t app_mul_q15(int32_t value, int32_t q15)
{
    return (int32_t)(((int64_t)value * (int64_t)q15) / 32768);
}
