/*
 *  test_s3_main.c
 *  Host unit tests for subsystem 3: Code 39 decoding (code39.c), barcode
 *  capture from a 1 kHz sample stream (line_barcode.c) and line tracking
 *  (line_track.c).  The barcode tests use the element widths of the
 *  instructor's sample sheets (vectors_slides.h): A, F and Z at all three
 *  printed sizes, plus W, X and Y.
 *  Build and run:  make -C testing/subsystem3/host
 */

#include "test_s3_main.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "app_pack.h"
#include <math.h>
#include <stdbool.h>
#include "code39.h"
#include "line_barcode.h"
#include "line_track.h"
#include "app_types.h"
#include "app_math.h"
#include "vectors_slides.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define T_SYMBOLS        3   /* *X*: start, data, stop */
#define T_CHAR_ELEMS     9   /* bars and spaces in one character */
#define T_CODE_ELEMS     29  /* elements of *X* */
#define T_CODE_ELEMS_U   29u /* the same, for unsigned counters */
#define T_NARROW_WIDTHS  3   /* narrow bar widths tried */
#define T_RATIOS         3   /* wide:narrow ratios tried */
#define T_DIRECTIONS     2   /* forwards, reversed */
#define T_DATA_FIRST     10  /* first element of the data character */
#define T_DATA_END       19  /* one past its last element */
#define T_DATA_FIRST_U   10u
#define T_DATA_END_U     19u
#define T_AB_CHARS       4     /* "*AB*" */
#define T_SPEEDS         2     /* 0.15 and 0.30 m/s */
#define T_NOISY_PASSES   20    /* per character, speed and direction */
#define T_BLUR_SPOTS     2     /* entries of spots[] in the blur tests */
#define T_WEAK_SPOTS     3     /* entries of spots[] in the weak-sensor test */
#define T_WEAK_SPEEDS    3     /* entries of speeds[] */
#define T_THIN_LEVELS    3     /* entries of thin[] */
#define T_CODE_ENDS      2     /* first bar, last bar */
#define T_LETTERS        26u   /* A..Z */
#define T_BLUR_NOISE     40    /* sensor noise, norm counts */
#define T_BLEED_NOISE    30    /* sensor noise, norm counts */
#define T_BLEED_CASES    4     /* 31 or 33 elements, each way round */
#define T_ALTERNATIONS   40    /* blocks alternating left / right */
#define T_RAMP_BLOCKS    17    /* blocks of one-sided error */
#define T_CAL_SAMPLES    100   /* calibration samples */
#define T_SLIP_STEPS     400   /* spin steps of 0.9 deg */
#define T_GAINS          4     /* entries of gains[] */
#define T_DAMAGE_KINDS   4     /* split, merge, both, end lost */
#define T_RANDOM_CODES   20000 /* random element lists */
#define T_TRIES_PER_CHAR 40    /* random damage per character */
#define T_DAMAGES_U      3u    /* random merges or splits */

#define CHECK(id, cond)                                                      \
    do                                                                       \
    {                                                                        \
        if (cond)                                                            \
        {                                                                    \
            g_pass++;                                                        \
        }                                                                    \
        else                                                                 \
        {                                                                    \
            g_fail++;                                                        \
            printf("FAIL %s  (%s:%d)  %s\n", id, __FILE__, __LINE__, #cond); \
        }                                                                    \
    } while (0)

#define N_SLIDES ((int32_t)SLIDE_VEC_COUNT)

/* ---- sample-stream simulation ---------------------------------------- */
#define TRACK_LEN_UM 1200000u /* 1.2 m of track per pass */

static int32_t g_pass;
static int32_t g_fail;

static char const g_data_chars[] =
    "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ-. $/+%";

static uint32_t g_rng = 12345u;

/* Reflectance (0..1000, 1000 = black) at position x along a track with
   60 mm of white, then the code, then white.  Sensor spot: a box of
   g_spot_um (1.5 mm; S3-H15 widens it to blur the 3 mm elements). */
static uint32_t g_spot_um = 1500u;
static uint32_t g_white   = 80u;
static uint32_t g_black   = 920u;
static uint8_t  g_last_pass =
    0u; /* which reading decoded in the last run_pass */
static uint32_t g_thin_black =
    0u; /* S3-H19: level of bars narrower than g_thin_um (0 = g_black) */
static uint32_t g_thin_um = 0u;

/* ---- line tracking ---------------------------------------------------- */
static line_track_cfg_t const g_cfg = {20000u, 80000u, 300};

/* S3-H16 (3 Nov): turn slip from the calibration spin.  Model: the car
   spins about its axle on a straight 18 mm line through the axle; the
   line sensors are OBST_IR_AHEAD 120 mm ahead, 9 mm either side (both
   half on the line at the start); the encoders count `gain` x the real
   rotation; a sensor reads the fraction of its 8 mm spot on black. */
static float64_t g_slip_half = 9.0; /* sensor offset from the centreline, mm */

static uint16_t  make_code(char chr, uint32_t narrow, float64_t ratio,
                           float64_t jitter, uint32_t * p_w);
static void      reverse(uint32_t * p_w, uint16_t n_elems);
static void      test_synthetic(void);
static void      test_rejects(void);
static void      test_capture(void);
static uint32_t  rnd(void);
static float64_t rndu(void);
static uint16_t  profile_at(uint32_t const * p_w, uint16_t n_elems,
                            uint32_t x_um);
static line_barcode_result_t run_pass(uint32_t const * p_w, uint16_t n_elems,
                                      uint32_t v_mm_s, int32_t noise,
                                      line_barcode_out_t * p_out);
static void                  test_table(void);
static void                  test_slides(void);
static uint8_t  feed(line_track_t * p_t, uint16_t left, uint16_t ret,
                     uint32_t dist_um);
static void     test_track(void);
static uint16_t slip_raw(float64_t x_val, float64_t y_val);
static uint16_t slip_run(int32_t gain_pm, bool b_line, int32_t start_cdeg);
static void     test_slip(void);
static uint16_t split_at(uint32_t const * p_in, uint16_t n_elems,
                         uint16_t at_idx, uint32_t speck, uint32_t * p_out);
static uint16_t merge_at(uint32_t const * p_in, uint16_t n_elems,
                         uint16_t at_idx, uint32_t * p_out);
static bool     thin_triplet(uint32_t const * p_w, uint16_t at_idx,
                             uint32_t narrow);
static void     test_best(void);
static void     test_light_print(void);
static void     test_pack_s3(void);

int main (void)
{
    test_table();
    test_slides();
    test_synthetic();
    test_rejects();
    test_capture();
    test_best();
    test_light_print();
    test_track();
    test_slip();
    test_pack_s3();
    printf("host tests: %d passed, %d failed\n", g_pass, g_fail);
    return (0 == g_fail) ? 0 : 1;
}

/* Widths of *c* with the given narrow width, ratio and +-jitter. */
static uint16_t make_code (char chr, uint32_t narrow, float64_t ratio,
                           float64_t jitter, uint32_t * p_w)
{
    char const seq[3]  = {'*', chr, '*'};
    uint16_t   n_elems = 0u;
    int32_t    kdx;
    int32_t    elem;

    for (kdx = 0; kdx < T_SYMBOLS; kdx++)
    {
        uint16_t pat = code39_pattern(seq[kdx]);

        for (elem = 0; elem < T_CHAR_ELEMS; elem++)
        {
            float64_t omega =
                (float64_t)narrow * (((pat >> (8 - elem)) & 1u) ? ratio : 1.0);
            p_w[n_elems++] = (uint32_t)(omega * (1.0 + (jitter * rndu())));
        }
        if (kdx < 2)
        {
            p_w[n_elems++] = (uint32_t)((float64_t)narrow
                                        * (1.0 + (jitter * rndu()))); /* gap */
        }
    }
    return n_elems;
}

static void reverse (uint32_t * p_w, uint16_t n_elems)
{
    uint16_t idx;

    for (idx = 0u; idx < (n_elems / 2u); idx++)
    {
        uint32_t elapsed          = p_w[idx];
        p_w[idx]                  = p_w[(n_elems - 1u) - idx];
        p_w[(n_elems - 1u) - idx] = elapsed;
    }
}

static void test_synthetic (void)
{
    uint32_t const narrows[3] = {3000u, 5100u, 6900u};
    bool           b_ok       = true;
    int32_t        wrong      = 0;
    size_t         idx;
    int32_t        val_a;
    int32_t        ret;
    int32_t        diff;

    for (idx = 0u; idx < (sizeof(g_data_chars) - 1u); idx++)
    {
        for (val_a = 0; val_a < T_NARROW_WIDTHS; val_a++)
        {
            for (ret = 0; ret < T_RATIOS; ret++)
            {
                float64_t const ratio =
                    (0 == ret) ? 2.2 : ((1 == ret) ? 2.5 : 3.0);

                for (diff = 0; diff < T_DIRECTIONS; diff++)
                {
                    uint32_t widths[29];
                    uint16_t n_elems = make_code(
                        g_data_chars[idx], narrows[val_a], ratio, 0.15, widths);
                    char            chr;
                    bool            b_rev;
                    code39_result_t res;

                    if (diff)
                    {
                        reverse(widths, n_elems);
                    }
                    res = code39_decode_one(widths, n_elems, 15, &chr, &b_rev);
                    if (res != CODE39_OK)
                    {
                        b_ok = false;
                    }
                    else if (chr != g_data_chars[idx])
                    {
                        wrong++;
                    }
                    else
                    {
                        /* No action required. */
                    }
                }
            }
        }
    }
    CHECK("S3-H03", b_ok && (0 == wrong));
}

static void test_rejects (void)
{
    uint32_t widths[40];
    uint16_t n_elems;
    char     chr;
    bool     b_rev;
    int32_t  idx;
    int32_t  wrong = 0;

    /* a wide bar printed narrow: rejected, never a different character */
    for (idx = 0; idx < (int32_t)(sizeof(g_data_chars) - 1u); idx++)
    {
        int32_t elem;
        for (elem = T_DATA_FIRST; elem < T_DATA_END; elem++)
        {
            uint16_t pat = code39_pattern(g_data_chars[idx]);

            n_elems = make_code(g_data_chars[idx], 3000u, 3.0, 0.0, widths);
            if ((pat >> (8 - (elem - 10))) & 1u)
            {
                widths[elem] = 3000u; /* wide -> narrow */
                if (CODE39_OK
                    == code39_decode_one(widths, n_elems, 15, &chr, &b_rev))
                {
                    wrong++;
                }
            }
        }
    }
    CHECK("S3-H04", 0 == wrong);
    n_elems = make_code('A', 3000u, 3.0, 0.0, widths);
    CHECK("S3-H04",
          CODE39_ERR_COUNT == code39_decode_one(widths, 28, 15, &chr, &b_rev));
    n_elems = make_code('A', 3000u, 1.3, 0.0, widths);
    CHECK("S3-H04",
          CODE39_ERR_RATIO
              == code39_decode_one(widths, n_elems, 15, &chr, &b_rev));
    CHECK("S3-H04",
          CODE39_NOT_BARCODE == code39_decode_one(widths, 5, 15, &chr, &b_rev));
    {
        /* "*AB*": valid Code 39 but two data characters */
        uint32_t   words_out[40];
        char const seq[4] = {'*', 'A', 'B', '*'};
        int32_t    kdx;
        int32_t    elem;
        n_elems = 0u;
        for (kdx = 0; kdx < T_AB_CHARS; kdx++)
        {
            uint16_t pat = code39_pattern(seq[kdx]);
            for (elem = 0; elem < T_CHAR_ELEMS; elem++)
            {
                words_out[n_elems++] =
                    ((pat >> (8 - elem)) & 1u) ? 9000u : 3000u;
            }
            if (kdx < 3)
            {
                words_out[n_elems++] = 3000u;
            }
        }
        CHECK("S3-H04",
              CODE39_ERR_LENGTH
                  == code39_decode_one(words_out, n_elems, 15, &chr, &b_rev));
    }
}

static void test_capture (void)
{
    uint32_t const speeds[2] = {150u, 300u}; /* NFR3 speeds, mm/s */
    int32_t        step;
    int32_t        value;
    int32_t        diff;
    int32_t        pass;
    int32_t        good  = 0;
    int32_t        total = 0;
    int32_t        wrong = 0;

    /* every slide, both speeds, both directions, clean */
    for (step = 0; step < N_SLIDES; step++)
    {
        for (value = 0; value < T_SPEEDS; value++)
        {
            for (diff = 0; diff < T_DIRECTIONS; diff++)
            {
                uint32_t           widths[29];
                line_barcode_out_t out;
                int32_t            kdx;

                for (kdx = 0; kdx < T_CODE_ELEMS; kdx++)
                {
                    widths[kdx] = (g_slides[step].p_w[kdx] * 297u) / 339u;
                }
                if (diff)
                {
                    reverse(widths, 29);
                }
                memset(&out, 0, sizeof(out));
                CHECK("S3-H05",
                      (BC_VALID == run_pass(widths, 29, speeds[value], 0, &out))
                          && (out.chr == g_slides[step].chr));
            }
        }
    }

    /* NFR3-style: 20 noisy passes per character per speed and direction */
    for (step = 0; step < N_SLIDES; step += 3)
    {
        for (value = 0; value < T_SPEEDS; value++)
        {
            for (pass = 0; pass < T_NOISY_PASSES; pass++)
            {
                uint32_t              widths[29];
                line_barcode_out_t    out;
                line_barcode_result_t result;
                int32_t               kdx;

                for (kdx = 0; kdx < T_CODE_ELEMS; kdx++)
                {
                    widths[kdx] =
                        (uint32_t)((((float64_t)g_slides[step].p_w[kdx] * 297.0)
                                    / 338.67)
                                   * (1.0 + (0.10 * rndu())));
                }
                if (pass % 2)
                {
                    reverse(widths, 29);
                }
                memset(&out, 0, sizeof(out));
                result = run_pass(widths, 29, speeds[value], 120, &out);
                total++;
                if ((BC_VALID == result) && (out.chr == g_slides[step].chr))
                {
                    good++;
                }
                else if (BC_VALID == result)
                {
                    wrong++;
                }
                else
                {
                    /* No action required. */
                }
            }
        }
    }
    printf("noisy passes: %d / %d decoded, %d wrong\n", good, total, wrong);
    CHECK("S3-H05", (0 == wrong) && ((good * 100) >= (total * 95)));

    /* S3-H15: blurred sensor spot (2 Nov, the robot saw 3 elements: thin
       bars and thin gaps never crossed the old fixed 350 / 220 levels).  A
       3.5 / 4 mm spot over 3 mm elements (thin bars peak ~670, thin gaps
       ~330 inside wide bars): every slide decodes, both speeds and
       directions, clean and noisy (old levels: 32 / 48 and 0 / 48).  A weak
       sensor (white 250, black 800, 4 mm spot): at least 2 of 3 slides
       decode, never a wrong character (old levels: none, all too long). */
    {
        uint32_t const spots[2] = {3500u, 4000u};
        int32_t        setpoint;
        int32_t        blur_good  = 0;
        int32_t        blur_total = 0;
        int32_t        blur_wrong = 0;
        int32_t        weak_good  = 0;
        int32_t        weak_total = 0;
        int32_t        weak_wrong = 0;

        for (setpoint = 0; setpoint < T_BLUR_SPOTS; setpoint++)
        {
            g_spot_um = spots[setpoint];
            for (step = 0; step < N_SLIDES; step++)
            {
                for (value = 0; value < T_SPEEDS; value++)
                {
                    for (diff = 0; diff < T_DIRECTIONS; diff++)
                    {
                        uint32_t              widths[29];
                        line_barcode_out_t    out;
                        line_barcode_result_t result;
                        int32_t               kdx;

                        for (kdx = 0; kdx < T_CODE_ELEMS; kdx++)
                        {
                            widths[kdx] =
                                (g_slides[step].p_w[kdx] * 297u) / 339u;
                        }
                        if (diff)
                        {
                            reverse(widths, 29);
                        }
                        memset(&out, 0, sizeof(out));
                        result = run_pass(widths, 29, speeds[value],
                                          (diff ? 60 : 0), &out);
                        blur_total++;
                        if ((BC_VALID == result)
                            && (out.chr == g_slides[step].chr))
                        {
                            blur_good++;
                        }
                        else if (BC_VALID == result)
                        {
                            blur_wrong++;
                        }
                        else
                        {
                            /* No action required. */
                        }
                    }
                }
            }
        }
        g_spot_um = 4000u;
        g_white   = 250u;
        g_black   = 800u;
        for (step = 0; step < N_SLIDES; step++)
        {
            uint32_t           widths[29];
            line_barcode_out_t out;
            int32_t            kdx;

            for (kdx = 0; kdx < T_CODE_ELEMS; kdx++)
            {
                widths[kdx] = (g_slides[step].p_w[kdx] * 297u) / 339u;
            }
            line_barcode_result_t result;

            memset(&out, 0, sizeof(out));
            result = run_pass(widths, 29, 150u, 0, &out);
            weak_total++;
            if ((BC_VALID == result) && (out.chr == g_slides[step].chr))
            {
                weak_good++;
            }
            else if (BC_VALID == result)
            {
                weak_wrong++;
            }
            else
            {
                /* No action required. */
            }
        }
        g_spot_um = 1500u;
        g_white   = 80u;
        g_black   = 920u;
        printf("blurred passes: %d / %d decoded, %d wrong; weak sensor %d / "
               "%d, %d wrong\n",
               blur_good, blur_total, blur_wrong, weak_good, weak_total,
               weak_wrong);
        CHECK("S3-H15", (0 == blur_wrong) && (blur_good == blur_total));
        CHECK("S3-H15",
              (0 == weak_wrong) && ((weak_good * 3) >= (weak_total * 2)));
    }

    /* S3-H18 (7 Nov): (a) a read one bar short at either end (2 elements
       lost) still decodes, both directions, never a wrong letter; (b) black
       bleeding into white (bars +40 % of a narrow, spaces -40 %) decodes
       with bars and spaces ranked apart; (c) a 5-6 mm spot over 3 mm
       elements: the profile's second try decodes more than the swing
       reading alone, never a wrong letter. */
    {
        int32_t good  = 0;
        int32_t total = 0;
        int32_t wrong = 0;
        int32_t kdx;

        for (step = 0; step < N_SLIDES; step++)
        {
            for (diff = 0; diff < T_DIRECTIONS; diff++)
            {
                int32_t cut;

                for (cut = 0; cut < T_CODE_ENDS; cut++)
                {
                    uint32_t widths[29];
                    char     chr;
                    bool     b_rev;

                    for (kdx = 0; kdx < T_CODE_ELEMS; kdx++)
                    {
                        widths[kdx] = g_slides[step].p_w[kdx];
                    }
                    if (diff)
                    {
                        reverse(widths, 29);
                    }
                    total++;
                    if (CODE39_OK
                        == code39_decode_one_repair(cut ? widths : &widths[2],
                                                    27, 15, &chr, &b_rev))
                    {
                        if (chr == g_slides[step].chr)
                        {
                            good++;
                        }
                        else
                        {
                            wrong++;
                        }
                    }
                }
            }
        }
        printf("one bar short at an end: %d / %d decoded, %d wrong\n", good,
               total, wrong);
        CHECK("S3-H18", (0 == wrong) && (good == total));

        good  = 0;
        total = 0;
        wrong = 0;
        {
            size_t idx;

            for (idx = 0u; idx < T_LETTERS; idx++)
            {
                uint32_t widths[29];
                uint16_t n_elems =
                    make_code((char)('A' + idx), 3000u, 2.5, 0.0, widths);
                char     chr;
                bool     b_rev;
                uint16_t elem;

                for (elem = 0u; elem < n_elems; elem++)
                {
                    widths[elem] =
                        (0u == (elem % 2u))
                            ? (widths[elem] + 1200u)
                            : ((widths[elem] > 1200u) ? (widths[elem] - 1200u)
                                                      : 100u);
                }
                total++;
                if (CODE39_OK
                    == code39_decode_one(widths, n_elems, 15, &chr, &b_rev))
                {
                    if (chr == (char)('A' + idx))
                    {
                        good++;
                    }
                    else
                    {
                        wrong++;
                    }
                }
            }
        }
        printf("black bleeding 40 %%: %d / %d decoded, %d wrong\n", good, total,
               wrong);
        CHECK("S3-H18", (0 == wrong) && (good == total));

        {
            uint32_t const spots[2] = {5000u, 6000u};
            int32_t        setpoint;
            int32_t        pass2 = 0;

            good  = 0;
            total = 0;
            wrong = 0;
            for (setpoint = 0; setpoint < T_BLUR_SPOTS; setpoint++)
            {
                g_spot_um = spots[setpoint];
                for (step = 0; step < N_SLIDES; step++)
                {
                    for (diff = 0; diff < T_DIRECTIONS; diff++)
                    {
                        uint32_t              widths[29];
                        line_barcode_out_t    out;
                        line_barcode_result_t result;

                        for (kdx = 0; kdx < T_CODE_ELEMS; kdx++)
                        {
                            widths[kdx] =
                                (g_slides[step].p_w[kdx] * 297u) / 339u;
                        }
                        if (diff)
                        {
                            reverse(widths, 29);
                        }
                        memset(&out, 0, sizeof(out));
                        result = run_pass(widths, 29, 90u, 0, &out);
                        total++;
                        if ((BC_VALID == result)
                            && (out.chr == g_slides[step].chr))
                        {
                            good++;
                        }
                        else if (BC_VALID == result)
                        {
                            wrong++;
                        }
                        else
                        {
                            /* No action required. */
                        }
                        pass2 += (g_last_pass != 0u) ? 1 : 0;
                    }
                }
            }
            g_spot_um = 1500u;
            printf("5-6 mm spot: %d / %d decoded (%d by the second try), %d "
                   "wrong\n",
                   good, total, pass2, wrong);
            CHECK("S3-H18", 0 == wrong);
        }
    }

    /* S3-H19 (7 Nov 18:33 robot runs): "*Z*", the robot's levels while
       driving: white ~20, wide bars ~690, 3 mm bars too faint for the
       swing reading (here 240: a rise of 220 < BC_DELTA).  Only the 9 mm
       bars were found; inside the Z the white between them reaches 33 mm,
       which with a 25 mm end stopped every capture at 3 elements / 51 mm.
       With the 40 mm end the whole code is kept and the darkness record
       (low threshold) finds the faint bars. */
    {
        int32_t step;
        int32_t diff;
        int32_t good  = 0;
        int32_t total = 0;
        int32_t wrong = 0;
        int32_t noise;

        g_spot_um    = 1500u;
        g_white      = 20u;
        g_black      = 690u;
        g_thin_black = 240u;
        for (noise = 0; noise <= T_BLUR_NOISE; noise += T_BLUR_NOISE)
        {
            for (step = 0; step < N_SLIDES; step++)
            {
                if ('Z' == g_slides[step].chr)
                {
                    for (diff = 0; diff < T_DIRECTIONS; diff++)
                    {
                        uint32_t              widths[29];
                        line_barcode_out_t    out;
                        line_barcode_result_t result;
                        int32_t               kdx;

                        uint32_t wmin = 0xFFFFFFFFu;

                        /* printed at the specified size: 3 mm narrow (~141 mm
                         * code)
                         */
                        for (kdx = 0; kdx < T_CODE_ELEMS; kdx++)
                        {
                            wmin = (g_slides[step].p_w[kdx] < wmin)
                                       ? g_slides[step].p_w[kdx]
                                       : wmin;
                        }
                        for (kdx = 0; kdx < T_CODE_ELEMS; kdx++)
                        {
                            widths[kdx] =
                                (g_slides[step].p_w[kdx] * 3000u) / wmin;
                        }
                        g_thin_um = 6000u; /* bars under 6 mm are the faint
                                              narrow ones */
                        if (diff)
                        {
                            reverse(widths, 29);
                        }
                        memset(&out, 0, sizeof(out));
                        result = run_pass(widths, 29, 90u, noise, &out);
                        total++;
                        if ((BC_VALID == result) && ('Z' == out.chr))
                        {
                            good++;
                        }
                        else if (BC_VALID == result)
                        {
                            wrong++;
                        }
                        else
                        {
                            printf(
                                "  Z slide %d dir %d noise %d: result %d, %u "
                                "elements, %u mm\n",
                                g_slides[step].slide, diff, noise,
                                (int32_t)result, (uint32_t)out.elements,
                                (uint32_t)(out.length_um / 1000u));
                        }
                    }
                }
            }
        }
        g_spot_um    = 1500u;
        g_white      = 80u;
        g_black      = 920u;
        g_thin_black = 0u;
        printf("faint 3 mm bars (robot 7 Nov): Z %d / %d decoded, %d wrong\n",
               good, total, wrong);
        CHECK("S3-H19", (0 == wrong) && (good == total));
    }

    /* S3-H20 (14 Nov car runs, 1 mm slit on the sensor, 3 mm "*A*"): two
       reads with a 1.4 mm speck outside the code, one after the stop
       (31 elements) and one before the start with a wide bar split by a
       1 mm white speck (33).  Both were rejected for their count, and the
       best match that ran on four readings took over 50 ms and tripped
       the line task's heartbeat.  Widths as logged (LOG 75, 0.2 mm). */
    {
        static uint32_t const k_r1[31] = {
            2400u, 9000u, 3400u, 3400u, 8000u, 4000u, 9000u, 3600u,
            2600u, 3400u, 8000u, 4600u, 2600u, 4200u, 2000u, 10200u,
            2000u, 4200u, 8000u, 4200u, 2400u, 9600u, 2400u, 3400u,
            7600u, 4200u, 8000u, 4200u, 2600u, 8200u, 1400u};
        static uint32_t const k_r2[33] = {
            1400u, 2600u, 3000u, 8600u, 3400u, 2400u,  10000u, 2000u, 2400u,
            1000u, 9600u, 2400u, 2600u, 2000u, 10000u, 3000u,  2400u, 3400u,
            3600u, 9600u, 2400u, 3600u, 8200u, 3600u,  2400u,  9600u, 3000u,
            3400u, 8600u, 3400u, 9000u, 3400u, 3400u};
        int32_t diff;
        int32_t noise;
        int32_t good  = 0;
        int32_t total = 0;
        int32_t wrong = 0;

        g_spot_um = 1000u;
        g_white   = 25u;
        g_black   = 1000u;
        for (noise = 0; noise <= T_BLEED_NOISE; noise += T_BLEED_NOISE)
        {
            for (diff = 0; diff < T_BLEED_CASES; diff++)
            {
                uint32_t              widths[33];
                uint16_t const        n_elems = (diff < 2) ? 31u : 33u;
                line_barcode_out_t    out;
                line_barcode_result_t result;

                memcpy(widths, (diff < 2) ? k_r1 : k_r2,
                       n_elems * sizeof(uint32_t));
                if ((diff % 2) != 0)
                {
                    reverse(widths, n_elems);
                }
                memset(&out, 0, sizeof(out));
                result = run_pass(widths, n_elems, 90u, noise, &out);
                total++;
                if ((BC_VALID == result) && ('A' == out.chr)
                    && (out.b_reversed == ((diff % 2) != 0)))
                {
                    good++;
                }
                else if (BC_VALID == result)
                {
                    wrong++;
                }
                else
                {
                    printf("  14 Nov read %d dir %d noise %d: result %d reason "
                           "%u, %u elements\n",
                           (diff / 2) + 1, diff % 2, noise, (int32_t)result,
                           (uint32_t)out.reason, (uint32_t)out.elements);
                }
            }
        }
        g_spot_um = 1500u;
        g_white   = 80u;
        g_black   = 920u;
        printf("end specks (car 14 Nov): A %d / %d decoded, %d wrong\n", good,
               total, wrong);
        CHECK("S3-H20", (0 == wrong) && (good == total));
    }

    /* S3-H06: a crossing line (18 mm black) is not a barcode and not an error
     */
    {
        uint32_t           widths[1] = {18000u};
        line_barcode_out_t out;
        CHECK("S3-H06", BC_NONE == run_pass(widths, 1, 300u, 0, &out));
    }
    /* S3-H07: a long dark patch is rejected, never decoded */
    {
        uint32_t           widths[1] = {500000u};
        line_barcode_out_t out;
        memset(&out, 0, sizeof(out));
        CHECK("S3-H07", (BC_REJECT == run_pass(widths, 1, 300u, 0, &out))
                            && (CODE39_ERR_TOO_LONG == out.reason));
    }
}

static uint32_t rnd (void)
{
    g_rng = (g_rng * 1103515245u) + 12345u;
    return (g_rng >> 8) & 0xFFFFu;
}

/* uniform in [-1, 1] */
static float64_t rndu (void)
{
    return ((float64_t)rnd() / 32767.5) - 1.0;
}

static uint16_t profile_at (uint32_t const * p_w, uint16_t n_elems,
                            uint32_t x_um)
{
    uint32_t const lead = 60000u;
    uint32_t       spot;
    uint32_t       level = 0u;
    uint32_t       taps  = 0u;

    for (spot = 0u; spot < g_spot_um; spot += 100u)
    {
        taps++;
        uint32_t x_val = x_um + spot;
        uint32_t pos   = lead;
        uint16_t elem;
        uint32_t here = g_white;

        for (elem = 0u; elem < n_elems; elem++)
        {
            if ((x_val >= pos) && (x_val < (pos + p_w[elem])))
            {
                if (0u == (elem % 2u))
                {
                    here = ((g_thin_black != 0u) && (p_w[elem] < g_thin_um))
                               ? g_thin_black
                               : g_black;
                }
                break;
            }
            pos += p_w[elem];
        }
        level += here;
    }
    return (uint16_t)(level / taps);
}

/* Run one pass at v_mm_s; returns the result and fills *p_out. */
static line_barcode_result_t run_pass (uint32_t const * p_w, uint16_t n_elems,
                                       uint32_t v_mm_s, int32_t noise,
                                       line_barcode_out_t * p_out)
{
    static line_barcode_capture_t capture;
    uint32_t                      x_val;
    uint32_t                      step = v_mm_s; /* um per 1 ms sample = mm/s */
    line_barcode_result_t         found = BC_NONE;

    g_last_pass = 0u;
    line_barcode_init(&capture);
    for (x_val = 0u; x_val < TRACK_LEN_UM; x_val += step)
    {
        int32_t               sample = (int32_t)profile_at(p_w, n_elems, x_val)
                                       + (int32_t)(noise * rndu());
        line_barcode_out_t    out;
        line_barcode_result_t result;

        if (sample < 0)
        {
            sample = 0;
        }
        if (sample > 1000)
        {
            sample = 1000;
        }
        memset(&out, 0, sizeof(out));
        result = line_barcode_sample(&capture, (uint16_t)sample, step, &out);
        if ((result != BC_NONE) && (BC_NONE == found))
        {
            found       = result;
            *p_out      = out;
            g_last_pass = capture.pass;
        }
        else if (result != BC_NONE)
        {
            return BC_REJECT; /* more than one result per code */
        }
        else
        {
            /* No action required. */
        }
    }
    return found;
}

static void test_table (void)
{
    size_t idx;
    size_t jdx;
    bool   b_ok = true;

    for (idx = 0u; idx < (sizeof(g_data_chars) - 1u); idx++)
    {
        uint16_t pos     = code39_pattern(g_data_chars[idx]);
        int32_t  wide    = 0;
        int32_t  wide_sp = 0;
        int32_t  val_b;

        for (val_b = 0; val_b < T_CHAR_ELEMS; val_b++)
        {
            if ((pos >> (8 - val_b)) & 1u)
            {
                wide++;
                if (val_b % 2)
                {
                    wide_sp++;
                }
            }
        }
        if ((0u == pos) || (wide != 3) || ((wide_sp != 1) && (wide_sp != 3)))
        {
            b_ok = false;
        }
        if (code39_char(pos) != g_data_chars[idx])
        {
            b_ok = false;
        }
        for (jdx = 0u; jdx < idx; jdx++)
        {
            if (code39_pattern(g_data_chars[jdx]) == pos)
            {
                b_ok = false;
            }
        }
    }
    CHECK("S3-H01", b_ok);
    CHECK("S3-H01",
          (0x094u == code39_pattern('*')) && ('\0' == code39_char(0x1FFu)));
}

static void test_slides (void)
{
    int32_t step;

    for (step = 0; step < N_SLIDES; step++)
    {
        uint32_t  widths[29];
        char      chr;
        bool      b_rev;
        int32_t   kdx;
        float64_t scale;

        for (scale = 1.0; scale > 0.5;
             scale -= (1.0 - (297.0 / 338.67))) /* screen, A4 */
        {
            for (kdx = 0; kdx < T_CODE_ELEMS; kdx++)
            {
                widths[kdx] = (uint32_t)(g_slides[step].p_w[kdx] * scale);
            }
            CHECK("S3-H02",
                  (CODE39_OK == code39_decode_one(widths, 29, 15, &chr, &b_rev))
                      && (chr == g_slides[step].chr) && !b_rev);
            reverse(widths, 29);
            CHECK("S3-H02",
                  (CODE39_OK == code39_decode_one(widths, 29, 15, &chr, &b_rev))
                      && (chr == g_slides[step].chr) && b_rev);
            if (scale < 1.0)
            {
                break;
            }
        }
    }
}

/* Feed blocks of 3 mm (0.3 m/s x 10 ms) for dist_um; returns OR of events. */
static uint8_t feed (line_track_t * p_t, uint16_t left, uint16_t ret,
                     uint32_t dist_um)
{
    uint8_t  events = 0u;
    uint32_t diff;

    for (diff = 0u; diff < dist_um; diff += 3000u)
    {
        events |= line_track_step(p_t, &g_cfg, left, ret, 3000u);
    }
    return events;
}

static void test_track (void)
{
    line_track_t track;
    uint8_t      events;

    /* S3-H10: steering sign: line under the left sensor -> turn left */
    line_track_init(&track);
    (void)feed(&track, 500u, 500u, 60000u);
    CHECK("S3-H10",
          (0 == track.err) && (0 == track.omega) && (300 == track.v_mm_s));
    (void)line_track_step(&track, &g_cfg, 800u, 200u, 3000u);
    CHECK("S3-H10",
          (600 == track.err) && (track.omega > 0) && (track.v_mm_s < 300));
    (void)feed(&track, 200u, 800u, 9000u);
    CHECK("S3-H10", track.omega < 0);

    /* S3-H08: junction on a straight approach, once; not on a tight curve */
    line_track_init(&track);
    (void)feed(&track, 500u, 500u, 60000u);
    events = feed(&track, 1000u, 1000u, 15000u);
    CHECK("S3-H08", 0u == (events & LINE_TRK_EV_JUNCTION));
    events = feed(&track, 1000u, 1000u, 15000u);
    CHECK("S3-H08",
          ((events & LINE_TRK_EV_JUNCTION) != 0u) && (0 == track.omega));
    events = feed(&track, 1000u, 1000u, 30000u);
    CHECK("S3-H08", 0u == (events & LINE_TRK_EV_JUNCTION));
    line_track_init(&track);
    (void)feed(&track, 950u, 300u, 30000u); /* hard steering on a curve */
    events = feed(&track, 1000u, 1000u, 30000u);
    CHECK("S3-H08", 0u == (events & LINE_TRK_EV_JUNCTION));

    /* S3-H09: 50 mm gap bridged (NFR11, heading held), 90 mm -> LOST once, then
     * FOUND */
    line_track_init(&track);
    (void)feed(&track, 500u, 500u, 60000u);
    events = feed(&track, 50u, 50u, 50000u);
    CHECK("S3-H09", (0u == (events & LINE_TRK_EV_LOST)) && (0 == track.omega)
                        && (300 == track.v_mm_s));
    (void)feed(&track, 500u, 500u, 30000u);
    events = feed(&track, 50u, 50u, 90000u);
    CHECK("S3-H09", ((events & LINE_TRK_EV_LOST) != 0u) && track.b_lost
                        && (0 == track.v_mm_s));
    events = feed(&track, 50u, 50u, 90000u);
    CHECK("S3-H09", 0u == (events & LINE_TRK_EV_LOST));
    events = feed(&track, 600u, 400u, 3000u);
    CHECK("S3-H09", ((events & LINE_TRK_EV_FOUND) != 0u) && !track.b_lost);

    /* S3-H11: drifting off the right of the line -> keeps turning back left */
    line_track_init(&track);
    (void)feed(&track, 500u, 500u, 60000u);
    (void)feed(&track, 950u, 80u, 6000u); /* line under the left sensor only */
    (void)feed(&track, 60u, 60u, 9000u);  /* ... then past it */
    CHECK("S3-H11", (track.omega > 0) && (track.v_mm_s < 300));

    /* S3-H14: right curve, the line slides out past the right sensor and
       the error FADES (-1000 -> -300) before both read white: still a turn
       back right (was: taken for a gap, straight on); an old strong error
       (more than LINE_EXIT_PEAK_UM ago) does not count, a gap stays a gap */
    line_track_init(&track);
    (void)feed(&track, 500u, 500u, 60000u);
    (void)feed(&track, 0u, 1000u, 6000u); /* line under the right sensor */
    (void)feed(&track, 0u, 300u, 6000u);  /* sliding out: error fades */
    CHECK("S3-H14", -300 == track.err);
    (void)line_track_step(&track, &g_cfg, 60u, 60u, 3000u); /* both white */
    CHECK("S3-H14", (-1000 == track.exit_err) && (track.omega < 0)
                        && (track.v_mm_s < 300));
    line_track_init(&track);
    (void)feed(&track, 500u, 500u, 60000u);
    (void)feed(&track, 0u, 1000u, 6000u);
    (void)feed(&track, 500u, 500u, 45000u); /* centred again for 45 mm */
    (void)feed(&track, 60u, 60u, 9000u);    /* a gap */
    CHECK("S3-H14", (0 == track.omega) && (300 == track.v_mm_s));

    /* S3-H13: steering ramp: the error flips full left / full right every
       10 ms (the weaving seen on 2 Nov); the turn command never moves more
       than LINE_OMEGA_STEP per block; a steady error still reaches the full
       command within 850 / 50 = 17 blocks */
    {
        int32_t prev;
        int32_t idx;
        int32_t max_jump = 0;

        line_track_init(&track);
        (void)feed(&track, 500u, 500u, 30000u);
        prev = track.omega;
        for (idx = 0; idx < T_ALTERNATIONS; idx++)
        {
            if (0 == (idx % 2))
            {
                (void)line_track_step(&track, &g_cfg, 1000u, 0u, 3000u);
            }
            else
            {
                (void)line_track_step(&track, &g_cfg, 0u, 1000u, 3000u);
            }
            max_jump = (app_types_abs_i32(track.omega - prev) > max_jump)
                           ? app_types_abs_i32(track.omega - prev)
                           : max_jump;
            prev     = track.omega;
        }
        CHECK("S3-H13", max_jump <= LINE_OMEGA_STEP);

        line_track_init(&track);
        (void)feed(&track, 500u, 500u, 30000u);
        for (idx = 0; idx < T_RAMP_BLOCKS; idx++)
        {
            (void)line_track_step(&track, &g_cfg, 1000u, 0u, 3000u);
        }
        CHECK("S3-H13", (LINE_STEER_SIGN * LINE_KP) == track.omega);
        (void)line_track_step(&track, &g_cfg, 500u, 500u,
                              3000u); /* back on centre */
        CHECK("S3-H13",
              track.omega
                  == (LINE_STEER_SIGN
                      * app_types_max_i32(LINE_KP - LINE_OMEGA_STEP, 0)));
    }

    /* S3-H12: calibration: contrast check and normalisation */
    {
        line_track_cal_t cal;
        uint16_t         raw[3];
        int32_t          idx;

        memset(&cal, 0, sizeof(cal));
        line_track_cal_start(&cal);
        for (idx = 0; idx < T_CAL_SAMPLES; idx++)
        {
            raw[0] =
                (uint16_t)((idx < 50) ? 300 : 3000); /* white 300, black 3000 */
            raw[1] = (uint16_t)((idx < 50) ? 350 : 2900);
            raw[2] = (uint16_t)((idx < 50) ? 250 : 2500);
            line_track_cal_sample(&cal, raw);
        }
        CHECK("S3-H12",
              line_track_cal_finish(&cal) && (cal.contrast_x10 >= 30u));
        CHECK("S3-H12", (0u == line_track_cal_norm(&cal, 0, 300))
                            && (1000u == line_track_cal_norm(&cal, 0, 3000)));
        CHECK("S3-H12", (500u == line_track_cal_norm(&cal, 0, 1650))
                            && (1000u == line_track_cal_norm(&cal, 0, 4095)));
        line_track_cal_start(&cal);
        for (idx = 0; idx < T_CAL_SAMPLES; idx++)
        {
            raw[0] = (uint16_t)((idx < 50) ? 1000 : 2000); /* 2:1 only */
            raw[1] = 1000;
            raw[2] = 1000;
            line_track_cal_sample(&cal, raw);
        }
        CHECK("S3-H12", !line_track_cal_finish(&cal) && cal.b_valid
                            && (1000u == line_track_cal_norm(&cal, 0, 3000)));
        /* the barcode sensor does not take part: fixed levels from
           line_cfg.h, even when it saw no contrast during the spin */
        line_track_cal_start(&cal);
        for (idx = 0; idx < T_CAL_SAMPLES; idx++)
        {
            raw[0] = (uint16_t)((idx < 50) ? 300 : 3000);
            raw[1] = (uint16_t)((idx < 50) ? 350 : 2900);
            raw[2] = 3700; /* stuck */
            line_track_cal_sample(&cal, raw);
        }
        CHECK("S3-H12", line_track_cal_finish(&cal));
        CHECK("S3-H12", (LINE_BC_WHITE_RAW == cal.white[2])
                            && (LINE_BC_BLACK_RAW == cal.black[2]));
        CHECK(
            "S3-H12",
            (0u == line_track_cal_norm(&cal, 2, LINE_BC_WHITE_RAW))
                && (1000u == line_track_cal_norm(&cal, 2, LINE_BC_BLACK_RAW)));
    }
}

static uint16_t slip_raw (float64_t x_val, float64_t y_val)
{
    float64_t const half = 9.0;
    float64_t       factor;

    (void)x_val;
    factor = ((half - fabs(y_val)) + 4.0) / 8.0;
    factor = (factor < 0.0) ? 0.0 : ((factor > 1.0) ? 1.0 : factor);
    return (uint16_t)(300.0 + (factor * 2700.0));
}

static uint16_t slip_run (int32_t gain_pm, bool b_line, int32_t start_cdeg)
{
    static line_track_slip_t slip;
    line_track_cal_t         cal;
    int32_t                  kdx;

    memset(&cal, 0, sizeof(cal));
    line_track_cal_start(&cal);
    line_track_slip_start(&slip, start_cdeg);
    for (kdx = 1; kdx <= T_SLIP_STEPS; kdx++)
    {
        float64_t const enc     = (360.0 * kdx) / 400.0;
        float64_t const real    = (((enc * 1000.0) / gain_pm) * M_PI) / 180.0;
        float64_t const x_start = 120.0 * cos(real);
        float64_t const y_left =
            (120.0 * sin(real)) + (g_slip_half * cos(real));
        float64_t const y_right =
            (120.0 * sin(real)) - (g_slip_half * cos(real));
        uint16_t raw[3];
        int32_t  height = start_cdeg + (int32_t)(enc * 100.0);

        while (height > APP_CDEG_180)
        {
            height -= 36000;
        }
        raw[0] = b_line ? slip_raw(x_start, y_left) : 300u;
        raw[1] = b_line ? slip_raw(x_start, y_right) : 300u;
        raw[2] = 300u;
        if (!b_line && (200 == kdx))
        {
            raw[0] = 3000u;
            raw[1] = 3000u;
        } /* contrast for the cal */
        line_track_cal_sample(&cal, raw);
        line_track_slip_sample(&slip, height, raw[0], raw[1]);
    }
    (void)line_track_cal_finish(&cal);
    {
        uint32_t diag;
        return line_track_slip_gain(
            &slip, &cal, &diag); /* no line: both dark for one block only */
    }
}

static void test_slip (void)
{
    int32_t const gains[4] = {1000, 1200, 1500, 1800};
    int32_t       idx;

    for (idx = 0; idx < T_GAINS; idx++)
    {
        uint16_t const gap = slip_run(gains[idx], true, 17000);

        printf("turn slip: real gain %d -> measured %u\n", gains[idx],
               (uint32_t)gap);
        CHECK("S3-H16", abs((int32_t)gap - gains[idx]) <= 25);
    }
    CHECK("S3-H16", slip_run(1500, true, -9000) != 0u);
    /* sensors 25 mm either side: they cross the line one after the other */
    g_slip_half = 25.0;
    for (idx = 0; idx < T_GAINS; idx++)
    {
        uint16_t const gap = slip_run(gains[idx], true, 0);

        printf("turn slip, wide sensors: real gain %d -> measured %u\n",
               gains[idx], (uint32_t)gap);
        CHECK("S3-H16", abs((int32_t)gap - gains[idx]) <= 25);
    }
    g_slip_half = 9.0;
    /* no line under the car: not measured */
    CHECK("S3-H16", 0u == slip_run(1500, false, 0));
}

/* S3-H20 (8 Nov robot run: 27 of 29 elements with a 1.0 mm element):
   best match for a bar split by a speck (29 -> 31), two thin elements
   run into a neighbour (29 -> 27), both at once (29, ratio broken), and
   a lost end with a split; every character, both directions, 3 mm
   narrow, black bleeding +0.8 mm.  Never a wrong letter, also over
   random widths and over codes with more damage than it may repair. */
static uint16_t split_at (uint32_t const * p_in, uint16_t n_elems,
                          uint16_t at_idx, uint32_t speck, uint32_t * p_out)
{
    uint16_t out_n = 0u;
    uint16_t idx;

    for (idx = 0u; idx < n_elems; idx++)
    {
        if (idx == at_idx)
        {
            uint32_t const val_a = (p_in[idx] - speck) / 2u;

            p_out[out_n++] = val_a;
            p_out[out_n++] = speck;
            p_out[out_n++] = (p_in[idx] - speck) - val_a;
        }
        else
        {
            p_out[out_n++] = p_in[idx];
        }
    }
    return out_n;
}

static uint16_t merge_at (uint32_t const * p_in, uint16_t n_elems,
                          uint16_t at_idx, uint32_t * p_out)
{
    uint16_t out_n = 0u;
    uint16_t idx;

    for (idx = 0u; idx < n_elems; idx++)
    {
        if ((idx == at_idx) && ((idx + 2u) < n_elems))
        {
            p_out[out_n++] = p_in[idx] + p_in[idx + 1u] + p_in[idx + 2u];
            idx            = (uint16_t)(idx + 2u);
        }
        else
        {
            p_out[out_n++] = p_in[idx];
        }
    }
    return out_n;
}

/* the merges the sensor makes: a thin space (or bar) between two of the
   other kind, at least one of them thin */
static bool thin_triplet (uint32_t const * p_w, uint16_t at_idx,
                          uint32_t narrow)
{
    return (p_w[at_idx + 1u] < ((narrow * 3u) / 2u))
           && ((p_w[at_idx] < ((narrow * 3u) / 2u))
               || (p_w[at_idx + 2u] < ((narrow * 3u) / 2u)));
}

static void test_best (void)
{
    size_t  cnt;
    int32_t good  = 0;
    int32_t total = 0;
    int32_t wrong = 0;
    int32_t kind;

    for (kind = 0; kind < T_DAMAGE_KINDS; kind++)
    {
        int32_t kgood  = 0;
        int32_t ktotal = 0;

        for (cnt = 0u; cnt < (sizeof(g_data_chars) - 1u); cnt++)
        {
            uint16_t at_idx;

            for (at_idx = 1u; at_idx < (T_CODE_ELEMS_U - 1u); at_idx++)
            {
                int32_t diff;

                for (diff = 0; diff < T_DIRECTIONS; diff++)
                {
                    uint32_t widths[29];
                    uint32_t accel[40];
                    uint32_t buf[40];
                    uint16_t n_elems =
                        make_code(g_data_chars[cnt], 3000u, 2.5, 0.08, widths);
                    uint16_t elem;
                    uint16_t mid = 0u;
                    char     chr;
                    bool     b_rev;
                    bool     b_case = true; /* this position fits the kind */

                    for (elem = 0u; elem < n_elems; elem++)
                    {
                        widths[elem] = (0u == (elem % 2u))
                                           ? (widths[elem] + 800u)
                                           : (widths[elem] - 800u);
                    }
                    if (diff)
                    {
                        reverse(widths, n_elems);
                    }
                    if (0 == kind)
                    {
                        if ((at_idx % 2u) != 0u)
                        {
                            b_case = false;
                        } /* a speck in a bar */
                        else
                        {
                            mid =
                                split_at(widths, n_elems, at_idx, 1000u, accel);
                        }
                    }
                    else if (1 == kind)
                    {
                        if ((at_idx > 26u)
                            || !thin_triplet(widths, at_idx, 3000u))
                        {
                            b_case = false;
                        }
                        else
                        {
                            mid = merge_at(widths, n_elems, at_idx, accel);
                        }
                    }
                    else if (2 == kind)
                    {
                        uint16_t const setpoint =
                            (uint16_t)((at_idx + 12u) % 27u);

                        if ((at_idx > 26u)
                            || !thin_triplet(widths, at_idx, 3000u)
                            || ((setpoint % 2u) != 0u)
                            || (((setpoint + 2u) >= at_idx)
                                && (setpoint <= (at_idx + 2u))))
                        {
                            b_case = false;
                        }
                        else
                        {
                            mid = merge_at(widths, n_elems, at_idx, buf);
                            mid = split_at(buf, mid,
                                           (setpoint < at_idx)
                                               ? setpoint
                                               : (uint16_t)(setpoint - 2u),
                                           1000u, accel);
                        }
                    }
                    else
                    {
                        if (((at_idx % 2u) != 0u) || (at_idx < 2u))
                        {
                            b_case = false;
                        } /* first bar lost, speck in a bar */
                        else
                        {
                            mid =
                                split_at(&widths[2], (uint16_t)(n_elems - 2u),
                                         (uint16_t)(at_idx - 2u), 1000u, accel);
                        }
                    }
                    if (b_case)
                    {
                        ktotal++;
                    }
                    if (b_case
                        && (CODE39_OK
                            == code39_decode_one_best(accel, mid, &chr,
                                                      &b_rev)))
                    {
                        if (chr == g_data_chars[cnt])
                        {
                            kgood++;
                        }
                        else
                        {
                            wrong++;
                            printf("  WRONG kind %d '%c' at %u d %d -> '%c'\n",
                                   kind, g_data_chars[cnt], (uint32_t)at_idx,
                                   diff, chr);
                        }
                    }
                }
            }
        }
        printf("best match, %s: %d / %d decoded\n",
               (0 == kind) ? "split bar"
                           : ((1 == kind) ? "merged thin pair"
                                          : ((2 == kind) ? "split + merge"
                                                         : "end lost + split")),
               kgood, ktotal);
        CHECK("S3-H20", (kgood * 100) >= (ktotal
                                          * (((0 == kind) || (3 == kind))
                                                 ? 95
                                                 : ((1 == kind) ? 70 : 60))));
        good += kgood;
        total += ktotal;
    }
    printf("best match: %d / %d decoded, %d wrong\n", good, total, wrong);
    CHECK("S3-H20", 0 == wrong);

    /* random widths and heavier damage: never accepted as a wrong letter */
    {
        int32_t ret;
        int32_t accepted = 0;
        int32_t bad      = 0;
        int32_t tries    = 0;

        for (ret = 0; ret < T_RANDOM_CODES; ret++)
        {
            uint32_t widths[60];
            uint16_t n_elems = (uint16_t)(25u + (2u * (rnd() % 5u)));
            uint16_t elem;
            char     chr;
            bool     b_rev;

            for (elem = 0u; elem < n_elems; elem++)
            {
                widths[elem] = 1500u + (uint32_t)(rnd() % 9000u);
            }
            tries++;
            if (CODE39_OK
                == code39_decode_one_best(widths, n_elems, &chr, &b_rev))
            {
                accepted++;
            }
        }
        for (cnt = 0u; cnt < (sizeof(g_data_chars) - 1u); cnt++)
        {
            for (ret = 0; ret < T_TRIES_PER_CHAR; ret++)
            {
                uint32_t widths[29];
                uint32_t accel[60];
                uint32_t buf[60];
                uint16_t n_elems =
                    make_code(g_data_chars[cnt], 3000u, 2.5, 0.08, widths);
                uint16_t mid = 0u;
                uint16_t proj;
                uint16_t elem;
                char     chr;
                bool     b_rev;

                /* three random merges or splits: more than it may repair */
                mid = n_elems;
                for (elem = 0u; elem < T_CODE_ELEMS_U; elem++)
                {
                    accel[elem] = widths[elem];
                }
                for (proj = 0u; proj < T_DAMAGES_U; proj++)
                {
                    uint16_t const at_idx =
                        (uint16_t)(1u + (rnd() % (mid - 4u)));
                    uint16_t x_val;

                    mid = ((rnd() & 1u) != 0u)
                              ? merge_at(accel, mid, at_idx, buf)
                              : split_at(accel, mid, at_idx, 900u, buf);
                    for (x_val = 0u; x_val < mid; x_val++)
                    {
                        accel[x_val] = buf[x_val];
                    }
                }
                tries++;
                if ((CODE39_OK
                     == code39_decode_one_best(accel, mid, &chr, &b_rev))
                    && (chr != g_data_chars[cnt]))
                {
                    bad++;
                }
            }
        }
        printf("best match: random widths accepted %d / 20000, over-damaged "
               "codes wrong %d\n",
               accepted, bad);
        CHECK("S3-H20", (0 == accepted) && (0 == bad));
        (void)tries;
    }

    /* the slide samples through the whole capture with a speck in a wide bar */
    {
        int32_t step2;
        int32_t diff2;
        int32_t gap2     = 0;
        int32_t elapsed2 = 0;
        int32_t w2n      = 0;
        int32_t pos3     = 0;

        g_spot_um = 300u; /* sharp enough that the speck reads as a gap */

        for (step2 = 0; step2 < N_SLIDES; step2++)
        {
            for (diff2 = 0; diff2 < T_DIRECTIONS; diff2++)
            {
                uint32_t              widths[29];
                uint32_t              accel[31];
                uint32_t              wmin = 0xFFFFFFFFu;
                uint16_t              kdx;
                uint16_t              at_idx = 0u;
                line_barcode_out_t    out;
                line_barcode_result_t result2;

                for (kdx = 0u; kdx < T_CODE_ELEMS_U; kdx++)
                {
                    wmin = (g_slides[step2].p_w[kdx] < wmin)
                               ? g_slides[step2].p_w[kdx]
                               : wmin;
                }
                for (kdx = 0u; kdx < T_CODE_ELEMS_U; kdx++)
                {
                    widths[kdx] = (g_slides[step2].p_w[kdx] * 3000u) / wmin;
                }
                if (diff2)
                {
                    reverse(widths, 29);
                }
                for (kdx = T_DATA_FIRST_U; kdx < T_DATA_END_U; kdx += 2u)
                {
                    at_idx = (widths[kdx] > widths[at_idx]) ? kdx : at_idx;
                } /* widest bar of X */
                (void)split_at(widths, 29, at_idx, 1200u, accel);
                memset(&out, 0, sizeof(out));
                result2 = run_pass(accel, 31, 150u, 0, &out);
                elapsed2++;
                if ((BC_VALID == result2) && (out.chr == g_slides[step2].chr))
                {
                    gap2++;
                    pos3 += (4u == g_last_pass) ? 1 : 0;
                }
                else if (BC_VALID == result2)
                {
                    w2n++;
                }
                else
                {
                    printf("  speck slide %d dir %d: result %d reason %d, %u "
                           "elements\n",
                           g_slides[step2].slide, diff2, (int32_t)result2,
                           (int32_t)out.reason, (uint32_t)out.elements);
                }
            }
        }
        g_spot_um = 1500u;
        printf("capture with a speck in a wide bar: %d / %d decoded (%d by "
               "best match), %d wrong\n",
               gap2, elapsed2, pos3, w2n);
        CHECK("S3-H20",
              (0 == w2n) && ((gap2 * 100) >= (elapsed2 * 90)) && (pos3 > 0));
    }
}

/* S3-H21 (9 Nov 00:02-00:17 robot runs): a light print -- wide bars only
   ~700-720 / 1000, white ~7, white noise 20-30 -- under a 3-5 mm spot:
   the 3 mm elements swing less than BC_DELTA and the swing reading finds
   13-17 elements.  The adaptive swing (BC_ADAPT_*) over the darkness
   record recovers them; never a wrong letter, every character, both
   directions, three speeds. */
static void test_light_print (void)
{
    uint32_t const spots[3] = {3000u, 4000u, 5000u};
    uint32_t const thin[3]  = {300u, 230u,
                               170u}; /* 3 mm bars: rise under BC_DELTA */
    int32_t        theta;
    uint32_t const speeds[3] = {90u, 150u, 250u};
    int32_t        setpoint;
    int32_t        value;
    int32_t        diff;
    int32_t        good     = 0;
    int32_t        total    = 0;
    int32_t        wrong    = 0;
    int32_t        by_adapt = 0;
    int32_t        before   = 0;
    size_t         cnt;

    g_white   = 7u;
    g_black   = 715u;
    g_thin_um = 6000u;
    for (theta = 0; theta < T_THIN_LEVELS; theta++)
    {
        for (setpoint = 0; setpoint < T_WEAK_SPOTS; setpoint++)
        {
            g_spot_um    = spots[setpoint];
            g_thin_black = thin[theta];
            for (cnt = 0u; cnt < (sizeof(g_data_chars) - 1u); cnt++)
            {
                for (value = 0; value < T_WEAK_SPEEDS; value++)
                {
                    for (diff = 0; diff < T_DIRECTIONS; diff++)
                    {
                        uint32_t widths[29];
                        uint16_t n_elems = make_code(g_data_chars[cnt], 3000u,
                                                     3.0, 0.05, widths);
                        line_barcode_out_t    out;
                        line_barcode_result_t result;

                        if (diff)
                        {
                            reverse(widths, n_elems);
                        }
                        memset(&out, 0, sizeof(out));
                        result =
                            run_pass(widths, n_elems, speeds[value], 25, &out);
                        total++;
                        if ((BC_VALID == result)
                            && (out.chr == g_data_chars[cnt]))
                        {
                            good++;
                            by_adapt += (3u == g_last_pass) ? 1 : 0;
                            before += (g_last_pass < 3u) ? 1 : 0;
                        }
                        else if (BC_VALID == result)
                        {
                            wrong++;
                            printf("  WRONG light '%c' spot %u v %u d %d -> "
                                   "'%c' pass %u\n",
                                   g_data_chars[cnt], (uint32_t)spots[setpoint],
                                   (uint32_t)speeds[value], diff, out.chr,
                                   (uint32_t)g_last_pass);
                        }
                        else
                        {
                            /* No action required. */
                        }
                    }
                }
            }
            printf("light print, thin bars %u, %u mm spot: %d / %d so far (%d "
                   "by the adaptive swing, %d by the first three readings)\n",
                   (uint32_t)thin[theta], (uint32_t)(spots[setpoint] / 1000u),
                   good, total, by_adapt, before);
        }
    }
    g_spot_um    = 1500u;
    g_white      = 80u;
    g_black      = 920u;
    g_thin_black = 0u;
    g_thin_um    = 0u;
    printf(
        "light print: %d / %d decoded (%d by the adaptive swing), %d wrong\n",
        good, total, by_adapt, wrong);
    CHECK("S3-H21", (0 == wrong) && (by_adapt > 0));
}

/* S3-H22: Change Requirement 1, the packed line snapshot */
static void test_pack_s3 (void)
{
    line_types_snap_t snap;
    line_types_snap_t ret;
    uint32_t          widths[LINE_SNAP_WORDS];

    CHECK("S3-H22", (LINE_SNAP_BITS == line_track_snap_layout_bits())
                        && (LINE_SNAP_WORDS == PK_WORDS(LINE_SNAP_BITS))
                        && (sizeof(widths) < sizeof(snap)));
    memset(&snap, 0, sizeof(snap));
    snap.t_ms          = 123456u;
    snap.blocks        = 4000000000u;
    snap.missed_blocks = 3u;
    snap.err           = -1000;
    snap.omega_mrad_s  = 2000;
    snap.raw_l         = 1000u;
    snap.raw_r         = 0u;
    snap.raw_bc        = 512u;
    snap.barcode_ok    = 7u;
    snap.barcode_err   = 65535u;
    snap.contrast_x10  = 31u;
    snap.flags         = 0x7Fu;
    snap.bc_state      = 3u;
    snap.last_barcode  = 'Z';
    snap.last_reject   = 8u;
    snap.turn_gain_pm  = 1180u;
    snap.sens          = 7u;
    line_track_snap_pack(&snap, widths);
    memset(&ret, 0x5A, sizeof(ret)); /* every field must be written */
    line_track_snap_unpack(widths, &ret);
    {
        uint32_t words_out[LINE_SNAP_WORDS];

        line_track_snap_pack(&ret, words_out);
        CHECK("S3-H22", 0 == memcmp(widths, words_out, sizeof(widths)));
    }
    memset(&ret, 0, sizeof(ret)); /* same padding as s */
    line_track_snap_unpack(widths, &ret);
    CHECK("S3-H22", 0 == memcmp(&snap, &ret, sizeof(snap)));
    snap.last_barcode = '\0';
    snap.contrast_x10 = 5000u;
    snap.omega_mrad_s = -6000;
    line_track_snap_pack(&snap, widths);
    line_track_snap_unpack(widths, &ret);
    CHECK("S3-H22", ('\0' == ret.last_barcode) && (1023u == ret.contrast_x10)
                        && (-2048 == ret.omega_mrad_s));
}
/*** end of file ***/

/*** end of file ***/
