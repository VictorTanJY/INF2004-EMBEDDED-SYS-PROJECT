/*
 *  test_s3.c
 *  Host unit tests for subsystem 3: Code 39 decoding (code39.c), barcode
 *  capture from a 1 kHz sample stream (line_barcode.c) and line tracking
 *  (line_track.c).  The barcode tests use the element widths of the
 *  instructor's sample sheets (vectors_slides.h): A, F and Z at all three
 *  printed sizes, plus W, X and Y.
 *  Build and run:  make -C testing/subsystem3/host
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "app_pack.h"
#include <math.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include <stdbool.h>
#include "code39.h"
#include "line_barcode.h"
#include "line_track.h"
#include "app_types.h"
#include "vectors_slides.h"

static int g_pass;
static int g_fail;

#define CHECK(id, cond) \
    do { if (cond) { g_pass++; } else { g_fail++; \
        printf("FAIL %s  (%s:%d)  %s\n", id, __FILE__, __LINE__, #cond); } } while (0)

#define N_SLIDES ((int)(sizeof(g_slides) / sizeof(g_slides[0])))

static char const g_data_chars[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ-. $/+%";

static uint32_t g_rng = 12345u;
static uint32_t rnd(void) { g_rng = g_rng * 1103515245u + 12345u; return (g_rng >> 8) & 0xFFFFu; }
/* uniform in [-1, 1] */
static double rndu(void) { return ((double)rnd() / 32767.5) - 1.0; }

/* Widths of *c* with the given narrow width, ratio and +-jitter. */
static uint16_t make_code(char chr, uint32_t narrow, double ratio, double jitter, uint32_t * p_w)
{
    char const seq[3] = { '*', chr, '*' };
    uint16_t n_elems = 0u;
    int kdx;
    int elem;

    for (kdx = 0; kdx < 3; kdx++)
    {
        uint16_t pat = code39_pattern(seq[kdx]);

        for (elem = 0; elem < 9; elem++)
        {
            double omega = (double)narrow * (((pat >> (8 - elem)) & 1u) ? ratio : 1.0);
            p_w[n_elems++] = (uint32_t)(omega * (1.0 + jitter * rndu()));
        }
        if (kdx < 2)
        {
            p_w[n_elems++] = (uint32_t)((double)narrow * (1.0 + jitter * rndu()));   /* gap */
        }
    }
    return n_elems;
}

static void reverse(uint32_t * p_w, uint16_t n_elems)
{
    uint16_t idx;

    for (idx = 0u; idx < n_elems / 2u; idx++)
    {
        uint32_t elapsed = p_w[idx];
        p_w[idx] = p_w[n_elems - 1u - idx];
        p_w[n_elems - 1u - idx] = elapsed;
    }
}

/* ---- sample-stream simulation ---------------------------------------- */
#define TRACK_LEN_UM    1200000u        /* 1.2 m of track per pass */

/* Reflectance (0..1000, 1000 = black) at position x along a track with
   60 mm of white, then the code, then white.  Sensor spot: a box of
   g_spot_um (1.5 mm; S3-H15 widens it to blur the 3 mm elements). */
static uint32_t g_spot_um = 1500u;
static uint32_t g_white   = 80u;
static uint32_t g_black   = 920u;
static uint8_t  g_last_pass = 0u;    /* which reading decoded in the last run_pass */
static uint32_t g_thin_black = 0u;   /* S3-H19: level of bars narrower than g_thin_um (0 = g_black) */
static uint32_t g_thin_um    = 0u;

static uint16_t profile_at(uint32_t const * p_w, uint16_t n_elems, uint32_t x_um)
{
    uint32_t const lead = 60000u;
    uint32_t       spot;
    uint32_t       level = 0u;
    uint32_t       taps  = 0u;

    for (spot = 0u; spot < g_spot_um; spot += 100u)
    {
        taps++;
        uint32_t x_val = x_um + spot;
        uint32_t pos = lead;
        uint16_t elem;
        uint32_t here = g_white;

        for (elem = 0u; elem < n_elems; elem++)
        {
            if ((x_val >= pos) && (x_val < pos + p_w[elem]))
            {
                if ((elem % 2u) == 0u)
                {
                    here = ((g_thin_black != 0u) && (p_w[elem] < g_thin_um)) ? g_thin_black : g_black;
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
static bc_result_t run_pass(uint32_t const * p_w, uint16_t n_elems, uint32_t v_mm_s, int noise, bc_out_t * p_out)
{
    static bc_capture_t capture;
    uint32_t     x_val;
    uint32_t     step = v_mm_s;     /* um per 1 ms sample = mm/s */
    bc_result_t  found = BC_NONE;

    g_last_pass = 0u;
    bc_init(&capture);
    for (x_val = 0u; x_val < TRACK_LEN_UM; x_val += step)
    {
        int32_t  sample = (int32_t)profile_at(p_w, n_elems, x_val) + (int32_t)(noise * rndu());
        bc_out_t out;
        bc_result_t result;

        if (sample < 0) { sample = 0; }
        if (sample > 1000) { sample = 1000; }
        memset(&out, 0, sizeof(out));
        result = bc_sample(&capture, (uint16_t)sample, step, &out);
        if ((result != BC_NONE) && (found == BC_NONE))
        {
            found = result;
            *p_out = out;
            g_last_pass = capture.pass;
        }
        else if (result != BC_NONE)
        {
            return BC_REJECT;           /* more than one result per code */
        }
    }
    return found;
}

static void test_table(void)
{
    size_t idx;
    size_t jdx;
    int    b_ok = 1;

    for (idx = 0u; idx < sizeof(g_data_chars) - 1u; idx++)
    {
        uint16_t pos = code39_pattern(g_data_chars[idx]);
        int wide = 0;
        int wide_sp = 0;
        int val_b;

        for (val_b = 0; val_b < 9; val_b++)
        {
            if ((pos >> (8 - val_b)) & 1u) { wide++; if (val_b % 2) { wide_sp++; } }
        }
        if ((pos == 0u) || (wide != 3) || ((wide_sp != 1) && (wide_sp != 3))) { b_ok = 0; }
        if (code39_char(pos) != g_data_chars[idx]) { b_ok = 0; }
        for (jdx = 0u; jdx < idx; jdx++)
        {
            if (code39_pattern(g_data_chars[jdx]) == pos) { b_ok = 0; }
        }
    }
    CHECK("S3-H01", b_ok);
    CHECK("S3-H01", code39_pattern('*') == 0x094u && code39_char(0x1FFu) == '\0');
}

static void test_slides(void)
{
    int step;

    for (step = 0; step < N_SLIDES; step++)
    {
        uint32_t widths[29];
        char     chr;
        bool     b_rev;
        int      kdx;
        double   scale;

        for (scale = 1.0; scale > 0.5; scale -= (1.0 - 297.0 / 338.67))   /* screen, A4 */
        {
            for (kdx = 0; kdx < 29; kdx++) { widths[kdx] = (uint32_t)(g_slides[step].p_w[kdx] * scale); }
            CHECK("S3-H02", code39_decode_one(widths, 29, 15, &chr, &b_rev) == CODE39_OK &&
                            chr == g_slides[step].chr && !b_rev);
            reverse(widths, 29);
            CHECK("S3-H02", code39_decode_one(widths, 29, 15, &chr, &b_rev) == CODE39_OK &&
                            chr == g_slides[step].chr && b_rev);
            if (scale < 1.0) { break; }
        }
    }
}

static void test_synthetic(void)
{
    uint32_t const narrows[3] = { 3000u, 5100u, 6900u };
    int b_ok = 1;
    int wrong = 0;
    size_t idx;
    int val_a;
    int ret;
    int diff;

    for (idx = 0u; idx < sizeof(g_data_chars) - 1u; idx++)
    {
        for (val_a = 0; val_a < 3; val_a++)
        {
            for (ret = 0; ret < 3; ret++)
            {
                double const ratio = (ret == 0) ? 2.2 : ((ret == 1) ? 2.5 : 3.0);

                for (diff = 0; diff < 2; diff++)
                {
                    uint32_t widths[29];
                    uint16_t n_elems = make_code(g_data_chars[idx], narrows[val_a], ratio, 0.15, widths);
                    char chr;
                    bool b_rev;
                    code39_result_t res;

                    if (diff) { reverse(widths, n_elems); }
                    res = code39_decode_one(widths, n_elems, 15, &chr, &b_rev);
                    if (res != CODE39_OK) { b_ok = 0; }
                    else if (chr != g_data_chars[idx]) { wrong++; }
                }
            }
        }
    }
    CHECK("S3-H03", b_ok && (wrong == 0));
}

static void test_rejects(void)
{
    uint32_t widths[40];
    uint16_t n_elems;
    char chr;
    bool b_rev;
    int idx;
    int wrong = 0;

    /* a wide bar printed narrow: rejected, never a different character */
    for (idx = 0; idx < (int)(sizeof(g_data_chars) - 1u); idx++)
    {
        int elem;
        for (elem = 10; elem < 19; elem++)
        {
            uint16_t pat = code39_pattern(g_data_chars[idx]);

            n_elems = make_code(g_data_chars[idx], 3000u, 3.0, 0.0, widths);
            if ((pat >> (8 - (elem - 10))) & 1u)
            {
                widths[elem] = 3000u;           /* wide -> narrow */
                if (code39_decode_one(widths, n_elems, 15, &chr, &b_rev) == CODE39_OK) { wrong++; }
            }
        }
    }
    CHECK("S3-H04", wrong == 0);
    n_elems = make_code('A', 3000u, 3.0, 0.0, widths);
    CHECK("S3-H04", code39_decode_one(widths, 28, 15, &chr, &b_rev) == CODE39_ERR_COUNT);
    n_elems = make_code('A', 3000u, 1.3, 0.0, widths);
    CHECK("S3-H04", code39_decode_one(widths, n_elems, 15, &chr, &b_rev) == CODE39_ERR_RATIO);
    CHECK("S3-H04", code39_decode_one(widths, 5, 15, &chr, &b_rev) == CODE39_NOT_BARCODE);
    {
        /* "*AB*": valid Code 39 but two data characters */
        uint32_t words_out[40];
        char const seq[4] = { '*', 'A', 'B', '*' };
        int kdx;
        int elem;
        n_elems = 0u;
        for (kdx = 0; kdx < 4; kdx++)
        {
            uint16_t pat = code39_pattern(seq[kdx]);
            for (elem = 0; elem < 9; elem++) { words_out[n_elems++] = ((pat >> (8 - elem)) & 1u) ? 9000u : 3000u; }
            if (kdx < 3) { words_out[n_elems++] = 3000u; }
        }
        CHECK("S3-H04", code39_decode_one(words_out, n_elems, 15, &chr, &b_rev) == CODE39_ERR_LENGTH);
    }
}

static void test_capture(void)
{
    uint32_t const speeds[2] = { 150u, 300u };      /* NFR3 speeds, mm/s */
    int step;
    int value;
    int diff;
    int pass;
    int good = 0;
    int total = 0;
    int wrong = 0;

    /* every slide, both speeds, both directions, clean */
    for (step = 0; step < N_SLIDES; step++)
    {
        for (value = 0; value < 2; value++)
        {
            for (diff = 0; diff < 2; diff++)
            {
                uint32_t widths[29];
                bc_out_t out;
                int kdx;

                for (kdx = 0; kdx < 29; kdx++) { widths[kdx] = g_slides[step].p_w[kdx] * 297u / 339u; }
                if (diff) { reverse(widths, 29); }
                memset(&out, 0, sizeof(out));
                CHECK("S3-H05", run_pass(widths, 29, speeds[value], 0, &out) == BC_VALID && out.chr == g_slides[step].chr);
            }
        }
    }

    /* NFR3-style: 20 noisy passes per character per speed and direction */
    for (step = 0; step < N_SLIDES; step += 3)
    {
        for (value = 0; value < 2; value++)
        {
            for (pass = 0; pass < 20; pass++)
            {
                uint32_t widths[29];
                bc_out_t out;
                bc_result_t result;
                int kdx;

                for (kdx = 0; kdx < 29; kdx++)
                {
                    widths[kdx] = (uint32_t)((double)g_slides[step].p_w[kdx] * 297.0 / 338.67 * (1.0 + 0.10 * rndu()));
                }
                if (pass % 2) { reverse(widths, 29); }
                memset(&out, 0, sizeof(out));
                result = run_pass(widths, 29, speeds[value], 120, &out);
                total++;
                if ((result == BC_VALID) && (out.chr == g_slides[step].chr)) { good++; }
                else if (result == BC_VALID) { wrong++; }
            }
        }
    }
    printf("noisy passes: %d / %d decoded, %d wrong\n", good, total, wrong);
    CHECK("S3-H05", wrong == 0 && (good * 100) >= (total * 95));

    /* S3-H15: blurred sensor spot (2 Nov, the robot saw 3 elements: thin
       bars and thin gaps never crossed the old fixed 350 / 220 levels).  A
       3.5 / 4 mm spot over 3 mm elements (thin bars peak ~670, thin gaps
       ~330 inside wide bars): every slide decodes, both speeds and
       directions, clean and noisy (old levels: 32 / 48 and 0 / 48).  A weak
       sensor (white 250, black 800, 4 mm spot): at least 2 of 3 slides
       decode, never a wrong character (old levels: none, all too long). */
    {
        uint32_t const spots[2] = { 3500u, 4000u };
        int setpoint;
        int blur_good = 0;
        int blur_total = 0;
        int blur_wrong = 0;
        int weak_good = 0;
        int weak_total = 0;
        int weak_wrong = 0;

        for (setpoint = 0; setpoint < 2; setpoint++)
        {
            g_spot_um = spots[setpoint];
            for (step = 0; step < N_SLIDES; step++)
            {
                for (value = 0; value < 2; value++)
                {
                    for (diff = 0; diff < 2; diff++)
                    {
                        uint32_t widths[29];
                        bc_out_t out;
                        bc_result_t result;
                        int kdx;

                        for (kdx = 0; kdx < 29; kdx++) { widths[kdx] = g_slides[step].p_w[kdx] * 297u / 339u; }
                        if (diff) { reverse(widths, 29); }
                        memset(&out, 0, sizeof(out));
                        result = run_pass(widths, 29, speeds[value], (diff ? 60 : 0), &out);
                        blur_total++;
                        if ((result == BC_VALID) && (out.chr == g_slides[step].chr)) { blur_good++; }
                        else if (result == BC_VALID) { blur_wrong++; }
                    }
                }
            }
        }
        g_spot_um = 4000u;  g_white = 250u;  g_black = 800u;
        for (step = 0; step < N_SLIDES; step++)
        {
            uint32_t widths[29];
            bc_out_t out;
            int kdx;

            for (kdx = 0; kdx < 29; kdx++) { widths[kdx] = g_slides[step].p_w[kdx] * 297u / 339u; }
            bc_result_t result;

            memset(&out, 0, sizeof(out));
            result = run_pass(widths, 29, 150u, 0, &out);
            weak_total++;
            if ((result == BC_VALID) && (out.chr == g_slides[step].chr)) { weak_good++; }
            else if (result == BC_VALID) { weak_wrong++; }
        }
        g_spot_um = 1500u;  g_white = 80u;  g_black = 920u;
        printf("blurred passes: %d / %d decoded, %d wrong; weak sensor %d / %d, %d wrong\n",
               blur_good, blur_total, blur_wrong, weak_good, weak_total, weak_wrong);
        CHECK("S3-H15", blur_wrong == 0 && blur_good == blur_total);
        CHECK("S3-H15", weak_wrong == 0 && (weak_good * 3) >= (weak_total * 2));
    }

    /* S3-H18 (7 Nov): (a) a read one bar short at either end (2 elements
       lost) still decodes, both directions, never a wrong letter; (b) black
       bleeding into white (bars +40 % of a narrow, spaces -40 %) decodes
       with bars and spaces ranked apart; (c) a 5-6 mm spot over 3 mm
       elements: the profile's second try decodes more than the swing
       reading alone, never a wrong letter. */
    {
        int good = 0;
        int total = 0;
        int wrong = 0;
        int kdx;

        for (step = 0; step < N_SLIDES; step++)
        {
            for (diff = 0; diff < 2; diff++)
            {
                int cut;

                for (cut = 0; cut < 2; cut++)
                {
                    uint32_t widths[29];
                    char chr;
                    bool b_rev;

                    for (kdx = 0; kdx < 29; kdx++) { widths[kdx] = g_slides[step].p_w[kdx]; }
                    if (diff) { reverse(widths, 29); }
                    total++;
                    if (code39_decode_one_repair(cut ? widths : &widths[2], 27, 15, &chr, &b_rev) == CODE39_OK)
                    {
                        if (chr == g_slides[step].chr) { good++; } else { wrong++; }
                    }
                }
            }
        }
        printf("one bar short at an end: %d / %d decoded, %d wrong\n", good, total, wrong);
        CHECK("S3-H18", (wrong == 0) && (good == total));

        good = 0; total = 0; wrong = 0;
        {
            size_t idx;

            for (idx = 0u; idx < 26u; idx++)
            {
                uint32_t widths[29];
                uint16_t n_elems = make_code((char)('A' + idx), 3000u, 2.5, 0.0, widths);
                char chr;
                bool b_rev;
                uint16_t elem;

                for (elem = 0u; elem < n_elems; elem++)
                {
                    widths[elem] = ((elem % 2u) == 0u) ? (widths[elem] + 1200u) : ((widths[elem] > 1200u) ? (widths[elem] - 1200u) : 100u);
                }
                total++;
                if (code39_decode_one(widths, n_elems, 15, &chr, &b_rev) == CODE39_OK)
                {
                    if (chr == (char)('A' + idx)) { good++; } else { wrong++; }
                }
            }
        }
        printf("black bleeding 40 %%: %d / %d decoded, %d wrong\n", good, total, wrong);
        CHECK("S3-H18", (wrong == 0) && (good == total));

        {
            uint32_t const spots[2] = { 5000u, 6000u };
            int setpoint;
            int pass2 = 0;

            good = 0; total = 0; wrong = 0;
            for (setpoint = 0; setpoint < 2; setpoint++)
            {
                g_spot_um = spots[setpoint];
                for (step = 0; step < N_SLIDES; step++)
                {
                    for (diff = 0; diff < 2; diff++)
                    {
                        uint32_t widths[29];
                        bc_out_t out;
                        bc_result_t result;

                        for (kdx = 0; kdx < 29; kdx++) { widths[kdx] = g_slides[step].p_w[kdx] * 297u / 339u; }
                        if (diff) { reverse(widths, 29); }
                        memset(&out, 0, sizeof(out));
                        result = run_pass(widths, 29, 90u, 0, &out);
                        total++;
                        if ((result == BC_VALID) && (out.chr == g_slides[step].chr)) { good++; }
                        else if (result == BC_VALID) { wrong++; }
                        pass2 += (g_last_pass != 0u) ? 1 : 0;
                    }
                }
            }
            g_spot_um = 1500u;
            printf("5-6 mm spot: %d / %d decoded (%d by the second try), %d wrong\n", good, total, pass2, wrong);
            CHECK("S3-H18", wrong == 0);
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
        int step;
        int diff;
        int good = 0;
        int total = 0;
        int wrong = 0;
        int noise;

        g_spot_um = 1500u;  g_white = 20u;  g_black = 690u;  g_thin_black = 240u;
        for (noise = 0; noise <= 40; noise += 40)
        {
            for (step = 0; step < N_SLIDES; step++)
            {
                if (g_slides[step].chr != 'Z') { continue; }
                for (diff = 0; diff < 2; diff++)
                {
                    uint32_t widths[29];
                    bc_out_t out;
                    bc_result_t result;
                    int kdx;

                    uint32_t wmin = 0xFFFFFFFFu;

                    /* printed at the specified size: 3 mm narrow (~141 mm code) */
                    for (kdx = 0; kdx < 29; kdx++) { wmin = (g_slides[step].p_w[kdx] < wmin) ? g_slides[step].p_w[kdx] : wmin; }
                    for (kdx = 0; kdx < 29; kdx++) { widths[kdx] = (g_slides[step].p_w[kdx] * 3000u) / wmin; }
                    g_thin_um = 6000u;          /* bars under 6 mm are the faint narrow ones */
                    if (diff) { reverse(widths, 29); }
                    memset(&out, 0, sizeof(out));
                    result = run_pass(widths, 29, 90u, noise, &out);
                    total++;
                    if ((result == BC_VALID) && (out.chr == 'Z')) { good++; }
                    else if (result == BC_VALID) { wrong++; }
                    else { printf("  Z slide %d dir %d noise %d: result %d, %u elements, %u mm\n", g_slides[step].slide, diff,
                                  noise, (int)result, (unsigned)out.elements, (unsigned)(out.length_um / 1000u)); }
                }
            }
        }
        g_spot_um = 1500u;  g_white = 80u;  g_black = 920u;  g_thin_black = 0u;
        printf("faint 3 mm bars (robot 7 Nov): Z %d / %d decoded, %d wrong\n", good, total, wrong);
        CHECK("S3-H19", (wrong == 0) && (good == total));
    }

    /* S3-H20 (14 Nov car runs, 1 mm slit on the sensor, 3 mm "*A*"): two
       reads with a 1.4 mm speck outside the code, one after the stop
       (31 elements) and one before the start with a wide bar split by a
       1 mm white speck (33).  Both were rejected for their count, and the
       best match that ran on four readings took over 50 ms and tripped
       the line task's heartbeat.  Widths as logged (LOG 75, 0.2 mm). */
    {
        static uint32_t const k_r1[31] = {
            2400u, 9000u, 3400u, 3400u, 8000u, 4000u, 9000u, 3600u, 2600u, 3400u, 8000u, 4600u, 2600u,
            4200u, 2000u, 10200u, 2000u, 4200u, 8000u, 4200u, 2400u, 9600u, 2400u, 3400u, 7600u,
            4200u, 8000u, 4200u, 2600u, 8200u, 1400u };
        static uint32_t const k_r2[33] = {
            1400u, 2600u, 3000u, 8600u, 3400u, 2400u, 10000u, 2000u, 2400u, 1000u, 9600u, 2400u, 2600u,
            2000u, 10000u, 3000u, 2400u, 3400u, 3600u, 9600u, 2400u, 3600u, 8200u, 3600u, 2400u,
            9600u, 3000u, 3400u, 8600u, 3400u, 9000u, 3400u, 3400u };
        int diff;
        int noise;
        int good = 0;
        int total = 0;
        int wrong = 0;

        g_spot_um = 1000u;  g_white = 25u;  g_black = 1000u;
        for (noise = 0; noise <= 30; noise += 30)
        {
            for (diff = 0; diff < 4; diff++)
            {
                uint32_t       widths[33];
                uint16_t const n_elems = (diff < 2) ? 31u : 33u;
                bc_out_t       out;
                bc_result_t    result;

                memcpy(widths, (diff < 2) ? k_r1 : k_r2, n_elems * sizeof(uint32_t));
                if ((diff % 2) != 0) { reverse(widths, n_elems); }
                memset(&out, 0, sizeof(out));
                result = run_pass(widths, n_elems, 90u, noise, &out);
                total++;
                if ((result == BC_VALID) && (out.chr == 'A') && (out.b_reversed == ((diff % 2) != 0))) { good++; }
                else if (result == BC_VALID) { wrong++; }
                else { printf("  14 Nov read %d dir %d noise %d: result %d reason %u, %u elements\n", diff / 2 + 1, diff % 2,
                              noise, (int)result, (unsigned)out.reason, (unsigned)out.elements); }
            }
        }
        g_spot_um = 1500u;  g_white = 80u;  g_black = 920u;
        printf("end specks (car 14 Nov): A %d / %d decoded, %d wrong\n", good, total, wrong);
        CHECK("S3-H20", (wrong == 0) && (good == total));
    }

    /* S3-H06: a crossing line (18 mm black) is not a barcode and not an error */
    {
        uint32_t widths[1] = { 18000u };
        bc_out_t out;
        CHECK("S3-H06", run_pass(widths, 1, 300u, 0, &out) == BC_NONE);
    }
    /* S3-H07: a long dark patch is rejected, never decoded */
    {
        uint32_t widths[1] = { 500000u };
        bc_out_t out;
        memset(&out, 0, sizeof(out));
        CHECK("S3-H07", run_pass(widths, 1, 300u, 0, &out) == BC_REJECT && out.reason == CODE39_ERR_TOO_LONG);
    }
}

/* ---- line tracking ---------------------------------------------------- */
static line_trk_cfg_t const g_cfg = { 20000u, 80000u, 300 };

/* Feed blocks of 3 mm (0.3 m/s x 10 ms) for dist_um; returns OR of events. */
static uint8_t feed(line_track_t * p_t, uint16_t left, uint16_t ret, uint32_t dist_um)
{
    uint8_t events = 0u;
    uint32_t diff;

    for (diff = 0u; diff < dist_um; diff += 3000u) { events |= line_track_step(p_t, &g_cfg, left, ret, 3000u); }
    return events;
}

static void test_track(void)
{
    line_track_t track;
    uint8_t events;

    /* S3-H10: steering sign: line under the left sensor -> turn left */
    line_track_init(&track);
    (void)feed(&track, 500u, 500u, 60000u);
    CHECK("S3-H10", track.err == 0 && track.omega == 0 && track.v_mm_s == 300);
    (void)line_track_step(&track, &g_cfg, 800u, 200u, 3000u);
    CHECK("S3-H10", track.err == 600 && track.omega > 0 && track.v_mm_s < 300);
    (void)feed(&track, 200u, 800u, 9000u);
    CHECK("S3-H10", track.omega < 0);

    /* S3-H08: junction on a straight approach, once; not on a tight curve */
    line_track_init(&track);
    (void)feed(&track, 500u, 500u, 60000u);
    events = feed(&track, 1000u, 1000u, 15000u);
    CHECK("S3-H08", (events & LINE_TRK_EV_JUNCTION) == 0u);
    events = feed(&track, 1000u, 1000u, 15000u);
    CHECK("S3-H08", (events & LINE_TRK_EV_JUNCTION) != 0u && track.omega == 0);
    events = feed(&track, 1000u, 1000u, 30000u);
    CHECK("S3-H08", (events & LINE_TRK_EV_JUNCTION) == 0u);
    line_track_init(&track);
    (void)feed(&track, 950u, 300u, 30000u);        /* hard steering on a curve */
    events = feed(&track, 1000u, 1000u, 30000u);
    CHECK("S3-H08", (events & LINE_TRK_EV_JUNCTION) == 0u);

    /* S3-H09: 50 mm gap bridged (NFR11, heading held), 90 mm -> LOST once, then FOUND */
    line_track_init(&track);
    (void)feed(&track, 500u, 500u, 60000u);
    events = feed(&track, 50u, 50u, 50000u);
    CHECK("S3-H09", (events & LINE_TRK_EV_LOST) == 0u && track.omega == 0 && track.v_mm_s == 300);
    (void)feed(&track, 500u, 500u, 30000u);
    events = feed(&track, 50u, 50u, 90000u);
    CHECK("S3-H09", (events & LINE_TRK_EV_LOST) != 0u && track.b_lost && track.v_mm_s == 0);
    events = feed(&track, 50u, 50u, 90000u);
    CHECK("S3-H09", (events & LINE_TRK_EV_LOST) == 0u);
    events = feed(&track, 600u, 400u, 3000u);
    CHECK("S3-H09", (events & LINE_TRK_EV_FOUND) != 0u && !track.b_lost);

    /* S3-H11: drifting off the right of the line -> keeps turning back left */
    line_track_init(&track);
    (void)feed(&track, 500u, 500u, 60000u);
    (void)feed(&track, 950u, 80u, 6000u);          /* line under the left sensor only */
    (void)feed(&track, 60u, 60u, 9000u);           /* ... then past it */
    CHECK("S3-H11", track.omega > 0 && track.v_mm_s < 300);

    /* S3-H14: right curve, the line slides out past the right sensor and
       the error FADES (-1000 -> -300) before both read white: still a turn
       back right (was: taken for a gap, straight on); an old strong error
       (more than LINE_EXIT_PEAK_UM ago) does not count, a gap stays a gap */
    line_track_init(&track);
    (void)feed(&track, 500u, 500u, 60000u);
    (void)feed(&track, 0u, 1000u, 6000u);          /* line under the right sensor */
    (void)feed(&track, 0u, 300u, 6000u);           /* sliding out: error fades */
    CHECK("S3-H14", track.err == -300);
    (void)line_track_step(&track, &g_cfg, 60u, 60u, 3000u);   /* both white */
    CHECK("S3-H14", track.exit_err == -1000 && track.omega < 0 && track.v_mm_s < 300);
    line_track_init(&track);
    (void)feed(&track, 500u, 500u, 60000u);
    (void)feed(&track, 0u, 1000u, 6000u);
    (void)feed(&track, 500u, 500u, 45000u);        /* centred again for 45 mm */
    (void)feed(&track, 60u, 60u, 9000u);           /* a gap */
    CHECK("S3-H14", track.omega == 0 && track.v_mm_s == 300);

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
        for (idx = 0; idx < 40; idx++)
        {
            if ((idx & 1) == 0) { (void)line_track_step(&track, &g_cfg, 1000u, 0u, 3000u); }
            else              { (void)line_track_step(&track, &g_cfg, 0u, 1000u, 3000u); }
            max_jump = (app_abs_i32(track.omega - prev) > max_jump) ? app_abs_i32(track.omega - prev) : max_jump;
            prev = track.omega;
        }
        CHECK("S3-H13", max_jump <= LINE_OMEGA_STEP);

        line_track_init(&track);
        (void)feed(&track, 500u, 500u, 30000u);
        for (idx = 0; idx < 17; idx++) { (void)line_track_step(&track, &g_cfg, 1000u, 0u, 3000u); }
        CHECK("S3-H13", track.omega == LINE_STEER_SIGN * LINE_KP);
        (void)line_track_step(&track, &g_cfg, 500u, 500u, 3000u);   /* back on centre */
        CHECK("S3-H13", track.omega == LINE_STEER_SIGN * app_max_i32(LINE_KP - LINE_OMEGA_STEP, 0));
    }

    /* S3-H12: calibration: contrast check and normalisation */
    {
        line_cal_t cal;
        uint16_t raw[3];
        int idx;

        memset(&cal, 0, sizeof(cal));
        line_cal_start(&cal);
        for (idx = 0; idx < 100; idx++)
        {
            raw[0] = (uint16_t)((idx < 50) ? 300 : 3000);  /* white 300, black 3000 */
            raw[1] = (uint16_t)((idx < 50) ? 350 : 2900);
            raw[2] = (uint16_t)((idx < 50) ? 250 : 2500);
            line_cal_sample(&cal, raw);
        }
        CHECK("S3-H12", line_cal_finish(&cal) && cal.contrast_x10 >= 30);
        CHECK("S3-H12", line_cal_norm(&cal, 0, 300) == 0 && line_cal_norm(&cal, 0, 3000) == 1000);
        CHECK("S3-H12", line_cal_norm(&cal, 0, 1650) == 500 && line_cal_norm(&cal, 0, 4095) == 1000);
        line_cal_start(&cal);
        for (idx = 0; idx < 100; idx++)
        {
            raw[0] = (uint16_t)((idx < 50) ? 1000 : 2000);  /* 2:1 only */
            raw[1] = 1000; raw[2] = 1000;
            line_cal_sample(&cal, raw);
        }
        CHECK("S3-H12", !line_cal_finish(&cal) && cal.b_valid && line_cal_norm(&cal, 0, 3000) == 1000);
        /* the barcode sensor does not take part: fixed levels from
           line_cfg.h, even when it saw no contrast during the spin */
        line_cal_start(&cal);
        for (idx = 0; idx < 100; idx++)
        {
            raw[0] = (uint16_t)((idx < 50) ? 300 : 3000);
            raw[1] = (uint16_t)((idx < 50) ? 350 : 2900);
            raw[2] = 3700;                                  /* stuck */
            line_cal_sample(&cal, raw);
        }
        CHECK("S3-H12", line_cal_finish(&cal));
        CHECK("S3-H12", cal.white[2] == LINE_BC_WHITE_RAW && cal.black[2] == LINE_BC_BLACK_RAW);
        CHECK("S3-H12", line_cal_norm(&cal, 2, LINE_BC_WHITE_RAW) == 0 &&
                        line_cal_norm(&cal, 2, LINE_BC_BLACK_RAW) == 1000);
    }
}

/* S3-H16 (3 Nov): turn slip from the calibration spin.  Model: the car
   spins about its axle on a straight 18 mm line through the axle; the
   line sensors are OBST_IR_AHEAD 120 mm ahead, 9 mm either side (both
   half on the line at the start); the encoders count `gain` x the real
   rotation; a sensor reads the fraction of its 8 mm spot on black. */
static double g_slip_half = 9.0;    /* sensor offset from the centreline, mm */

static uint16_t slip_raw(double x_val, double y_val)
{
    double const half = 9.0;
    double       factor;

    (void)x_val;
    factor = (half - fabs(y_val) + 4.0) / 8.0;
    factor = (factor < 0.0) ? 0.0 : ((factor > 1.0) ? 1.0 : factor);
    return (uint16_t)(300.0 + (factor * 2700.0));
}

static uint16_t slip_run(int gain_pm, bool b_line, int start_cdeg)
{
    static line_slip_t slip;
    line_cal_t         cal;
    int                kdx;

    memset(&cal, 0, sizeof(cal));
    line_cal_start(&cal);
    line_slip_start(&slip, start_cdeg);
    for (kdx = 1; kdx <= 400; kdx++)
    {
        double const enc  = 360.0 * kdx / 400.0;
        double const real = enc * 1000.0 / gain_pm * M_PI / 180.0;
        double const x_start   = 120.0 * cos(real);
        double const y_left   = (120.0 * sin(real)) + (g_slip_half * cos(real));
        double const y_right   = (120.0 * sin(real)) - (g_slip_half * cos(real));
        uint16_t     raw[3];
        int          height    = start_cdeg + (int)(enc * 100.0);

        while (height > 18000) { height -= 36000; }
        raw[0] = b_line ? slip_raw(x_start, y_left) : 300u;
        raw[1] = b_line ? slip_raw(x_start, y_right) : 300u;
        raw[2] = 300u;
        if (!b_line && (kdx == 200)) { raw[0] = 3000u; raw[1] = 3000u; }   /* contrast for the cal */
        line_cal_sample(&cal, raw);
        line_slip_sample(&slip, height, raw[0], raw[1]);
    }
    (void)line_cal_finish(&cal);
    {
        uint32_t diag;
        return line_slip_gain(&slip, &cal, &diag);  /* no line: both dark for one block only */
    }
}

static void test_slip(void)
{
    int const gains[4] = { 1000, 1200, 1500, 1800 };
    int       idx;

    for (idx = 0; idx < 4; idx++)
    {
        uint16_t const gap = slip_run(gains[idx], true, 17000);

        printf("turn slip: real gain %d -> measured %u\n", gains[idx], (unsigned)gap);
        CHECK("S3-H16", abs((int)gap - gains[idx]) <= 25);
    }
    CHECK("S3-H16", slip_run(1500, true, -9000) != 0u);
    /* sensors 25 mm either side: they cross the line one after the other */
    g_slip_half = 25.0;
    for (idx = 0; idx < 4; idx++)
    {
        uint16_t const gap = slip_run(gains[idx], true, 0);

        printf("turn slip, wide sensors: real gain %d -> measured %u\n", gains[idx], (unsigned)gap);
        CHECK("S3-H16", abs((int)gap - gains[idx]) <= 25);
    }
    g_slip_half = 9.0;
    /* no line under the car: not measured */
    CHECK("S3-H16", slip_run(1500, false, 0) == 0u);
}


/* S3-H20 (8 Nov robot run: 27 of 29 elements with a 1.0 mm element):
   best match for a bar split by a speck (29 -> 31), two thin elements
   run into a neighbour (29 -> 27), both at once (29, ratio broken), and
   a lost end with a split; every character, both directions, 3 mm
   narrow, black bleeding +0.8 mm.  Never a wrong letter, also over
   random widths and over codes with more damage than it may repair. */
static uint16_t split_at(uint32_t const * p_in, uint16_t n_elems, uint16_t at_idx, uint32_t speck, uint32_t * p_out)
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
            p_out[out_n++] = p_in[idx] - speck - val_a;
        }
        else
        {
            p_out[out_n++] = p_in[idx];
        }
    }
    return out_n;
}

static uint16_t merge_at(uint32_t const * p_in, uint16_t n_elems, uint16_t at_idx, uint32_t * p_out)
{
    uint16_t out_n = 0u;
    uint16_t idx;

    for (idx = 0u; idx < n_elems; idx++)
    {
        if ((idx == at_idx) && ((idx + 2u) < n_elems))
        {
            p_out[out_n++] = p_in[idx] + p_in[idx + 1u] + p_in[idx + 2u];
            idx = (uint16_t)(idx + 2u);
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
static bool thin_triplet(uint32_t const * p_w, uint16_t at_idx, uint32_t narrow)
{
    return (p_w[at_idx + 1u] < (narrow * 3u / 2u)) &&
           ((p_w[at_idx] < (narrow * 3u / 2u)) || (p_w[at_idx + 2u] < (narrow * 3u / 2u)));
}

static void test_best(void)
{
    size_t cnt;
    int good = 0;
    int total = 0;
    int wrong = 0;
    int kind;

    for (kind = 0; kind < 4; kind++)
    {
        int kgood = 0;
        int ktotal = 0;

        for (cnt = 0u; cnt < (sizeof(g_data_chars) - 1u); cnt++)
        {
            uint16_t at_idx;

            for (at_idx = 1u; at_idx < 28u; at_idx++)
            {
                int diff;

                for (diff = 0; diff < 2; diff++)
                {
                    uint32_t widths[29];
                    uint32_t accel[40];
                    uint32_t buf[40];
                    uint16_t n_elems = make_code(g_data_chars[cnt], 3000u, 2.5, 0.08, widths), elem, mid;
                    char     chr;
                    bool     b_rev;

                    for (elem = 0u; elem < n_elems; elem++)
                    {
                        widths[elem] = ((elem % 2u) == 0u) ? (widths[elem] + 800u) : (widths[elem] - 800u);
                    }
                    if (diff) { reverse(widths, n_elems); }
                    if (kind == 0)
                    {
                        if ((at_idx % 2u) != 0u) { continue; }     /* a speck in a bar */
                        mid = split_at(widths, n_elems, at_idx, 1000u, accel);
                    }
                    else if (kind == 1)
                    {
                        if ((at_idx > 26u) || !thin_triplet(widths, at_idx, 3000u)) { continue; }
                        mid = merge_at(widths, n_elems, at_idx, accel);
                    }
                    else if (kind == 2)
                    {
                        uint16_t const setpoint = (uint16_t)((at_idx + 12u) % 27u);

                        if ((at_idx > 26u) || !thin_triplet(widths, at_idx, 3000u) || ((setpoint % 2u) != 0u) ||
                            ((setpoint + 2u) >= at_idx && setpoint <= (at_idx + 2u))) { continue; }
                        mid = merge_at(widths, n_elems, at_idx, buf);
                        mid = split_at(buf, mid, (setpoint < at_idx) ? setpoint : (uint16_t)(setpoint - 2u), 1000u, accel);
                    }
                    else
                    {
                        if (((at_idx % 2u) != 0u) || (at_idx < 2u)) { continue; }   /* first bar lost, speck in a bar */
                        mid = split_at(&widths[2], (uint16_t)(n_elems - 2u), (uint16_t)(at_idx - 2u), 1000u, accel);
                    }
                    ktotal++;
                    if (code39_decode_one_best(accel, mid, &chr, &b_rev) == CODE39_OK)
                    {
                        if (chr == g_data_chars[cnt]) { kgood++; } else { wrong++;
                            printf("  WRONG kind %d '%c' at %u d %d -> '%c'\n", kind, g_data_chars[cnt], (unsigned)at_idx, diff, chr); }
                    }
                }
            }
        }
        printf("best match, %s: %d / %d decoded\n",
               (kind == 0) ? "split bar" : (kind == 1) ? "merged thin pair" : (kind == 2) ? "split + merge" : "end lost + split",
               kgood, ktotal);
        CHECK("S3-H20", (kgood * 100) >= (ktotal * ((kind == 0) || (kind == 3) ? 95 : ((kind == 1) ? 70 : 60))));
        good += kgood; total += ktotal;
    }
    printf("best match: %d / %d decoded, %d wrong\n", good, total, wrong);
    CHECK("S3-H20", wrong == 0);

    /* random widths and heavier damage: never accepted as a wrong letter */
    {
        int ret;
        int accepted = 0;
        int bad = 0;
        int tries = 0;

        for (ret = 0; ret < 20000; ret++)
        {
            uint32_t widths[60];
            uint16_t n_elems = (uint16_t)(25u + 2u * (rnd() % 5u)), elem;
            char     chr;
            bool     b_rev;

            for (elem = 0u; elem < n_elems; elem++) { widths[elem] = 1500u + (uint32_t)(rnd() % 9000u); }
            tries++;
            if (code39_decode_one_best(widths, n_elems, &chr, &b_rev) == CODE39_OK) { accepted++; }
        }
        for (cnt = 0u; cnt < (sizeof(g_data_chars) - 1u); cnt++)
        {
            for (ret = 0; ret < 40; ret++)
            {
                uint32_t widths[29];
                uint32_t accel[60];
                uint32_t buf[60];
                uint16_t n_elems = make_code(g_data_chars[cnt], 3000u, 2.5, 0.08, widths), mid, proj, elem;
                char     chr;
                bool     b_rev;

                /* three random merges or splits: more than it may repair */
                mid = n_elems;
                for (elem = 0u; elem < 29u; elem++) { accel[elem] = widths[elem]; }
                for (proj = 0u; proj < 3u; proj++)
                {
                    uint16_t const at_idx = (uint16_t)(1u + (rnd() % (mid - 4u)));
                    uint16_t       x_val;

                    mid = ((rnd() & 1u) != 0u) ? merge_at(accel, mid, at_idx, buf) : split_at(accel, mid, at_idx, 900u, buf);
                    for (x_val = 0u; x_val < mid; x_val++) { accel[x_val] = buf[x_val]; }
                }
                tries++;
                if ((code39_decode_one_best(accel, mid, &chr, &b_rev) == CODE39_OK) && (chr != g_data_chars[cnt])) { bad++; }
            }
        }
        printf("best match: random widths accepted %d / 20000, over-damaged codes wrong %d\n", accepted, bad);
        CHECK("S3-H20", (accepted == 0) && (bad == 0));
        (void)tries;
    }

    /* the slide samples through the whole capture with a speck in a wide bar */
    {
        int step2;
        int diff2;
        int gap2 = 0;
        int elapsed2 = 0;
        int w2n = 0;
        int pos3 = 0;

        g_spot_um = 300u;               /* sharp enough that the speck reads as a gap */

        for (step2 = 0; step2 < N_SLIDES; step2++)
        {
            for (diff2 = 0; diff2 < 2; diff2++)
            {
                uint32_t widths[29];
                uint32_t accel[31];
                uint32_t wmin = 0xFFFFFFFFu;
                uint16_t kdx;
                uint16_t at_idx = 0u;
                bc_out_t out;
                bc_result_t result2;

                for (kdx = 0u; kdx < 29u; kdx++) { wmin = (g_slides[step2].p_w[kdx] < wmin) ? g_slides[step2].p_w[kdx] : wmin; }
                for (kdx = 0u; kdx < 29u; kdx++) { widths[kdx] = (g_slides[step2].p_w[kdx] * 3000u) / wmin; }
                if (diff2) { reverse(widths, 29); }
                for (kdx = 10u; kdx < 19u; kdx += 2u) { at_idx = (widths[kdx] > widths[at_idx]) ? kdx : at_idx; }   /* widest bar of X */
                (void)split_at(widths, 29, at_idx, 1200u, accel);
                memset(&out, 0, sizeof(out));
                result2 = run_pass(accel, 31, 150u, 0, &out);
                elapsed2++;
                if ((result2 == BC_VALID) && (out.chr == g_slides[step2].chr)) { gap2++; pos3 += (g_last_pass == 4u) ? 1 : 0; }
                else if (result2 == BC_VALID) { w2n++; }
                else { printf("  speck slide %d dir %d: result %d reason %d, %u elements\n", g_slides[step2].slide, diff2,
                              (int)result2, (int)out.reason, (unsigned)out.elements); }
            }
        }
        g_spot_um = 1500u;
        printf("capture with a speck in a wide bar: %d / %d decoded (%d by best match), %d wrong\n", gap2, elapsed2, pos3, w2n);
        CHECK("S3-H20", (w2n == 0) && ((gap2 * 100) >= (elapsed2 * 90)) && (pos3 > 0));
    }
}

/* S3-H21 (9 Nov 00:02-00:17 robot runs): a light print -- wide bars only
   ~700-720 / 1000, white ~7, white noise 20-30 -- under a 3-5 mm spot:
   the 3 mm elements swing less than BC_DELTA and the swing reading finds
   13-17 elements.  The adaptive swing (BC_ADAPT_*) over the darkness
   record recovers them; never a wrong letter, every character, both
   directions, three speeds. */
static void test_light_print(void)
{
    uint32_t const spots[3]  = { 3000u, 4000u, 5000u };
    uint32_t const thin[3]   = { 300u, 230u, 170u };   /* 3 mm bars: rise under BC_DELTA */
    int theta;
    uint32_t const speeds[3] = { 90u, 150u, 250u };
    int setpoint;
    int value;
    int diff;
    int good = 0;
    int total = 0;
    int wrong = 0;
    int by_adapt = 0;
    int before = 0;
    size_t cnt;

    g_white = 7u;  g_black = 715u;  g_thin_um = 6000u;
    for (theta = 0; theta < 3; theta++)
    for (setpoint = 0; setpoint < 3; setpoint++)
    {
        g_spot_um = spots[setpoint];  g_thin_black = thin[theta];
        for (cnt = 0u; cnt < (sizeof(g_data_chars) - 1u); cnt++)
        {
            for (value = 0; value < 3; value++)
            {
                for (diff = 0; diff < 2; diff++)
                {
                    uint32_t widths[29];
                    uint16_t n_elems = make_code(g_data_chars[cnt], 3000u, 3.0, 0.05, widths);
                    bc_out_t out;
                    bc_result_t result;

                    if (diff) { reverse(widths, n_elems); }
                    memset(&out, 0, sizeof(out));
                    result = run_pass(widths, n_elems, speeds[value], 25, &out);
                    total++;
                    if ((result == BC_VALID) && (out.chr == g_data_chars[cnt]))
                    {
                        good++;
                        by_adapt += (g_last_pass == 3u) ? 1 : 0;
                        before   += (g_last_pass < 3u) ? 1 : 0;
                    }
                    else if (result == BC_VALID)
                    {
                        wrong++;
                        printf("  WRONG light '%c' spot %u v %u d %d -> '%c' pass %u\n", g_data_chars[cnt],
                               (unsigned)spots[setpoint], (unsigned)speeds[value], diff, out.chr, (unsigned)g_last_pass);
                    }
                }
            }
        }
        printf("light print, thin bars %u, %u mm spot: %d / %d so far (%d by the adaptive swing, %d by the first three readings)\n",
               (unsigned)thin[theta], (unsigned)(spots[setpoint] / 1000u), good, total, by_adapt, before);
    }
    g_spot_um = 1500u;  g_white = 80u;  g_black = 920u;  g_thin_black = 0u;  g_thin_um = 0u;
    printf("light print: %d / %d decoded (%d by the adaptive swing), %d wrong\n", good, total, by_adapt, wrong);
    CHECK("S3-H21", (wrong == 0) && (by_adapt > 0));
}

/* S3-H22: Change Requirement 1, the packed line snapshot */
static void test_pack_s3(void)
{
    line_snap_t snap;
    line_snap_t ret;
    uint32_t    widths[LINE_SNAP_WORDS];

    CHECK("S3-H22", line_snap_layout_bits() == LINE_SNAP_BITS && PK_WORDS(LINE_SNAP_BITS) == LINE_SNAP_WORDS &&
                    sizeof(widths) < sizeof(snap));
    memset(&snap, 0, sizeof(snap));
    snap.t_ms = 123456u; snap.blocks = 4000000000u; snap.missed_blocks = 3u; snap.err = -1000; snap.omega_mrad_s = 2000;
    snap.raw_l = 1000u; snap.raw_r = 0u; snap.raw_bc = 512u; snap.barcode_ok = 7u; snap.barcode_err = 65535u;
    snap.contrast_x10 = 31u; snap.flags = 0x7Fu; snap.bc_state = 3u; snap.last_barcode = 'Z'; snap.last_reject = 8u;
    snap.turn_gain_pm = 1180u; snap.sens = 7u;
    line_snap_pack(&snap, widths);
    memset(&ret, 0x5A, sizeof(ret));                /* every field must be written */
    line_snap_unpack(widths, &ret);
    {
        uint32_t words_out[LINE_SNAP_WORDS];

        line_snap_pack(&ret, words_out);
        CHECK("S3-H22", memcmp(widths, words_out, sizeof(widths)) == 0);
    }
    memset(&ret, 0, sizeof(ret));                   /* same padding as s */
    line_snap_unpack(widths, &ret);
    CHECK("S3-H22", memcmp(&snap, &ret, sizeof(snap)) == 0);
    snap.last_barcode = '\0'; snap.contrast_x10 = 5000u; snap.omega_mrad_s = -6000;
    line_snap_pack(&snap, widths);
    line_snap_unpack(widths, &ret);
    CHECK("S3-H22", ret.last_barcode == '\0' && ret.contrast_x10 == 1023u && ret.omega_mrad_s == -2048);
}

int main(void)
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
    return (g_fail == 0) ? 0 : 1;
}
