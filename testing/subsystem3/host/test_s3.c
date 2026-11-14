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

#define N_SLIDES ((int)(sizeof(k_slides) / sizeof(k_slides[0])))

static char const k_data_chars[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ-. $/+%";

static uint32_t g_rng = 12345u;
static uint32_t rnd(void) { g_rng = g_rng * 1103515245u + 12345u; return (g_rng >> 8) & 0xFFFFu; }
/* uniform in [-1, 1] */
static double rndu(void) { return ((double)rnd() / 32767.5) - 1.0; }

/* Widths of *c* with the given narrow width, ratio and +-jitter. */
static uint16_t make_code(char c, uint32_t narrow, double ratio, double jitter, uint32_t * p_w)
{
    char const seq[3] = { '*', c, '*' };
    uint16_t n = 0u;
    int k;
    int e;

    for (k = 0; k < 3; k++)
    {
        uint16_t pat = code39_pattern(seq[k]);

        for (e = 0; e < 9; e++)
        {
            double w = (double)narrow * (((pat >> (8 - e)) & 1u) ? ratio : 1.0);
            p_w[n++] = (uint32_t)(w * (1.0 + jitter * rndu()));
        }
        if (k < 2)
        {
            p_w[n++] = (uint32_t)((double)narrow * (1.0 + jitter * rndu()));   /* gap */
        }
    }
    return n;
}

static void reverse(uint32_t * p_w, uint16_t n)
{
    uint16_t i;

    for (i = 0u; i < n / 2u; i++)
    {
        uint32_t t = p_w[i];
        p_w[i] = p_w[n - 1u - i];
        p_w[n - 1u - i] = t;
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

static uint16_t profile_at(uint32_t const * p_w, uint16_t n, uint32_t x_um)
{
    uint32_t const lead = 60000u;
    uint32_t       spot;
    uint32_t       level = 0u;
    uint32_t       taps  = 0u;

    for (spot = 0u; spot < g_spot_um; spot += 100u)
    {
        taps++;
        uint32_t x = x_um + spot;
        uint32_t pos = lead;
        uint16_t e;
        uint32_t here = g_white;

        for (e = 0u; e < n; e++)
        {
            if ((x >= pos) && (x < pos + p_w[e]))
            {
                if ((e % 2u) == 0u)
                {
                    here = ((g_thin_black != 0u) && (p_w[e] < g_thin_um)) ? g_thin_black : g_black;
                }
                break;
            }
            pos += p_w[e];
        }
        level += here;
    }
    return (uint16_t)(level / taps);
}

/* Run one pass at v_mm_s; returns the result and fills *p_out. */
static bc_result_t run_pass(uint32_t const * p_w, uint16_t n, uint32_t v_mm_s, int noise, bc_out_t * p_out)
{
    static bc_capture_t bc;
    uint32_t     x;
    uint32_t     step = v_mm_s;     /* um per 1 ms sample = mm/s */
    bc_result_t  found = BC_NONE;

    g_last_pass = 0u;
    bc_init(&bc);
    for (x = 0u; x < TRACK_LEN_UM; x += step)
    {
        int32_t  s = (int32_t)profile_at(p_w, n, x) + (int32_t)(noise * rndu());
        bc_out_t out;
        bc_result_t r;

        if (s < 0) { s = 0; }
        if (s > 1000) { s = 1000; }
        memset(&out, 0, sizeof(out));
        r = bc_sample(&bc, (uint16_t)s, step, &out);
        if ((r != BC_NONE) && (found == BC_NONE))
        {
            found = r;
            *p_out = out;
            g_last_pass = bc.pass;
        }
        else if (r != BC_NONE)
        {
            return BC_REJECT;           /* more than one result per code */
        }
    }
    return found;
}

static void test_table(void)
{
    size_t i;
    size_t j;
    int    ok = 1;

    for (i = 0u; i < sizeof(k_data_chars) - 1u; i++)
    {
        uint16_t p = code39_pattern(k_data_chars[i]);
        int wide = 0, wide_sp = 0, b;

        for (b = 0; b < 9; b++)
        {
            if ((p >> (8 - b)) & 1u) { wide++; if (b % 2) { wide_sp++; } }
        }
        if ((p == 0u) || (wide != 3) || ((wide_sp != 1) && (wide_sp != 3))) { ok = 0; }
        if (code39_char(p) != k_data_chars[i]) { ok = 0; }
        for (j = 0u; j < i; j++)
        {
            if (code39_pattern(k_data_chars[j]) == p) { ok = 0; }
        }
    }
    CHECK("S3-H01", ok);
    CHECK("S3-H01", code39_pattern('*') == 0x094u && code39_char(0x1FFu) == '\0');
}

static void test_slides(void)
{
    int s;

    for (s = 0; s < N_SLIDES; s++)
    {
        uint32_t w[29];
        char     ch;
        bool     b_rev;
        int      k;
        double   scale;

        for (scale = 1.0; scale > 0.5; scale -= (1.0 - 297.0 / 338.67))   /* screen, A4 */
        {
            for (k = 0; k < 29; k++) { w[k] = (uint32_t)(k_slides[s].p_w[k] * scale); }
            CHECK("S3-H02", code39_decode_one(w, 29, 15, &ch, &b_rev) == CODE39_OK &&
                            ch == k_slides[s].ch && !b_rev);
            reverse(w, 29);
            CHECK("S3-H02", code39_decode_one(w, 29, 15, &ch, &b_rev) == CODE39_OK &&
                            ch == k_slides[s].ch && b_rev);
            if (scale < 1.0) { break; }
        }
    }
}

static void test_synthetic(void)
{
    uint32_t const narrows[3] = { 3000u, 5100u, 6900u };
    int ok = 1, wrong = 0;
    size_t i;
    int a, r, d;

    for (i = 0u; i < sizeof(k_data_chars) - 1u; i++)
    {
        for (a = 0; a < 3; a++)
        {
            for (r = 0; r < 3; r++)
            {
                double const ratio = (r == 0) ? 2.2 : ((r == 1) ? 2.5 : 3.0);

                for (d = 0; d < 2; d++)
                {
                    uint32_t w[29];
                    uint16_t n = make_code(k_data_chars[i], narrows[a], ratio, 0.15, w);
                    char ch;
                    bool b_rev;
                    code39_result_t res;

                    if (d) { reverse(w, n); }
                    res = code39_decode_one(w, n, 15, &ch, &b_rev);
                    if (res != CODE39_OK) { ok = 0; }
                    else if (ch != k_data_chars[i]) { wrong++; }
                }
            }
        }
    }
    CHECK("S3-H03", ok && (wrong == 0));
}

static void test_rejects(void)
{
    uint32_t w[40];
    uint16_t n;
    char ch;
    bool b_rev;
    int  i, wrong = 0;

    /* a wide bar printed narrow: rejected, never a different character */
    for (i = 0; i < (int)(sizeof(k_data_chars) - 1u); i++)
    {
        int e;
        for (e = 10; e < 19; e++)
        {
            uint16_t pat = code39_pattern(k_data_chars[i]);

            n = make_code(k_data_chars[i], 3000u, 3.0, 0.0, w);
            if ((pat >> (8 - (e - 10))) & 1u)
            {
                w[e] = 3000u;           /* wide -> narrow */
                if (code39_decode_one(w, n, 15, &ch, &b_rev) == CODE39_OK) { wrong++; }
            }
        }
    }
    CHECK("S3-H04", wrong == 0);
    n = make_code('A', 3000u, 3.0, 0.0, w);
    CHECK("S3-H04", code39_decode_one(w, 28, 15, &ch, &b_rev) == CODE39_ERR_COUNT);
    n = make_code('A', 3000u, 1.3, 0.0, w);
    CHECK("S3-H04", code39_decode_one(w, n, 15, &ch, &b_rev) == CODE39_ERR_RATIO);
    CHECK("S3-H04", code39_decode_one(w, 5, 15, &ch, &b_rev) == CODE39_NOT_BARCODE);
    {
        /* "*AB*": valid Code 39 but two data characters */
        uint32_t w2[40];
        char const seq[4] = { '*', 'A', 'B', '*' };
        int k, e;
        n = 0u;
        for (k = 0; k < 4; k++)
        {
            uint16_t pat = code39_pattern(seq[k]);
            for (e = 0; e < 9; e++) { w2[n++] = ((pat >> (8 - e)) & 1u) ? 9000u : 3000u; }
            if (k < 3) { w2[n++] = 3000u; }
        }
        CHECK("S3-H04", code39_decode_one(w2, n, 15, &ch, &b_rev) == CODE39_ERR_LENGTH);
    }
}

static void test_capture(void)
{
    uint32_t const speeds[2] = { 150u, 300u };      /* NFR3 speeds, mm/s */
    int s, v, d, pass;
    int good = 0, total = 0, wrong = 0;

    /* every slide, both speeds, both directions, clean */
    for (s = 0; s < N_SLIDES; s++)
    {
        for (v = 0; v < 2; v++)
        {
            for (d = 0; d < 2; d++)
            {
                uint32_t w[29];
                bc_out_t out;
                int k;

                for (k = 0; k < 29; k++) { w[k] = k_slides[s].p_w[k] * 297u / 339u; }
                if (d) { reverse(w, 29); }
                memset(&out, 0, sizeof(out));
                CHECK("S3-H05", run_pass(w, 29, speeds[v], 0, &out) == BC_VALID && out.ch == k_slides[s].ch);
            }
        }
    }

    /* NFR3-style: 20 noisy passes per character per speed and direction */
    for (s = 0; s < N_SLIDES; s += 3)
    {
        for (v = 0; v < 2; v++)
        {
            for (pass = 0; pass < 20; pass++)
            {
                uint32_t w[29];
                bc_out_t out;
                bc_result_t r;
                int k;

                for (k = 0; k < 29; k++)
                {
                    w[k] = (uint32_t)((double)k_slides[s].p_w[k] * 297.0 / 338.67 * (1.0 + 0.10 * rndu()));
                }
                if (pass % 2) { reverse(w, 29); }
                memset(&out, 0, sizeof(out));
                r = run_pass(w, 29, speeds[v], 120, &out);
                total++;
                if ((r == BC_VALID) && (out.ch == k_slides[s].ch)) { good++; }
                else if (r == BC_VALID) { wrong++; }
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
        int sp;
        int blur_good = 0, blur_total = 0, blur_wrong = 0;
        int weak_good = 0, weak_total = 0, weak_wrong = 0;

        for (sp = 0; sp < 2; sp++)
        {
            g_spot_um = spots[sp];
            for (s = 0; s < N_SLIDES; s++)
            {
                for (v = 0; v < 2; v++)
                {
                    for (d = 0; d < 2; d++)
                    {
                        uint32_t w[29];
                        bc_out_t out;
                        bc_result_t r;
                        int k;

                        for (k = 0; k < 29; k++) { w[k] = k_slides[s].p_w[k] * 297u / 339u; }
                        if (d) { reverse(w, 29); }
                        memset(&out, 0, sizeof(out));
                        r = run_pass(w, 29, speeds[v], (d ? 60 : 0), &out);
                        blur_total++;
                        if ((r == BC_VALID) && (out.ch == k_slides[s].ch)) { blur_good++; }
                        else if (r == BC_VALID) { blur_wrong++; }
                    }
                }
            }
        }
        g_spot_um = 4000u;  g_white = 250u;  g_black = 800u;
        for (s = 0; s < N_SLIDES; s++)
        {
            uint32_t w[29];
            bc_out_t out;
            int k;

            for (k = 0; k < 29; k++) { w[k] = k_slides[s].p_w[k] * 297u / 339u; }
            bc_result_t r;

            memset(&out, 0, sizeof(out));
            r = run_pass(w, 29, 150u, 0, &out);
            weak_total++;
            if ((r == BC_VALID) && (out.ch == k_slides[s].ch)) { weak_good++; }
            else if (r == BC_VALID) { weak_wrong++; }
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
        int good = 0, total = 0, wrong = 0;
        int k;

        for (s = 0; s < N_SLIDES; s++)
        {
            for (d = 0; d < 2; d++)
            {
                int cut;

                for (cut = 0; cut < 2; cut++)
                {
                    uint32_t w[29];
                    char ch;
                    bool b_rev;

                    for (k = 0; k < 29; k++) { w[k] = k_slides[s].p_w[k]; }
                    if (d) { reverse(w, 29); }
                    total++;
                    if (code39_decode_one_repair(cut ? w : &w[2], 27, 15, &ch, &b_rev) == CODE39_OK)
                    {
                        if (ch == k_slides[s].ch) { good++; } else { wrong++; }
                    }
                }
            }
        }
        printf("one bar short at an end: %d / %d decoded, %d wrong\n", good, total, wrong);
        CHECK("S3-H18", (wrong == 0) && (good == total));

        good = 0; total = 0; wrong = 0;
        {
            size_t i;

            for (i = 0u; i < 26u; i++)
            {
                uint32_t w[29];
                uint16_t n = make_code((char)('A' + i), 3000u, 2.5, 0.0, w);
                char ch;
                bool b_rev;
                uint16_t e;

                for (e = 0u; e < n; e++)
                {
                    w[e] = ((e % 2u) == 0u) ? (w[e] + 1200u) : ((w[e] > 1200u) ? (w[e] - 1200u) : 100u);
                }
                total++;
                if (code39_decode_one(w, n, 15, &ch, &b_rev) == CODE39_OK)
                {
                    if (ch == (char)('A' + i)) { good++; } else { wrong++; }
                }
            }
        }
        printf("black bleeding 40 %%: %d / %d decoded, %d wrong\n", good, total, wrong);
        CHECK("S3-H18", (wrong == 0) && (good == total));

        {
            uint32_t const spots[2] = { 5000u, 6000u };
            int sp, pass2 = 0;

            good = 0; total = 0; wrong = 0;
            for (sp = 0; sp < 2; sp++)
            {
                g_spot_um = spots[sp];
                for (s = 0; s < N_SLIDES; s++)
                {
                    for (d = 0; d < 2; d++)
                    {
                        uint32_t w[29];
                        bc_out_t out;
                        bc_result_t r;

                        for (k = 0; k < 29; k++) { w[k] = k_slides[s].p_w[k] * 297u / 339u; }
                        if (d) { reverse(w, 29); }
                        memset(&out, 0, sizeof(out));
                        r = run_pass(w, 29, 90u, 0, &out);
                        total++;
                        if ((r == BC_VALID) && (out.ch == k_slides[s].ch)) { good++; }
                        else if (r == BC_VALID) { wrong++; }
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
        int s, d, good = 0, total = 0, wrong = 0;
        int noise;

        g_spot_um = 1500u;  g_white = 20u;  g_black = 690u;  g_thin_black = 240u;
        for (noise = 0; noise <= 40; noise += 40)
        {
            for (s = 0; s < N_SLIDES; s++)
            {
                if (k_slides[s].ch != 'Z') { continue; }
                for (d = 0; d < 2; d++)
                {
                    uint32_t w[29];
                    bc_out_t out;
                    bc_result_t r;
                    int k;

                    uint32_t wmin = 0xFFFFFFFFu;

                    /* printed at the specified size: 3 mm narrow (~141 mm code) */
                    for (k = 0; k < 29; k++) { wmin = (k_slides[s].p_w[k] < wmin) ? k_slides[s].p_w[k] : wmin; }
                    for (k = 0; k < 29; k++) { w[k] = (k_slides[s].p_w[k] * 3000u) / wmin; }
                    g_thin_um = 6000u;          /* bars under 6 mm are the faint narrow ones */
                    if (d) { reverse(w, 29); }
                    memset(&out, 0, sizeof(out));
                    r = run_pass(w, 29, 90u, noise, &out);
                    total++;
                    if ((r == BC_VALID) && (out.ch == 'Z')) { good++; }
                    else if (r == BC_VALID) { wrong++; }
                    else { printf("  Z slide %d dir %d noise %d: result %d, %u elements, %u mm\n", k_slides[s].slide, d,
                                  noise, (int)r, (unsigned)out.elements, (unsigned)(out.length_um / 1000u)); }
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
        int d, noise, good = 0, total = 0, wrong = 0;

        g_spot_um = 1000u;  g_white = 25u;  g_black = 1000u;
        for (noise = 0; noise <= 30; noise += 30)
        {
            for (d = 0; d < 4; d++)
            {
                uint32_t       w[33];
                uint16_t const n = (d < 2) ? 31u : 33u;
                bc_out_t       out;
                bc_result_t    r;

                memcpy(w, (d < 2) ? k_r1 : k_r2, n * sizeof(uint32_t));
                if ((d % 2) != 0) { reverse(w, n); }
                memset(&out, 0, sizeof(out));
                r = run_pass(w, n, 90u, noise, &out);
                total++;
                if ((r == BC_VALID) && (out.ch == 'A') && (out.b_reversed == ((d % 2) != 0))) { good++; }
                else if (r == BC_VALID) { wrong++; }
                else { printf("  14 Nov read %d dir %d noise %d: result %d reason %u, %u elements\n", d / 2 + 1, d % 2,
                              noise, (int)r, (unsigned)out.reason, (unsigned)out.elements); }
            }
        }
        g_spot_um = 1500u;  g_white = 80u;  g_black = 920u;
        printf("end specks (car 14 Nov): A %d / %d decoded, %d wrong\n", good, total, wrong);
        CHECK("S3-H20", (wrong == 0) && (good == total));
    }

    /* S3-H06: a crossing line (18 mm black) is not a barcode and not an error */
    {
        uint32_t w[1] = { 18000u };
        bc_out_t out;
        CHECK("S3-H06", run_pass(w, 1, 300u, 0, &out) == BC_NONE);
    }
    /* S3-H07: a long dark patch is rejected, never decoded */
    {
        uint32_t w[1] = { 500000u };
        bc_out_t out;
        memset(&out, 0, sizeof(out));
        CHECK("S3-H07", run_pass(w, 1, 300u, 0, &out) == BC_REJECT && out.reason == CODE39_ERR_TOO_LONG);
    }
}

/* ---- line tracking ---------------------------------------------------- */
static line_trk_cfg_t const k_cfg = { 20000u, 80000u, 300 };

/* Feed blocks of 3 mm (0.3 m/s x 10 ms) for dist_um; returns OR of events. */
static uint8_t feed(line_track_t * p_t, uint16_t l, uint16_t r, uint32_t dist_um)
{
    uint8_t ev = 0u;
    uint32_t d;

    for (d = 0u; d < dist_um; d += 3000u) { ev |= line_track_step(p_t, &k_cfg, l, r, 3000u); }
    return ev;
}

static void test_track(void)
{
    line_track_t t;
    uint8_t ev;

    /* S3-H10: steering sign: line under the left sensor -> turn left */
    line_track_init(&t);
    (void)feed(&t, 500u, 500u, 60000u);
    CHECK("S3-H10", t.err == 0 && t.omega == 0 && t.v == 300);
    (void)line_track_step(&t, &k_cfg, 800u, 200u, 3000u);
    CHECK("S3-H10", t.err == 600 && t.omega > 0 && t.v < 300);
    (void)feed(&t, 200u, 800u, 9000u);
    CHECK("S3-H10", t.omega < 0);

    /* S3-H08: junction on a straight approach, once; not on a tight curve */
    line_track_init(&t);
    (void)feed(&t, 500u, 500u, 60000u);
    ev = feed(&t, 1000u, 1000u, 15000u);
    CHECK("S3-H08", (ev & LINE_TRK_EV_JUNCTION) == 0u);
    ev = feed(&t, 1000u, 1000u, 15000u);
    CHECK("S3-H08", (ev & LINE_TRK_EV_JUNCTION) != 0u && t.omega == 0);
    ev = feed(&t, 1000u, 1000u, 30000u);
    CHECK("S3-H08", (ev & LINE_TRK_EV_JUNCTION) == 0u);
    line_track_init(&t);
    (void)feed(&t, 950u, 300u, 30000u);        /* hard steering on a curve */
    ev = feed(&t, 1000u, 1000u, 30000u);
    CHECK("S3-H08", (ev & LINE_TRK_EV_JUNCTION) == 0u);

    /* S3-H09: 50 mm gap bridged (NFR11, heading held), 90 mm -> LOST once, then FOUND */
    line_track_init(&t);
    (void)feed(&t, 500u, 500u, 60000u);
    ev = feed(&t, 50u, 50u, 50000u);
    CHECK("S3-H09", (ev & LINE_TRK_EV_LOST) == 0u && t.omega == 0 && t.v == 300);
    (void)feed(&t, 500u, 500u, 30000u);
    ev = feed(&t, 50u, 50u, 90000u);
    CHECK("S3-H09", (ev & LINE_TRK_EV_LOST) != 0u && t.b_lost && t.v == 0);
    ev = feed(&t, 50u, 50u, 90000u);
    CHECK("S3-H09", (ev & LINE_TRK_EV_LOST) == 0u);
    ev = feed(&t, 600u, 400u, 3000u);
    CHECK("S3-H09", (ev & LINE_TRK_EV_FOUND) != 0u && !t.b_lost);

    /* S3-H11: drifting off the right of the line -> keeps turning back left */
    line_track_init(&t);
    (void)feed(&t, 500u, 500u, 60000u);
    (void)feed(&t, 950u, 80u, 6000u);          /* line under the left sensor only */
    (void)feed(&t, 60u, 60u, 9000u);           /* ... then past it */
    CHECK("S3-H11", t.omega > 0 && t.v < 300);

    /* S3-H14: right curve, the line slides out past the right sensor and
       the error FADES (-1000 -> -300) before both read white: still a turn
       back right (was: taken for a gap, straight on); an old strong error
       (more than LINE_EXIT_PEAK_UM ago) does not count, a gap stays a gap */
    line_track_init(&t);
    (void)feed(&t, 500u, 500u, 60000u);
    (void)feed(&t, 0u, 1000u, 6000u);          /* line under the right sensor */
    (void)feed(&t, 0u, 300u, 6000u);           /* sliding out: error fades */
    CHECK("S3-H14", t.err == -300);
    (void)line_track_step(&t, &k_cfg, 60u, 60u, 3000u);   /* both white */
    CHECK("S3-H14", t.exit_err == -1000 && t.omega < 0 && t.v < 300);
    line_track_init(&t);
    (void)feed(&t, 500u, 500u, 60000u);
    (void)feed(&t, 0u, 1000u, 6000u);
    (void)feed(&t, 500u, 500u, 45000u);        /* centred again for 45 mm */
    (void)feed(&t, 60u, 60u, 9000u);           /* a gap */
    CHECK("S3-H14", t.omega == 0 && t.v == 300);

    /* S3-H13: steering ramp: the error flips full left / full right every
       10 ms (the weaving seen on 2 Nov); the turn command never moves more
       than LINE_OMEGA_STEP per block; a steady error still reaches the full
       command within 850 / 50 = 17 blocks */
    {
        int32_t prev;
        int32_t i;
        int32_t max_jump = 0;

        line_track_init(&t);
        (void)feed(&t, 500u, 500u, 30000u);
        prev = t.omega;
        for (i = 0; i < 40; i++)
        {
            if ((i & 1) == 0) { (void)line_track_step(&t, &k_cfg, 1000u, 0u, 3000u); }
            else              { (void)line_track_step(&t, &k_cfg, 0u, 1000u, 3000u); }
            max_jump = (app_abs_i32(t.omega - prev) > max_jump) ? app_abs_i32(t.omega - prev) : max_jump;
            prev = t.omega;
        }
        CHECK("S3-H13", max_jump <= LINE_OMEGA_STEP);

        line_track_init(&t);
        (void)feed(&t, 500u, 500u, 30000u);
        for (i = 0; i < 17; i++) { (void)line_track_step(&t, &k_cfg, 1000u, 0u, 3000u); }
        CHECK("S3-H13", t.omega == LINE_STEER_SIGN * LINE_KP);
        (void)line_track_step(&t, &k_cfg, 500u, 500u, 3000u);   /* back on centre */
        CHECK("S3-H13", t.omega == LINE_STEER_SIGN * app_max_i32(LINE_KP - LINE_OMEGA_STEP, 0));
    }

    /* S3-H12: calibration: contrast check and normalisation */
    {
        line_cal_t cal;
        uint16_t raw[3];
        int i;

        memset(&cal, 0, sizeof(cal));
        line_cal_start(&cal);
        for (i = 0; i < 100; i++)
        {
            raw[0] = (uint16_t)((i < 50) ? 300 : 3000);  /* white 300, black 3000 */
            raw[1] = (uint16_t)((i < 50) ? 350 : 2900);
            raw[2] = (uint16_t)((i < 50) ? 250 : 2500);
            line_cal_sample(&cal, raw);
        }
        CHECK("S3-H12", line_cal_finish(&cal) && cal.contrast_x10 >= 30);
        CHECK("S3-H12", line_cal_norm(&cal, 0, 300) == 0 && line_cal_norm(&cal, 0, 3000) == 1000);
        CHECK("S3-H12", line_cal_norm(&cal, 0, 1650) == 500 && line_cal_norm(&cal, 0, 4095) == 1000);
        line_cal_start(&cal);
        for (i = 0; i < 100; i++)
        {
            raw[0] = (uint16_t)((i < 50) ? 1000 : 2000);  /* 2:1 only */
            raw[1] = 1000; raw[2] = 1000;
            line_cal_sample(&cal, raw);
        }
        CHECK("S3-H12", !line_cal_finish(&cal) && cal.b_valid && line_cal_norm(&cal, 0, 3000) == 1000);
        /* the barcode sensor does not take part: fixed levels from
           line_cfg.h, even when it saw no contrast during the spin */
        line_cal_start(&cal);
        for (i = 0; i < 100; i++)
        {
            raw[0] = (uint16_t)((i < 50) ? 300 : 3000);
            raw[1] = (uint16_t)((i < 50) ? 350 : 2900);
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

static uint16_t slip_raw(double x, double y)
{
    double const half = 9.0;
    double       f;

    (void)x;
    f = (half - fabs(y) + 4.0) / 8.0;
    f = (f < 0.0) ? 0.0 : ((f > 1.0) ? 1.0 : f);
    return (uint16_t)(300.0 + (f * 2700.0));
}

static uint16_t slip_run(int gain_pm, bool b_line, int start_cdeg)
{
    static line_slip_t slip;
    line_cal_t         cal;
    int                k;

    memset(&cal, 0, sizeof(cal));
    line_cal_start(&cal);
    line_slip_start(&slip, start_cdeg);
    for (k = 1; k <= 400; k++)
    {
        double const enc  = 360.0 * k / 400.0;
        double const real = enc * 1000.0 / gain_pm * M_PI / 180.0;
        double const xs   = 120.0 * cos(real);
        double const yl   = (120.0 * sin(real)) + (g_slip_half * cos(real));
        double const yr   = (120.0 * sin(real)) - (g_slip_half * cos(real));
        uint16_t     raw[3];
        int          h    = start_cdeg + (int)(enc * 100.0);

        while (h > 18000) { h -= 36000; }
        raw[0] = b_line ? slip_raw(xs, yl) : 300u;
        raw[1] = b_line ? slip_raw(xs, yr) : 300u;
        raw[2] = 300u;
        if (!b_line && (k == 200)) { raw[0] = 3000u; raw[1] = 3000u; }   /* contrast for the cal */
        line_cal_sample(&cal, raw);
        line_slip_sample(&slip, h, raw[0], raw[1]);
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
    int       i;

    for (i = 0; i < 4; i++)
    {
        uint16_t const g = slip_run(gains[i], true, 17000);

        printf("turn slip: real gain %d -> measured %u\n", gains[i], (unsigned)g);
        CHECK("S3-H16", abs((int)g - gains[i]) <= 25);
    }
    CHECK("S3-H16", slip_run(1500, true, -9000) != 0u);
    /* sensors 25 mm either side: they cross the line one after the other */
    g_slip_half = 25.0;
    for (i = 0; i < 4; i++)
    {
        uint16_t const g = slip_run(gains[i], true, 0);

        printf("turn slip, wide sensors: real gain %d -> measured %u\n", gains[i], (unsigned)g);
        CHECK("S3-H16", abs((int)g - gains[i]) <= 25);
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
static uint16_t split_at(uint32_t const * p_in, uint16_t n, uint16_t at, uint32_t speck, uint32_t * p_out)
{
    uint16_t o = 0u;
    uint16_t i;

    for (i = 0u; i < n; i++)
    {
        if (i == at)
        {
            uint32_t const a = (p_in[i] - speck) / 2u;

            p_out[o++] = a;
            p_out[o++] = speck;
            p_out[o++] = p_in[i] - speck - a;
        }
        else
        {
            p_out[o++] = p_in[i];
        }
    }
    return o;
}

static uint16_t merge_at(uint32_t const * p_in, uint16_t n, uint16_t at, uint32_t * p_out)
{
    uint16_t o = 0u;
    uint16_t i;

    for (i = 0u; i < n; i++)
    {
        if ((i == at) && ((i + 2u) < n))
        {
            p_out[o++] = p_in[i] + p_in[i + 1u] + p_in[i + 2u];
            i = (uint16_t)(i + 2u);
        }
        else
        {
            p_out[o++] = p_in[i];
        }
    }
    return o;
}

/* the merges the sensor makes: a thin space (or bar) between two of the
   other kind, at least one of them thin */
static bool thin_triplet(uint32_t const * p_w, uint16_t at, uint32_t narrow)
{
    return (p_w[at + 1u] < (narrow * 3u / 2u)) &&
           ((p_w[at] < (narrow * 3u / 2u)) || (p_w[at + 2u] < (narrow * 3u / 2u)));
}

static void test_best(void)
{
    size_t c;
    int    good = 0, total = 0, wrong = 0, kind;

    for (kind = 0; kind < 4; kind++)
    {
        int kgood = 0, ktotal = 0;

        for (c = 0u; c < (sizeof(k_data_chars) - 1u); c++)
        {
            uint16_t at;

            for (at = 1u; at < 28u; at++)
            {
                int d;

                for (d = 0; d < 2; d++)
                {
                    uint32_t w[29], a[40], b[40];
                    uint16_t n = make_code(k_data_chars[c], 3000u, 2.5, 0.08, w), e, m;
                    char     ch;
                    bool     b_rev;

                    for (e = 0u; e < n; e++)
                    {
                        w[e] = ((e % 2u) == 0u) ? (w[e] + 800u) : (w[e] - 800u);
                    }
                    if (d) { reverse(w, n); }
                    if (kind == 0)
                    {
                        if ((at % 2u) != 0u) { continue; }     /* a speck in a bar */
                        m = split_at(w, n, at, 1000u, a);
                    }
                    else if (kind == 1)
                    {
                        if ((at > 26u) || !thin_triplet(w, at, 3000u)) { continue; }
                        m = merge_at(w, n, at, a);
                    }
                    else if (kind == 2)
                    {
                        uint16_t const sp = (uint16_t)((at + 12u) % 27u);

                        if ((at > 26u) || !thin_triplet(w, at, 3000u) || ((sp % 2u) != 0u) ||
                            ((sp + 2u) >= at && sp <= (at + 2u))) { continue; }
                        m = merge_at(w, n, at, b);
                        m = split_at(b, m, (sp < at) ? sp : (uint16_t)(sp - 2u), 1000u, a);
                    }
                    else
                    {
                        if (((at % 2u) != 0u) || (at < 2u)) { continue; }   /* first bar lost, speck in a bar */
                        m = split_at(&w[2], (uint16_t)(n - 2u), (uint16_t)(at - 2u), 1000u, a);
                    }
                    ktotal++;
                    if (code39_decode_one_best(a, m, &ch, &b_rev) == CODE39_OK)
                    {
                        if (ch == k_data_chars[c]) { kgood++; } else { wrong++;
                            printf("  WRONG kind %d '%c' at %u d %d -> '%c'\n", kind, k_data_chars[c], (unsigned)at, d, ch); }
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
        int r, accepted = 0, bad = 0, tries = 0;

        for (r = 0; r < 20000; r++)
        {
            uint32_t w[60];
            uint16_t n = (uint16_t)(25u + 2u * (rnd() % 5u)), e;
            char     ch;
            bool     b_rev;

            for (e = 0u; e < n; e++) { w[e] = 1500u + (uint32_t)(rnd() % 9000u); }
            tries++;
            if (code39_decode_one_best(w, n, &ch, &b_rev) == CODE39_OK) { accepted++; }
        }
        for (c = 0u; c < (sizeof(k_data_chars) - 1u); c++)
        {
            for (r = 0; r < 40; r++)
            {
                uint32_t w[29], a[60], b[60];
                uint16_t n = make_code(k_data_chars[c], 3000u, 2.5, 0.08, w), m, q, e;
                char     ch;
                bool     b_rev;

                /* three random merges or splits: more than it may repair */
                m = n;
                for (e = 0u; e < 29u; e++) { a[e] = w[e]; }
                for (q = 0u; q < 3u; q++)
                {
                    uint16_t const at = (uint16_t)(1u + (rnd() % (m - 4u)));
                    uint16_t       x;

                    m = ((rnd() & 1u) != 0u) ? merge_at(a, m, at, b) : split_at(a, m, at, 900u, b);
                    for (x = 0u; x < m; x++) { a[x] = b[x]; }
                }
                tries++;
                if ((code39_decode_one_best(a, m, &ch, &b_rev) == CODE39_OK) && (ch != k_data_chars[c])) { bad++; }
            }
        }
        printf("best match: random widths accepted %d / 20000, over-damaged codes wrong %d\n", accepted, bad);
        CHECK("S3-H20", (accepted == 0) && (bad == 0));
        (void)tries;
    }

    /* the slide samples through the whole capture with a speck in a wide bar */
    {
        int s2, d2, g2 = 0, t2 = 0, w2n = 0, p3 = 0;

        g_spot_um = 300u;               /* sharp enough that the speck reads as a gap */

        for (s2 = 0; s2 < N_SLIDES; s2++)
        {
            for (d2 = 0; d2 < 2; d2++)
            {
                uint32_t w[29], a[31], wmin = 0xFFFFFFFFu;
                uint16_t k, at = 0u;
                bc_out_t out;
                bc_result_t rr;

                for (k = 0u; k < 29u; k++) { wmin = (k_slides[s2].p_w[k] < wmin) ? k_slides[s2].p_w[k] : wmin; }
                for (k = 0u; k < 29u; k++) { w[k] = (k_slides[s2].p_w[k] * 3000u) / wmin; }
                if (d2) { reverse(w, 29); }
                for (k = 10u; k < 19u; k += 2u) { at = (w[k] > w[at]) ? k : at; }   /* widest bar of X */
                (void)split_at(w, 29, at, 1200u, a);
                memset(&out, 0, sizeof(out));
                rr = run_pass(a, 31, 150u, 0, &out);
                t2++;
                if ((rr == BC_VALID) && (out.ch == k_slides[s2].ch)) { g2++; p3 += (g_last_pass == 4u) ? 1 : 0; }
                else if (rr == BC_VALID) { w2n++; }
                else { printf("  speck slide %d dir %d: result %d reason %d, %u elements\n", k_slides[s2].slide, d2,
                              (int)rr, (int)out.reason, (unsigned)out.elements); }
            }
        }
        g_spot_um = 1500u;
        printf("capture with a speck in a wide bar: %d / %d decoded (%d by best match), %d wrong\n", g2, t2, p3, w2n);
        CHECK("S3-H20", (w2n == 0) && ((g2 * 100) >= (t2 * 90)) && (p3 > 0));
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
    int th;
    uint32_t const speeds[3] = { 90u, 150u, 250u };
    int sp, v, d, good = 0, total = 0, wrong = 0, by_adapt = 0, before = 0;
    size_t c;

    g_white = 7u;  g_black = 715u;  g_thin_um = 6000u;
    for (th = 0; th < 3; th++)
    for (sp = 0; sp < 3; sp++)
    {
        g_spot_um = spots[sp];  g_thin_black = thin[th];
        for (c = 0u; c < (sizeof(k_data_chars) - 1u); c++)
        {
            for (v = 0; v < 3; v++)
            {
                for (d = 0; d < 2; d++)
                {
                    uint32_t w[29];
                    uint16_t n = make_code(k_data_chars[c], 3000u, 3.0, 0.05, w);
                    bc_out_t out;
                    bc_result_t r;

                    if (d) { reverse(w, n); }
                    memset(&out, 0, sizeof(out));
                    r = run_pass(w, n, speeds[v], 25, &out);
                    total++;
                    if ((r == BC_VALID) && (out.ch == k_data_chars[c]))
                    {
                        good++;
                        by_adapt += (g_last_pass == 3u) ? 1 : 0;
                        before   += (g_last_pass < 3u) ? 1 : 0;
                    }
                    else if (r == BC_VALID)
                    {
                        wrong++;
                        printf("  WRONG light '%c' spot %u v %u d %d -> '%c' pass %u\n", k_data_chars[c],
                               (unsigned)spots[sp], (unsigned)speeds[v], d, out.ch, (unsigned)g_last_pass);
                    }
                }
            }
        }
        printf("light print, thin bars %u, %u mm spot: %d / %d so far (%d by the adaptive swing, %d by the first three readings)\n",
               (unsigned)thin[th], (unsigned)(spots[sp] / 1000u), good, total, by_adapt, before);
    }
    g_spot_um = 1500u;  g_white = 80u;  g_black = 920u;  g_thin_black = 0u;  g_thin_um = 0u;
    printf("light print: %d / %d decoded (%d by the adaptive swing), %d wrong\n", good, total, by_adapt, wrong);
    CHECK("S3-H21", (wrong == 0) && (by_adapt > 0));
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
    printf("host tests: %d passed, %d failed\n", g_pass, g_fail);
    return (g_fail == 0) ? 0 : 1;
}
