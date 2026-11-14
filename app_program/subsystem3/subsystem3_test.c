/*
 *  subsystem3_test.c
 *  On-target tests for subsystem 3.  Built only with: make TEST=1
 *  Record results in testing/subsystem3/TEST_LOG.md.
 *  Decoding and tracking are tested on a PC (testing/subsystem3/host).
 */

#include "subsystem3.h"
#include "code39.h"
#include "line_barcode.h"
#include "../test_report.h"

#if APP_TEST

/* S3-U07: a clean printed *A* (narrow 3 mm, wide 7.5 mm), driven over at
   100 mm/s and sampled at 1 kHz (100 um per sample), 60 mm of white
   before it and 1.2 m in all, as in the host tests (S3-H05).  Levels are
   the normalised 0..1000 scale (1000 = black). */
#define T_NARROW_UM     3000u
#define T_WIDE_UM       7500u
#define T_LEAD_UM       60000u
#define T_TRACK_UM      1200000u
#define T_STEP_UM       100u
#define T_WHITE         80u
#define T_BLACK         920u
#define T_BLOCK_US      10000u          /* one 10 ms ADC block (NFR2)     */

static bc_capture_t g_t_bc;             /* ~3 KB: kept off the task stack */

static uint16_t make_star_code(char c, uint32_t * p_w)
{
    char const seq[3] = { '*', c, '*' };
    uint16_t   n = 0u;
    int        k;
    int        e;

    for (k = 0; k < 3; k++)
    {
        uint16_t const pat = code39_pattern(seq[k]);

        for (e = 0; e < 9; e++)
        {
            p_w[n++] = (((pat >> (8 - e)) & 1u) != 0u) ? T_WIDE_UM : T_NARROW_UM;
        }
        if (k < 2)
        {
            p_w[n++] = T_NARROW_UM;     /* gap between characters */
        }
    }
    return n;
}

static uint16_t level_at(uint32_t const * p_w, uint16_t n, uint32_t x_um)
{
    uint32_t pos = T_LEAD_UM;
    uint16_t e;

    for (e = 0u; e < n; e++)
    {
        if ((x_um >= pos) && (x_um < (pos + p_w[e])))
        {
            return ((e % 2u) == 0u) ? (uint16_t)T_BLACK : (uint16_t)T_WHITE;
        }
        pos += p_w[e];
    }
    return (uint16_t)T_WHITE;
}

EXPORT void subsystem3_test(void)
{
    line_snap_t a;
    line_snap_t b;
    uint32_t    w[29];
    char        ch = '\0';
    bool        b_rev = false;
    uint16_t    n = 0u;
    char const  seq[3] = { '*', 'F', '*' };
    int         k;
    int         e;

    TEST_INIT();

    line_get_snap(&a);
    (void)tk_dly_tsk(100);
    line_get_snap(&b);
    TEST_CHECK("S3-U02", b.t_ms != a.t_ms, "LineBarcodeTask publishes its snapshot");
    TEST_CHECK("S3-U03", ((b.flags & LINE_FLAG_HW_OK) != 0u) && ((b.blocks - a.blocks) >= 8u) &&
                         ((b.blocks - a.blocks) <= 12u),
               "ADC + DMA deliver ~10 blocks per 100 ms (1 kHz per channel)");

    /* Decoder on the target CPU (same code as the host tests). */
    for (k = 0; k < 3; k++)
    {
        uint16_t pat = code39_pattern(seq[k]);

        for (e = 0; e < 9; e++)
        {
            w[n++] = ((pat >> (8 - e)) & 1u) ? 9000u : 3000u;
        }
        if (k < 2)
        {
            w[n++] = 3000u;
        }
    }
    TEST_CHECK("S3-U04", (code39_decode_one(w, n, 15u, &ch, &b_rev) == CODE39_OK) && (ch == 'F'),
               "Code 39 *F* decodes on target");

    /* ---- S3-U05: no ADC block lost in 1 s; live readings (NFR2) ---- */
    line_get_snap(&a);
    (void)tk_dly_tsk(1000);
    line_get_snap(&b);
    tm_printf((UB *)"[INFO] S3-U05 blocks %d in 1 s, lost %d; IR now L %d R %d barcode %d (0..1000, 1000 = black)\n",
              (INT)(b.blocks - a.blocks), (INT)(b.missed_blocks - a.missed_blocks),
              (INT)b.raw_l, (INT)b.raw_r, (INT)b.raw_bc);
    TEST_CHECK("S3-U05", ((b.flags & LINE_FLAG_HW_OK) != 0u) && ((b.blocks - a.blocks) >= 95u) &&
                         ((b.blocks - a.blocks) <= 105u) && (b.missed_blocks == a.missed_blocks) &&
                         (b.raw_l <= 1000u) && (b.raw_r <= 1000u) && (b.raw_bc <= 1000u),
               "100 ADC blocks per second, none lost; readings in 0..1000");

    /* ---- S3-U07: whole barcode pipeline on the target, timed -------- */
    {
        uint32_t    widths[29];
        uint16_t    nw = make_star_code('A', widths);
        uint32_t    x;
        uint32_t    worst_us = 0u;
        uint32_t    total_us = 0u;
        uint32_t    samples  = 0u;
        bc_result_t found    = BC_NONE;
        uint8_t     results  = 0u;
        bc_out_t    out_ok   = {0};

        bc_init(&g_t_bc);
        for (x = 0u; x < T_TRACK_UM; x += T_STEP_UM)
        {
            bc_out_t       out = {0};
            uint32_t const t0  = (uint32_t)app_time_us();
            bc_result_t    r   = bc_sample(&g_t_bc, level_at(widths, nw, x), T_STEP_UM, &out);
            uint32_t const dt  = (uint32_t)app_time_us() - t0;

            total_us += dt;
            samples++;
            if (dt > worst_us)
            {
                worst_us = dt;
            }
            if (r != BC_NONE)
            {
                results++;
                if (found == BC_NONE)
                {
                    found  = r;
                    out_ok = out;
                }
            }
        }
        tm_printf((UB *)"[INFO] S3-U07 result %d char '%c' pass %d; worst sample %d us, mean %d us over %d samples\n",
                  (INT)found, (INT)((out_ok.ch != '\0') ? out_ok.ch : '?'), (INT)g_t_bc.pass,
                  (INT)worst_us, (INT)(total_us / samples), (INT)samples);
        TEST_CHECK("S3-U07", (found == BC_VALID) && (out_ok.ch == 'A') && (results == 1u) &&
                             (worst_us < T_BLOCK_US),
                   "barcode *A* decodes once through bc_sample; slowest call fits a 10 ms block");
    }

    TEST_SUMMARY("subsystem3");
}

#endif /* APP_TEST */
