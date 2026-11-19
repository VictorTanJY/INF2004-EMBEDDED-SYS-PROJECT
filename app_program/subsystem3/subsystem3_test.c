/*
 *  subsystem3_test.c
 *  On-target tests for subsystem 3.  Built only with: make TEST=1
 *  Record results in testing/subsystem3/TEST_LOG.md.
 *  Decoding and tracking are tested on a PC (testing/subsystem3/host).
 */

#include "subsystem3_test.h"
#include "subsystem3.h"
#include "code39.h"
#include "line_barcode.h"
#include "../test_report.h"

#define T_SYMBOLS    3 /* *X*: start, data, stop */
#define T_CHAR_ELEMS 9 /* bars and spaces in one character */

#if APP_TEST

/* S3-U07: a clean printed *A* (narrow 3 mm, wide 7.5 mm), driven over at
   100 mm/s and sampled at 1 kHz (100 um per sample), 60 mm of white
   before it and 1.2 m in all, as in the host tests (S3-H05).  Levels are
   the normalised 0..1000 scale (1000 = black). */
#define T_NARROW_UM 3000u
#define T_WIDE_UM   7500u
#define T_LEAD_UM   60000u
#define T_TRACK_UM  1200000u
#define T_STEP_UM   100u
#define T_WHITE     80u
#define T_BLACK     920u
#define T_BLOCK_US  10000u /* one 10 ms ADC block (NFR2)     */

#endif

#if APP_TEST

static line_barcode_capture_t g_t_bc; /* ~3 KB: kept off the task stack */

#endif

#if APP_TEST
static uint16_t make_star_code(char chr, uint32_t * p_w);
static uint16_t level_at(uint32_t const * p_w, uint16_t n_elems, uint32_t x_um);
#endif

#if APP_TEST

EXPORT void subsystem3_test_run (void)
{
    line_types_snap_t snap_a;
    line_types_snap_t snap_b;
    uint32_t          elem_widths[29];
    char              chr     = '\0';
    bool              b_rev   = false;
    uint16_t          n_elems = 0u;
    char const        seq[3]  = {'*', 'F', '*'};
    int32_t           kdx;
    int32_t           elem;

    test_report_t report = {0, 0, 0};

    subsystem3_get_snap(&snap_a);
    (void)tk_dly_tsk(100);
    subsystem3_get_snap(&snap_b);
    TEST_CHECK(&report, "S3-U02", snap_b.t_ms != snap_a.t_ms,
               "LineBarcodeTask publishes its snapshot");
    TEST_CHECK(&report, "S3-U03",
               ((snap_b.flags & LINE_FLAG_HW_OK) != 0u)
                   && ((snap_b.blocks - snap_a.blocks) >= 8u)
                   && ((snap_b.blocks - snap_a.blocks) <= 12u),
               "ADC + DMA deliver ~10 blocks per 100 ms (1 kHz per channel)");

    /* Decoder on the target CPU (same code as the host tests). */
    for (kdx = 0; kdx < T_SYMBOLS; kdx++)
    {
        uint16_t pat = code39_pattern(seq[kdx]);

        for (elem = 0; elem < T_CHAR_ELEMS; elem++)
        {
            elem_widths[n_elems++] = ((pat >> (8 - elem)) & 1u) ? 9000u : 3000u;
        }
        if (kdx < 2)
        {
            elem_widths[n_elems++] = 3000u;
        }
    }
    TEST_CHECK(&report, "S3-U04",
               (CODE39_OK
                == code39_decode_one(elem_widths, n_elems, 15u, &chr, &b_rev))
                   && ('F' == chr),
               "Code 39 *F* decodes on target");

    /* ---- S3-U05: no ADC block lost in 1 s; live readings (NFR2) ---- */
    subsystem3_get_snap(&snap_a);
    (void)tk_dly_tsk(1000);
    subsystem3_get_snap(&snap_b);
    tm_printf((UB *)"[INFO] S3-U05 blocks %d in 1 s, lost %d; IR now L %d R %d "
                    "barcode %d (0..1000, 1000 = black)\n",
              (INT)(snap_b.blocks - snap_a.blocks),
              (INT)(snap_b.missed_blocks - snap_a.missed_blocks),
              (INT)snap_b.raw_l, (INT)snap_b.raw_r, (INT)snap_b.raw_bc);
    TEST_CHECK(&report, "S3-U05",
               ((snap_b.flags & LINE_FLAG_HW_OK) != 0u)
                   && ((snap_b.blocks - snap_a.blocks) >= 95u)
                   && ((snap_b.blocks - snap_a.blocks) <= 105u)
                   && (snap_b.missed_blocks == snap_a.missed_blocks)
                   && (snap_b.raw_l <= 1000u) && (snap_b.raw_r <= 1000u)
                   && (snap_b.raw_bc <= 1000u),
               "100 ADC blocks per second, none lost; readings in 0..1000");

    /* ---- S3-U07: whole barcode pipeline on the target, timed -------- */
    {
        uint32_t              widths[29];
        uint16_t              n_widths = make_star_code('A', widths);
        uint32_t              x_val;
        uint32_t              worst_us = 0u;
        uint32_t              total_us = 0u;
        uint32_t              samples  = 0u;
        line_barcode_result_t found    = BC_NONE;
        uint8_t               results  = 0u;
        line_barcode_out_t    out_ok   = {0};

        line_barcode_init(&g_t_bc);
        for (x_val = 0u; x_val < T_TRACK_UM; x_val += T_STEP_UM)
        {
            line_barcode_out_t    out     = {0};
            uint32_t const        t_start = (uint32_t)app_time_us();
            line_barcode_result_t result  = line_barcode_sample(
                &g_t_bc, level_at(widths, n_widths, x_val), T_STEP_UM, &out);
            uint32_t const elapsed = (uint32_t)app_time_us() - t_start;

            total_us += elapsed;
            samples++;
            if (elapsed > worst_us)
            {
                worst_us = elapsed;
            }
            if (result != BC_NONE)
            {
                results++;
                if (BC_NONE == found)
                {
                    found  = result;
                    out_ok = out;
                }
            }
        }
        tm_printf((UB *)"[INFO] S3-U07 result %d char '%c' pass %d; worst "
                        "sample %d us, mean %d us over %d samples\n",
                  (INT)found, (INT)((out_ok.chr != '\0') ? out_ok.chr : '?'),
                  (INT)g_t_bc.pass, (INT)worst_us, (INT)(total_us / samples),
                  (INT)samples);
        TEST_CHECK(&report, "S3-U07",
                   (BC_VALID == found) && ('A' == out_ok.chr) && (1u == results)
                       && (worst_us < T_BLOCK_US),
                   "barcode *A* decodes once through bc_sample; slowest call "
                   "fits a 10 ms block");
    }

    test_report_summary(&report, "subsystem3");
}

#endif

#if APP_TEST

static uint16_t make_star_code (char chr, uint32_t * p_w)
{
    char const seq[3]  = {'*', chr, '*'};
    uint16_t   n_elems = 0u;
    int32_t    kdx;
    int32_t    elem;

    for (kdx = 0; kdx < T_SYMBOLS; kdx++)
    {
        uint16_t const pat = code39_pattern(seq[kdx]);

        for (elem = 0; elem < T_CHAR_ELEMS; elem++)
        {
            p_w[n_elems++] =
                (((pat >> (8 - elem)) & 1u) != 0u) ? T_WIDE_UM : T_NARROW_UM;
        }
        if (kdx < 2)
        {
            p_w[n_elems++] = T_NARROW_UM; /* gap between characters */
        }
    }
    return n_elems;
}

static uint16_t level_at (uint32_t const * p_w, uint16_t n_elems, uint32_t x_um)
{
    uint32_t pos = T_LEAD_UM;
    uint16_t elem;

    for (elem = 0u; elem < n_elems; elem++)
    {
        if ((x_um >= pos) && (x_um < (pos + p_w[elem])))
        {
            return (0u == (elem % 2u)) ? (uint16_t)T_BLACK : (uint16_t)T_WHITE;
        }
        pos += p_w[elem];
    }
    return (uint16_t)T_WHITE;
}

#endif

/*** end of file ***/
