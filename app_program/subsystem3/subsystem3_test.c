/*
 *  subsystem3_test.c
 *  On-target tests for subsystem 3.  Built only with: make TEST=1
 *  Record results in testing/subsystem3/TEST_LOG.md.
 *  Decoding and tracking are tested on a PC (testing/subsystem3/host).
 */

#include "subsystem3.h"
#include "code39.h"
#include "../test_report.h"

#if APP_TEST

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

    TEST_SUMMARY("subsystem3");
}

#endif /* APP_TEST */
