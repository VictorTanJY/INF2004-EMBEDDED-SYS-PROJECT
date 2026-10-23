/*
 *  subsystem3_test.c
 *  On-target tests for subsystem 3.  Built only with: make TEST=1
 *  Record results in testing/subsystem3/TEST_LOG.md.
 *
 *  Shared-base checks only; the owner adds the line and barcode tests.
 */

#include "subsystem3.h"
#include "../test_report.h"

#if APP_TEST

EXPORT void subsystem3_test(void)
{
    line_snap_t snap;

    TEST_INIT();

    snap.err = -321;
    line_put_snap(&snap);
    snap.err = 0;
    line_get_snap(&snap);
    TEST_CHECK("S3-U01", snap.err == -321, "line snapshot round trip");

    TEST_SUMMARY("subsystem3");
}

#endif /* APP_TEST */
