/*
 *  subsystem4_test.c
 *  On-target tests for subsystem 4.  Built only with: make TEST=1
 *  Record results in testing/subsystem4/TEST_LOG.md.
 *
 *  Shared-base checks only; the owner adds the IMU and safety tests.
 */

#include "subsystem4.h"
#include "../test_report.h"

#if APP_TEST

EXPORT void subsystem4_test(void)
{
    T_RFLG      rflg;
    imu_snap_t  snap;
    uint32_t    before;

    TEST_INIT();

    TEST_CHECK("S4-U01", tk_ref_flg(g_flg_safety, &rflg) == E_OK,
               "flg_safety exists");

    before = safety_heartbeat_count(SAFETY_HB_IMU);
    safety_heartbeat(SAFETY_HB_IMU);
    TEST_CHECK("S4-U02", safety_heartbeat_count(SAFETY_HB_IMU) == (before + 1u),
               "heartbeat counter increments");

    snap.pitch_cdeg = -450;
    imu_put_snap(&snap);
    snap.pitch_cdeg = 0;
    imu_get_snap(&snap);
    TEST_CHECK("S4-U03", snap.pitch_cdeg == -450, "IMU snapshot round trip");

    TEST_SUMMARY("subsystem4");
}

#endif /* APP_TEST */
