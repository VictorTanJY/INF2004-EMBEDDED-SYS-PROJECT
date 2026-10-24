/*
 *  mission_task.c
 *  Subsystem 5 -- MissionTask (priority 8, core 1): the system statechart
 *  (Week 6 Section 5).  Owner: Buddy 5 (Ryan).
 *
 *  Waits on mbf_mission with a 100 ms timeout.  Every event is echoed to
 *  telemetry (mbf_telem), fed to mission_core, and every pass runs the
 *  supervision tick (self-test, calibration phases, timeouts).  The
 *  actions mission_core returns are carried out here: motion commands to
 *  mbf_motion, scan requests to mbf_scan, the safety reset request and
 *  telemetry records.
 */

#include "subsystem5.h"
#include "mission_core.h"
#include "../subsystem1/subsystem1.h"
#include "../subsystem2/subsystem2.h"
#include "../subsystem3/subsystem3.h"
#include "../subsystem4/subsystem4.h"

/* mission_core.c hard-codes these two APP_SRC values (pure module). */
typedef char s5_check_src_line[(APP_SRC_LINE == 3u) ? 1 : -1];
typedef char s5_check_src_imu[(APP_SRC_SAFETY_IMU == 4u) ? 1 : -1];

static mission_core_t g_mc;
static uint8_t        g_scan_seq;

static void run_actions(mission_out_t const * p_out)
{
    uint8_t i;

    for (i = 0u; i < p_out->n; i++)
    {
        mission_act_t const * p_a = &p_out->act[i];

        switch (p_a->kind)
        {
            case MISSION_ACT_MOTION:
                if (motion_post((uint8_t)MOTION_SRC_MISSION, p_a->seq, p_a->cmd, p_a->p0, p_a->p1) != E_OK)
                {
                    /* mbf_motion full: MotionTask stalled; its heartbeat catches it */
                    (void)comms_telem_post((uint8_t)COMMS_KIND_LOG, (uint8_t)APP_SRC_OBST_MISSION,
                                           70u, (int32_t)p_a->cmd);
                }
                break;

            case MISSION_ACT_SCAN:
                g_scan_seq++;
                (void)scan_request(p_a->cmd, g_scan_seq, p_a->p0);
                break;

            case MISSION_ACT_SAFETY_RESET:
                safety_request_reset();
                break;

            case MISSION_ACT_STATE:
                (void)comms_telem_post((uint8_t)COMMS_KIND_STATE, (uint8_t)APP_SRC_OBST_MISSION,
                                       (uint16_t)p_a->p0, 0);
                break;

            case MISSION_ACT_LOG:
                if (p_a->cmd == MISSION_LOG_SUMMARY)
                {
                    imu_snap_t imu;

                    imu_get_snap(&imu);
                    /* run summary: highest hump peak of the run (FR7) */
                    (void)comms_telem_post((uint8_t)COMMS_KIND_LOG, (uint8_t)APP_SRC_OBST_MISSION,
                                           MISSION_LOG_SUMMARY, (int32_t)imu.hump_max_mm);
                }
                else
                {
                    (void)comms_telem_post((uint8_t)COMMS_KIND_LOG, (uint8_t)APP_SRC_OBST_MISSION,
                                           p_a->cmd, (int32_t)(((uint32_t)(uint16_t)p_a->p0 << 16) |
                                                               (uint16_t)p_a->p1));
                }
                break;

            default:                        /* NEW_RUN: ImuTask follows snap.runs */
                break;
        }
    }
}

EXPORT void mission_task(INT stacd, void * p_exinf)
{
    mission_evt_t    ev;
    mission_out_t    out;
    mission_inputs_t in;
    motion_odom_t    odom;
    line_snap_t      line;
    imu_snap_t       imu;

    (void)stacd;
    (void)p_exinf;

    mission_core_init(&g_mc, (uint32_t)app_time_ms());
    g_mc.snap.t_ms = (uint32_t)app_time_ms();
    mission_put_snap(&g_mc.snap);

    for (;;)
    {
        INT      n = tk_rcv_mbf(g_mbf_mission, &ev, S5_MISSION_TICK_MS);
        uint32_t now_ms;

        safety_heartbeat(SAFETY_HB_MISSION);
        now_ms = (uint32_t)app_time_ms();

        out.n = 0u;
        if (n == (INT)sizeof(ev))
        {
            /* the laptop's event log: every event MissionTask decides on */
            (void)comms_telem_post((uint8_t)COMMS_KIND_EVENT, ev.src, (uint16_t)ev.type, ev.data);
            mission_core_event(&g_mc, &ev, now_ms, &out);
            run_actions(&out);
        }

        motion_get_odom(&odom);
        line_get_snap(&line);
        imu_get_snap(&imu);
        in.b_motion_hw_ok  = (odom.hw_ok != 0u);
        in.b_line_hw_ok    = ((line.flags & LINE_FLAG_HW_OK) != 0u);
        in.b_imu_ok        = (imu.ok != 0u);
        in.b_timer_checked = true;
        in.b_timer_ok      = safety_timer_tick_ok();
        in.b_on_line       = ((line.flags & LINE_FLAG_ON_LINE) != 0u) &&
                             ((line.flags & LINE_FLAG_CALIBRATED) != 0u);
        in.mbf_drops       = mission_post_drops();
        out.n = 0u;
        mission_core_tick(&g_mc, &in, now_ms, &out);
        run_actions(&out);

        g_mc.snap.t_ms = now_ms;
        mission_put_snap(&g_mc.snap);
    }
}
