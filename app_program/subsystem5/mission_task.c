/*
 *  mission_task_thread.c
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
#include "../app_types.h"
#include "../subsystem1/subsystem1.h"
#include "../subsystem2/subsystem2.h"
#include "../subsystem3/subsystem3.h"
#include "../subsystem4/subsystem4.h"

/* mission_core.c hard-codes these two APP_SRC values (pure module). */
typedef uint8_t s5_check_src_line_t[(3u == APP_SRC_LINE) ? 1 : -1];
typedef uint8_t s5_check_src_imu_t[(4u == APP_SRC_SAFETY_IMU) ? 1 : -1];

static mission_core_t g_mc;
static uint8_t        g_scan_seq;

/* One motion action through the named API (FR5); SET_AUTH and
   CLEAR_ESTOP have no named form and use subsystem2_post. */
static ER mission_motion (mission_core_act_t const * p_a)
{
    uint8_t const  src    = MOTION_SRC_MISSION;
    uint16_t const amount = (uint16_t)app_types_abs_i32(p_a->arg0);
    ER             err;

    switch (p_a->cmd)
    {
        case MOTION_CMD_MOVE:
            err = (p_a->arg0 >= 0)
                      ? motion_api_move_forward(src, p_a->seq, amount,
                                                (uint16_t)p_a->arg1)
                      : motion_api_move_backward(src, p_a->seq, amount,
                                                 (uint16_t)p_a->arg1);
        break;

        case MOTION_CMD_TURN:
            err = (p_a->arg0 >= 0)
                      ? motion_api_turn_left(src, p_a->seq, amount,
                                             (uint16_t)p_a->arg1)
                      : motion_api_turn_right(src, p_a->seq, amount,
                                              (uint16_t)p_a->arg1);
        break;

        case MOTION_CMD_STOP:
            err = motion_api_stop(src, p_a->seq);
        break;

        case MOTION_CMD_DRIVE:
            err = motion_api_drive(src, p_a->seq, p_a->arg0, p_a->arg1);
        break;

        default:
            err =
                subsystem2_post(src, p_a->seq, p_a->cmd, p_a->arg0, p_a->arg1);
        break;
    }
    return err;
}

static void run_actions (mission_core_out_t const * p_out)
{
    uint8_t idx;

    for (idx = 0u; idx < p_out->n_acts; idx++)
    {
        mission_core_act_t const * p_a = &p_out->act[idx];

        switch (p_a->kind)
        {
            case MISSION_ACT_MOTION:
                if (mission_motion(p_a) != E_OK)
                {
                    /* mbf_motion full: MotionTask stalled; its heartbeat
                     * catches it */
                    (void)subsystem1_telem_post(COMMS_KIND_LOG,
                                                (uint8_t)APP_SRC_OBST_MISSION,
                                                70u, (int32_t)p_a->cmd);
                }
            break;

            case MISSION_ACT_SCAN:
                g_scan_seq++;
                (void)subsystem5_scan_request(p_a->cmd, g_scan_seq, p_a->arg0);
            break;

            case MISSION_ACT_SAFETY_RESET:
                subsystem4_request_reset();
            break;

            case MISSION_ACT_STATE:
                (void)subsystem1_telem_post(COMMS_KIND_STATE,
                                            (uint8_t)APP_SRC_OBST_MISSION,
                                            (uint16_t)p_a->arg0, 0);
            break;

            case MISSION_ACT_LOG:
                if (MISSION_LOG_SUMMARY == p_a->cmd)
                {
                    imu_types_snap_t imu;

                    subsystem4_get_imu_snap(&imu);
                    /* run summary: highest hump peak of the run (FR7) */
                    (void)subsystem1_telem_post(
                        COMMS_KIND_LOG, (uint8_t)APP_SRC_OBST_MISSION,
                        MISSION_LOG_SUMMARY, (int32_t)imu.hump_max_mm);
                }
                else
                {
                    (void)subsystem1_telem_post(
                        COMMS_KIND_LOG, (uint8_t)APP_SRC_OBST_MISSION, p_a->cmd,
                        (int32_t)(((uint32_t)(uint16_t)p_a->arg0 << 16)
                                  | (uint16_t)p_a->arg1));
                }
            break;

            default: /* NEW_RUN: ImuTask follows snap.runs */
            break;
        }
    }
}

EXPORT void mission_task_thread (INT stacd, void * p_exinf)
{
    mission_types_evt_t   event;
    uint32_t              ev_word; /* mbf_mission: one packed word (CR1) */
    mission_core_out_t    out;
    mission_core_inputs_t inputs;
    motion_types_odom_t   odom;
    line_types_snap_t     line;
    imu_types_snap_t      imu;

    (void)stacd;
    (void)p_exinf;

    mission_core_init(&g_mc, (uint32_t)app_time_ms());
    g_mc.snap.t_ms = (uint32_t)app_time_ms();
    subsystem5_put_mission_snap(&g_mc.snap);

    for (;;)
    {
        INT msg_len = tk_rcv_mbf(g_mbf_mission, &ev_word, S5_MISSION_TICK_MS);
        uint32_t now_ms;

        subsystem4_heartbeat(SAFETY_HB_MISSION);
        now_ms = (uint32_t)app_time_ms();

        out.n_acts = 0u;
        if ((INT)sizeof(ev_word) == msg_len)
        {
            mission_core_evt_unpack(ev_word, &event);
            /* the laptop's event log: every event MissionTask decides on */
            (void)subsystem1_telem_post(COMMS_KIND_EVENT, event.src,
                                        (uint16_t)event.type, event.data);
            mission_core_event(&g_mc, &event, now_ms, &out);
            run_actions(&out);
        }

        subsystem2_get_odom(&odom);
        subsystem3_get_snap(&line);
        subsystem4_get_imu_snap(&imu);
        inputs.b_motion_hw_ok  = (odom.hw_ok != 0u);
        inputs.b_line_hw_ok    = ((line.flags & LINE_FLAG_HW_OK) != 0u);
        inputs.b_imu_ok        = (imu.b_ok != 0u);
        inputs.b_timer_checked = true;
        inputs.b_timer_ok      = safety_task_timer_tick_ok();
        inputs.b_on_line       = ((line.flags & LINE_FLAG_ON_LINE) != 0u)
                                 && ((line.flags & LINE_FLAG_CALIBRATED) != 0u);
        inputs.mbf_drops       = subsystem5_mission_drops();
        out.n_acts             = 0u;
        mission_core_tick(&g_mc, &inputs, now_ms, &out);
        run_actions(&out);

        g_mc.snap.t_ms = now_ms;
        subsystem5_put_mission_snap(&g_mc.snap);
    }
}

/*** end of file ***/
