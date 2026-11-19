/*
 *  mission_core.c
 *  Subsystem 5 -- system statechart (pure).  See mission_core.h.
 *  Owner: Buddy 5 (Ryan).
 *
 *  States and transitions follow Week 6 Section 5:
 *    BOOT -> IDLE (self-test passed) | FAULT
 *    IDLE -> CALIBRATING (calibrate) -> IDLE
 *    IDLE / MISSION_COMPLETE -> LINE_FOLLOW (start, only once calibrated)
 *    LINE_FOLLOW -> EXECUTE_NAV (junction with a latched barcode command)
 *                -> OBSTACLE_HANDLING -> AVOIDING -> LINE_RECOVERY ->
 * LINE_FOLLOW
 *                -> LINE_SEARCH (line lost / recovery failed) -> LINE_FOLLOW |
 * FAULT any active state -> MISSION_COMPLETE (stop); any state -> FAULT FAULT
 * -> BOOT only after an operator reset with every cause cleared.
 */

#include <stddef.h>
#include "mission_core.h"
#include "mission_cfg.h"
#include "../app_config.h"
#include "../subsystem4/imu_types.h" /* SAFETY_FLT_LIFT */
#include "../app_pack.h"

/* APP_SRC_* of the calibrating subsystems (app.h is kernel-side). */
#define SRC_LINE       3u
#define SRC_SAFETY_IMU 4u


static char const g_map_chars[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ-. $/+%";

/* ------------------------------------------------------------------ */
static void push (mission_core_out_t * p_out, uint8_t kind, uint8_t cmd,
                  uint8_t seq, int16_t arg0, int16_t arg1)
{
    if (p_out->n_acts < MISSION_ACT_MAX)
    {
        p_out->act[p_out->n_acts].kind = kind;
        p_out->act[p_out->n_acts].cmd  = cmd;
        p_out->act[p_out->n_acts].seq  = seq;
        p_out->act[p_out->n_acts].arg0 = arg0;
        p_out->act[p_out->n_acts].arg1 = arg1;
        p_out->n_acts++;
    }
}

static void motion (mission_core_t * p_mc, mission_core_out_t * p_out,
                    uint8_t cmd, int16_t arg0, int16_t arg1)
{
    uint8_t seq = 0u;

    if ((MOTION_CMD_MOVE == cmd) || (MOTION_CMD_TURN == cmd))
    {
        p_mc->motion_seq =
            motion_types_seq_next(p_mc->motion_seq); /* 5 bits (CR1) */
        seq             = p_mc->motion_seq;
        p_mc->wait_seq  = seq;
        p_mc->b_waiting = true;
    }
    push(p_out, MISSION_ACT_MOTION, cmd, seq, arg0, arg1);
}

static void set_auth (mission_core_t * p_mc, mission_core_out_t * p_out,
                      uint8_t src)
{
    motion(p_mc, p_out, MOTION_CMD_SET_AUTH, (int16_t)src, 0);
    p_mc->snap.auth = src;
}

static void scan (mission_core_out_t * p_out, uint8_t type, int16_t param)
{
    push(p_out, MISSION_ACT_SCAN, type, 0u, param, 0);
}

static void log_act (mission_core_out_t * p_out, uint8_t code, int16_t arg0,
                     int16_t arg1)
{
    push(p_out, MISSION_ACT_LOG, code, 0u, arg0, arg1);
}

static bool is_active (uint8_t state)
{
    return (state >= MISSION_ST_LINE_FOLLOW)
           && (state <= MISSION_ST_LINE_SEARCH);
}

static void update_speed_cap (mission_core_t * p_mc)
{
    uint16_t cap = p_mc->snap.cruise_mm_s;

    if (p_mc->b_slow && (cap > MISSION_SLOW_SPEED_MM_S))
    {
        cap = MISSION_SLOW_SPEED_MM_S;
    }
    if (p_mc->b_hump && (cap > MISSION_HUMP_SPEED_MM_S))
    {
        cap = MISSION_HUMP_SPEED_MM_S;
    }
    p_mc->snap.speed_cap_mm_s = cap;
}

/* ---- navigation and search steps --------------------------------------- */
typedef struct
{
    uint8_t cmd;
    int16_t arg0;
    int16_t arg1;
} step_t;

static uint8_t nav_steps (uint8_t nav, step_t steps[2])
{
    steps[0].cmd  = MOTION_CMD_MOVE;
    steps[0].arg0 = MISSION_NAV_CENTRE_MM;
    steps[0].arg1 = MISSION_NAV_SPEED_MM_S;
    steps[1].cmd  = MOTION_CMD_TURN;
    steps[1].arg1 = MISSION_TURN_RATE_DEG_S;
    switch (nav)
    {
        case MISSION_NAV_LEFT:
            steps[1].arg0 = 900;
            return 2u;
        case MISSION_NAV_RIGHT:
            steps[1].arg0 = -900;
            return 2u;
        case MISSION_NAV_UTURN:
            steps[1].arg0 = 1800;
            return 2u;
        default: /* STRAIGHT: just clear the crossing */
            steps[0].arg0 = MISSION_NAV_STRAIGHT_MM;
            return 1u;
    }
}

static void enter(mission_core_t * p_mc, uint8_t state, uint32_t now_ms,
                  mission_core_out_t * p_out);

static void nav_next (mission_core_t * p_mc, uint32_t now_ms,
                      mission_core_out_t * p_out)
{
    step_t        steps[2];
    uint8_t const count = nav_steps(p_mc->nav_exec, steps);

    if (p_mc->step < count)
    {
        motion(p_mc, p_out, steps[p_mc->step].cmd, steps[p_mc->step].arg0,
               steps[p_mc->step].arg1);
        p_mc->step++;
        p_mc->phase_ms = now_ms;
    }
    else
    {
        p_mc->nav_exec = MISSION_NAV_NONE;
        enter(p_mc, MISSION_ST_LINE_FOLLOW, now_ms, p_out);
    }
}

static void search_next (mission_core_t * p_mc, uint32_t now_ms,
                         mission_core_out_t * p_out)
{
    static int16_t const k_sweep[3] = {MISSION_SEARCH_ANGLE_DDEG,
                                       -2 * MISSION_SEARCH_ANGLE_DDEG,
                                       MISSION_SEARCH_ANGLE_DDEG};

    if (p_mc->step < 3u)
    {
        motion(p_mc, p_out, MOTION_CMD_TURN, k_sweep[p_mc->step],
               MISSION_SEARCH_RATE_DEG_S);
        p_mc->step++;
        p_mc->phase_ms = now_ms;
    }
    else
    {
        p_mc->snap.cause = MISSION_CAUSE_LINE_LOST;
        enter(p_mc, MISSION_ST_FAULT, now_ms, p_out);
    }
}

/* ---- state entry -------------------------------------------------------- */
static void enter (mission_core_t * p_mc, uint8_t state, uint32_t now_ms,
                   mission_core_out_t * p_out)
{
    p_mc->snap.state = state;
    p_mc->entry_ms   = now_ms;
    p_mc->phase_ms   = now_ms;
    p_mc->step       = 0u;
    p_mc->b_waiting  = false;
    push(p_out, MISSION_ACT_STATE, 0u, 0u, (int16_t)state, 0);

    switch (state)
    {
        case MISSION_ST_IDLE:
        case MISSION_ST_BOOT:
        case MISSION_ST_FAULT:
        case MISSION_ST_MISSION_COMPLETE:
            motion(p_mc, p_out, MOTION_CMD_STOP, 0, 0);
            set_auth(p_mc, p_out, MOTION_SRC_MISSION);
            scan(p_out, SCAN_REQ_STOP, 0);
            p_mc->snap.cal_phase = MISSION_CAL_PH_NONE;
            p_mc->b_slow         = false;
            p_mc->b_hump         = false;
            update_speed_cap(p_mc);
            if (MISSION_ST_MISSION_COMPLETE == state)
            {
                log_act(p_out, MISSION_LOG_SUMMARY, (int16_t)p_mc->snap.runs,
                        0);
            }
        break;

        case MISSION_ST_CALIBRATING:
            motion(p_mc, p_out, MOTION_CMD_STOP, 0, 0);
            set_auth(p_mc, p_out, MOTION_SRC_MISSION);
            p_mc->snap.cal_phase = MISSION_CAL_PH_STILL;
            p_mc->cal_done       = 0u;
            p_mc->cal_ok         = 0u;
        break;

        case MISSION_ST_LINE_FOLLOW:
            update_speed_cap(p_mc);
            set_auth(p_mc, p_out, MOTION_SRC_LINE);
            scan(p_out, SCAN_REQ_MONITOR, 0);
        break;

        case MISSION_ST_EXECUTE_NAV:
            p_mc->nav_exec = p_mc->snap.pending_nav; /* consume the latch */
            p_mc->snap.pending_nav  = MISSION_NAV_NONE;
            p_mc->snap.pending_char = '\0';
            set_auth(p_mc, p_out, MOTION_SRC_MISSION);
            nav_next(p_mc, now_ms, p_out);
        break;

        case MISSION_ST_OBSTACLE_HANDLING:
            motion(p_mc, p_out, MOTION_CMD_STOP, 0, 0);
            set_auth(p_mc, p_out, MOTION_SRC_AVOID);
            scan(p_out, SCAN_REQ_PROFILE, 0);
        break;

        case MISSION_ST_LINE_RECOVERY:
            scan(p_out, SCAN_REQ_RECOVER, 0);
        break;

        case MISSION_ST_LINE_SEARCH:
            motion(p_mc, p_out, MOTION_CMD_STOP, 0, 0);
            set_auth(p_mc, p_out, MOTION_SRC_MISSION);
            scan(p_out, SCAN_REQ_STOP, 0);
            search_next(p_mc, now_ms, p_out);
        break;

        default: /* AVOIDING: the request is sent by the caller */
        break;
    }
}

static void enter_fault (mission_core_t * p_mc, uint8_t cause, uint32_t bits,
                         uint32_t now_ms, mission_core_out_t * p_out)
{
    p_mc->snap.cause      = cause;
    p_mc->snap.fault_bits = bits;
    if (p_mc->snap.state != MISSION_ST_FAULT)
    {
        enter(p_mc, MISSION_ST_FAULT, now_ms, p_out);
    }
}

static void start_run (mission_core_t * p_mc, uint32_t now_ms,
                       mission_core_out_t * p_out)
{
    if ((p_mc->snap.calibrated & MISSION_CAL_ALL) != MISSION_CAL_ALL)
    {
        log_act(p_out, MISSION_LOG_NOT_CALIBRATED,
                (int16_t)p_mc->snap.calibrated, 0);
        return;
    }
    p_mc->snap.runs++;
    p_mc->snap.pending_nav  = MISSION_NAV_NONE;
    p_mc->snap.pending_char = '\0';
    push(p_out, MISSION_ACT_NEW_RUN, 0u, 0u, (int16_t)p_mc->snap.runs, 0);
    enter(p_mc, MISSION_ST_LINE_FOLLOW, now_ms, p_out);
}

/* ---- map helpers --------------------------------------------------------- */
bool mission_core_map_char_ok (char chr)
{
    size_t idx;

    for (idx = 0u; idx < (sizeof(g_map_chars) - 1u); idx++)
    {
        if (g_map_chars[idx] == chr)
        {
            return true;
        }
    }
    return false;
}

int32_t mission_core_map_pack (char const map[MISSION_NAV_COUNT])
{
    uint32_t packed = 0u;
    uint8_t  kdx;

    for (kdx = 0u; kdx < MISSION_NAV_COUNT; kdx++)
    {
        packed |= ((uint32_t)(uint8_t)map[kdx]) << (8u * kdx);
    }
    return (int32_t)packed;
}

bool mission_core_map_unpack (int32_t packed, char map[MISSION_NAV_COUNT])
{
    uint8_t kdx;
    uint8_t jdx;

    for (kdx = 0u; kdx < MISSION_NAV_COUNT; kdx++)
    {
        map[kdx] = (char)(((uint32_t)packed >> (8u * kdx)) & 0xFFu);
        if (!mission_core_map_char_ok(map[kdx]))
        {
            return false;
        }
        for (jdx = 0u; jdx < kdx; jdx++)
        {
            if (map[jdx] == map[kdx])
            {
                return false; /* one character, one movement */
            }
        }
    }
    return true;
}

uint8_t mission_core_nav_for (mission_types_snap_t const * p_snap, char chr)
{
    uint8_t kdx;

    for (kdx = 0u; kdx < MISSION_NAV_COUNT; kdx++)
    {
        if ((p_snap->nav_map[kdx] != '\0') && (p_snap->nav_map[kdx] == chr))
        {
            return kdx;
        }
    }
    return MISSION_NAV_NONE;
}

/* ---- public ----------------------------------------------------------------
 */
void mission_core_init (mission_core_t * p_mc, uint32_t now_ms)
{
    uint8_t * p_b = (uint8_t *)p_mc;
    size_t    idx;

    for (idx = 0u; idx < sizeof(*p_mc); idx++)
    {
        p_b[idx] = 0u;
    }
    p_mc->snap.state          = MISSION_ST_BOOT;
    p_mc->snap.auth           = MOTION_SRC_MISSION;
    p_mc->snap.cruise_mm_s    = MISSION_CRUISE_MM_S;
    p_mc->snap.speed_cap_mm_s = MISSION_CRUISE_MM_S;
    p_mc->snap.slow_mm        = MISSION_SLOW_MM;
    p_mc->snap.stop_mm        = MISSION_STOP_MM;
    p_mc->snap.lost_mm        = MISSION_LOST_MM;
    p_mc->snap.junction_mm    = MISSION_JUNCTION_MM;
    p_mc->snap.pending_nav    = MISSION_NAV_NONE;
    p_mc->nav_exec            = MISSION_NAV_NONE;
    p_mc->entry_ms            = now_ms;
    p_mc->phase_ms            = now_ms;
}

static void apply_param (mission_core_t * p_mc, uint16_t param_id,
                         int32_t value, mission_core_out_t * p_out)
{
    switch (param_id)
    {
        case 1u:
            p_mc->snap.cruise_mm_s = (uint16_t)value;
        break;
        case 2u:
            p_mc->snap.slow_mm = (uint16_t)(value * 10);
        break;
        case 3u:
            p_mc->snap.stop_mm = (uint16_t)(value * 10);
        break;
        case 4u:
            p_mc->snap.lost_mm = (uint16_t)value;
        break;
        case 5u:
            p_mc->snap.junction_mm = (uint16_t)value;
        break;
#if APP_BENCH
        /* bring-up only (app_config.h): drive the hardware from the laptop */
        case 101u:
            motion(p_mc, p_out, MOTION_CMD_MOVE, (int16_t)value, 200);
            return;
        case 102u:
            motion(p_mc, p_out, MOTION_CMD_TURN, (int16_t)value, 90);
            return;
        case 103u:
            motion(p_mc, p_out, MOTION_CMD_MOVE, 1000, (int16_t)value);
            return;
#endif
        default:
            return;
    }
    update_speed_cap(p_mc);
    log_act(p_out, MISSION_LOG_PARAM_SET, (int16_t)param_id, (int16_t)value);
}

static bool seq_matches (mission_core_t *            p_mc,
                         mission_types_evt_t const * p_ev)
{
    if (p_mc->b_waiting && ((uint8_t)p_ev->arg == p_mc->wait_seq)
        && ((int32_t)MOTION_SRC_MISSION == p_ev->data))
    {
        p_mc->b_waiting = false;
        return true;
    }
    return false;
}

void mission_core_event (mission_core_t *            p_mc,
                         mission_types_evt_t const * p_ev, uint32_t now_ms,
                         mission_core_out_t * p_out)
{
    uint8_t const state = p_mc->snap.state;

    switch (p_ev->type)
    {
        /* ---- faults and resets (any state) ---- */
        case MISSION_EV_FAULT:
            if (p_ev->data != 0)
            {
                /* 14 Nov: lifting the robot ends the run in the demo; it is
                   braked and latched like any fault, and the run summary
                   (highest hump) is published as on STOP. */
                if (is_active(state)
                    && (((uint32_t)p_ev->data & SAFETY_FLT_LIFT) != 0u))
                {
                    log_act(p_out, MISSION_LOG_SUMMARY,
                            (int16_t)p_mc->snap.runs, 0);
                }
                enter_fault(p_mc, MISSION_CAUSE_SAFETY, (uint32_t)p_ev->data,
                            now_ms, p_out);
            }
        break;

        case MISSION_EV_BTN_RESET:
        case MISSION_EV_REMOTE_RESET:
            if (MISSION_ST_FAULT == state)
            {
                push(p_out, MISSION_ACT_SAFETY_RESET, 0u, 0u, 0, 0);
            }
            else if (!is_active(state) && (state != MISSION_ST_CALIBRATING))
            {
                enter(p_mc, MISSION_ST_BOOT, now_ms,
                      p_out); /* re-run the self-test */
            }
            else
            {
                /* while running, STOP first */
            }
        break;

        case MISSION_EV_SAFETY_OK:
            if (MISSION_ST_FAULT == state)
            {
                motion(p_mc, p_out, MOTION_CMD_CLEAR_ESTOP, 0, 0);
                p_mc->snap.cause      = MISSION_CAUSE_NONE;
                p_mc->snap.fault_bits = 0u;
                enter(p_mc, MISSION_ST_BOOT, now_ms, p_out);
            }
        break;

        /* ---- operator ---- */
        case MISSION_EV_BTN_START_STOP:
            if (is_active(state))
            {
                enter(p_mc, MISSION_ST_MISSION_COMPLETE, now_ms, p_out);
            }
            else if ((MISSION_ST_IDLE == state)
                     || (MISSION_ST_MISSION_COMPLETE == state))
            {
                start_run(p_mc, now_ms, p_out);
            }
            else if (MISSION_ST_CALIBRATING == state)
            {
                enter(p_mc, MISSION_ST_IDLE, now_ms, p_out); /* abort */
            }
            else
            {
                /* BOOT / FAULT: ignored */
            }
        break;

        case MISSION_EV_REMOTE_START:
            if ((MISSION_ST_IDLE == state)
                || (MISSION_ST_MISSION_COMPLETE == state))
            {
                start_run(p_mc, now_ms, p_out);
            }
        break;

        case MISSION_EV_REMOTE_STOP:
            if (is_active(state))
            {
                enter(p_mc, MISSION_ST_MISSION_COMPLETE, now_ms, p_out);
            }
            else if (MISSION_ST_CALIBRATING == state)
            {
                enter(p_mc, MISSION_ST_IDLE, now_ms, p_out);
            }
            else
            {
                /* already stopped */
            }
        break;

        case MISSION_EV_BTN_CALIBRATE:
            if ((MISSION_ST_IDLE == state)
                || (MISSION_ST_MISSION_COMPLETE == state))
            {
                enter(p_mc, MISSION_ST_CALIBRATING, now_ms, p_out);
            }
        break;

        case MISSION_EV_REMOTE_SET:
            if (MISSION_ST_IDLE == state)
            {
                apply_param(p_mc, p_ev->arg, p_ev->data, p_out);
            }
        break;

        case MISSION_EV_REMOTE_MAP:
            if (!is_active(state) && (state != MISSION_ST_CALIBRATING))
            {
                char map[MISSION_NAV_COUNT];

                if (mission_core_map_unpack(p_ev->data, map))
                {
                    uint8_t kdx;

                    for (kdx = 0u; kdx < MISSION_NAV_COUNT; kdx++)
                    {
                        p_mc->snap.nav_map[kdx] = map[kdx];
                    }
                    log_act(p_out, MISSION_LOG_MAP_SET,
                            (int16_t)((uint32_t)p_ev->data & 0xFFFFu),
                            (int16_t)((uint32_t)p_ev->data >> 16));
                }
                else
                {
                    log_act(p_out, MISSION_LOG_MAP_BAD, 0, 0);
                }
            }
        break;

        /* ---- calibration ---- */
        case MISSION_EV_CAL_DONE:
            if ((MISSION_ST_CALIBRATING == state)
                && (MISSION_CAL_PH_FINISH == p_mc->snap.cal_phase))
            {
                uint8_t const bit =
                    (SRC_LINE == p_ev->arg)
                        ? MISSION_CAL_IR
                        : ((SRC_SAFETY_IMU == p_ev->arg) ? MISSION_CAL_IMU
                                                         : 0u);

                p_mc->cal_done |= bit;
                if (p_ev->data != 0)
                {
                    p_mc->cal_ok |= bit;
                }
            }
        break;

        /* ---- motion ---- */
        case MISSION_EV_MOTION_DONE:
        case MISSION_EV_MOTION_FAILED:
            if (seq_matches(p_mc, p_ev))
            {
                bool const b_ok = (MISSION_EV_MOTION_DONE == p_ev->type);

                if (MISSION_ST_CALIBRATING == state)
                {
                    p_mc->snap.cal_phase =
                        MISSION_CAL_PH_FINISH; /* spin over */
                    p_mc->phase_ms = now_ms;
                }
                else if (MISSION_ST_EXECUTE_NAV == state)
                {
                    if (b_ok)
                    {
                        nav_next(p_mc, now_ms, p_out);
                    }
                    else
                    {
                        enter(p_mc, MISSION_ST_LINE_SEARCH, now_ms, p_out);
                    }
                }
                else if (MISSION_ST_LINE_SEARCH == state)
                {
                    search_next(p_mc, now_ms, p_out);
                }
                else
                {
                    /* stale completion: ignored */
                }
            }
        break;

        /* ---- line ---- */
        case MISSION_EV_BARCODE:
            if (is_active(state))
            {
                char const    chr = (char)p_ev->data;
                uint8_t const nav = mission_core_nav_for(&p_mc->snap, chr);

                if (nav != MISSION_NAV_NONE)
                {
                    /* latched until the next junction (FR4) */
                    p_mc->snap.pending_nav  = nav;
                    p_mc->snap.pending_char = chr;
                    log_act(p_out, MISSION_LOG_NAV_LATCHED, (int16_t)nav,
                            (int16_t)chr);
                }
                else
                {
                    log_act(p_out, MISSION_LOG_NAV_UNMAPPED, 0,
                            (int16_t)chr); /* reported, never acted on */
                }
            }
        break;

        case MISSION_EV_JUNCTION:
            if ((MISSION_ST_LINE_FOLLOW == state)
                && (p_mc->snap.pending_nav != MISSION_NAV_NONE))
            {
                enter(p_mc, MISSION_ST_EXECUTE_NAV, now_ms, p_out);
            }
        break;

        case MISSION_EV_LINE_LOST:
            if (MISSION_ST_LINE_FOLLOW == state)
            {
                enter(p_mc, MISSION_ST_LINE_SEARCH, now_ms, p_out);
            }
        break;

        case MISSION_EV_LINE_FOUND:
            if (MISSION_ST_LINE_SEARCH == state)
            {
                enter(p_mc, MISSION_ST_LINE_FOLLOW, now_ms, p_out);
            }
        break;

        /* ---- obstacle ---- */
        case MISSION_EV_SLOW_DOWN:
            p_mc->b_slow = (p_ev->data >= 0);
            update_speed_cap(p_mc);
        break;

        case MISSION_EV_OBSTACLE:
            if (MISSION_ST_LINE_FOLLOW == state)
            {
                enter(p_mc, MISSION_ST_OBSTACLE_HANDLING, now_ms, p_out);
            }
        break;

        case MISSION_EV_PROFILE_DONE:
        case MISSION_EV_CONTINUE:
            if (MISSION_ST_OBSTACLE_HANDLING == state)
            {
                uint8_t const plan = (MISSION_EV_CONTINUE == p_ev->type)
                                         ? OBST_PLAN_CONTINUE
                                         : (uint8_t)p_ev->arg;

                if ((OBST_PLAN_LEFT == plan) || (OBST_PLAN_RIGHT == plan))
                {
                    enter(p_mc, MISSION_ST_AVOIDING, now_ms, p_out);
                    scan(p_out,
                         (OBST_PLAN_LEFT == plan) ? SCAN_REQ_BYPASS_LEFT
                                                  : SCAN_REQ_BYPASS_RIGHT,
                         0);
                }
                else
                {
                    p_mc->b_slow = false;
                    enter(p_mc, MISSION_ST_LINE_FOLLOW, now_ms, p_out);
                }
            }
        break;

        case MISSION_EV_BYPASS_DONE:
            if (MISSION_ST_AVOIDING == state)
            {
                enter(p_mc, MISSION_ST_LINE_RECOVERY, now_ms, p_out);
            }
        break;

        case MISSION_EV_RECOVERY_DONE:
            if (MISSION_ST_LINE_RECOVERY == state)
            {
                p_mc->b_slow = false;
                enter(p_mc, MISSION_ST_LINE_FOLLOW, now_ms, p_out);
            }
        break;

        case MISSION_EV_RECOVERY_FAILED:
            if ((MISSION_ST_LINE_RECOVERY == state)
                || (MISSION_ST_AVOIDING == state))
            {
                enter(p_mc, MISSION_ST_LINE_SEARCH, now_ms, p_out);
            }
        break;

        case MISSION_EV_BLOCKED_TIMEOUT:
            if (MISSION_ST_OBSTACLE_HANDLING == state)
            {
                enter_fault(p_mc, MISSION_CAUSE_BLOCKED, 0u, now_ms, p_out);
            }
        break;

        /* ---- terrain ---- */
        case MISSION_EV_HUMP_START:
            p_mc->b_hump = true;
            update_speed_cap(p_mc);
        break;

        case MISSION_EV_HUMP_END:
            p_mc->b_hump = false;
            update_speed_cap(p_mc);
        break;

        default: /* MOTION_CLASS, NONE: telemetry only */
        break;
    }
}

void mission_core_tick (mission_core_t *              p_mc,
                        mission_core_inputs_t const * p_in, uint32_t now_ms,
                        mission_core_out_t * p_out)
{
    uint8_t const  state     = p_mc->snap.state;
    uint32_t const age       = now_ms - p_mc->entry_ms;
    uint32_t const phase_age = now_ms - p_mc->phase_ms;

    /* A dropped event means a decision may have been missed (Week 6 8.3). */
    if (p_in->mbf_drops != p_mc->last_drops)
    {
        p_mc->last_drops = p_in->mbf_drops;
        log_act(p_out, MISSION_LOG_MBF_DROP, (int16_t)p_in->mbf_drops, 0);
        if (is_active(state))
        {
            enter_fault(p_mc, MISSION_CAUSE_MBF_OVERFLOW, 0u, now_ms, p_out);
            return;
        }
    }

    switch (state)
    {
        case MISSION_ST_BOOT:
        {
            uint32_t failed = 0u;

            failed |= p_in->b_motion_hw_ok ? 0u : 0x1u;
            failed |= p_in->b_line_hw_ok ? 0u : 0x2u;
            failed |= p_in->b_imu_ok ? 0u : 0x4u;
            failed |= (p_in->b_timer_checked && p_in->b_timer_ok) ? 0u : 0x8u;
            failed &= MISSION_SELFTEST_MASK;
            if (0u == failed)
            {
                enter(p_mc, MISSION_ST_IDLE, now_ms, p_out);
            }
            else if (age >= MISSION_BOOT_TIMEOUT_MS)
            {
                log_act(p_out, MISSION_LOG_CAL_RESULT,
                        (int16_t)(0x100u | failed), 0);
                enter_fault(p_mc, MISSION_CAUSE_SELFTEST,
                            0x0400u | (failed << 16), now_ms, p_out);
            }
            else
            {
                /* still waiting for the subsystems */
            }
        }
        break;

        case MISSION_ST_CALIBRATING:
            if ((MISSION_CAL_PH_STILL == p_mc->snap.cal_phase)
                && (phase_age >= MISSION_CAL_STILL_MS))
            {
                p_mc->snap.cal_phase = MISSION_CAL_PH_SPIN;
                p_mc->phase_ms       = now_ms;
                motion(p_mc, p_out, MOTION_CMD_TURN, 3600,
                       MISSION_CAL_SPIN_RATE);
            }
            else if ((MISSION_CAL_PH_FINISH == p_mc->snap.cal_phase)
                     && ((MISSION_CAL_ALL == p_mc->cal_done)
                         || (phase_age >= MISSION_CAL_DONE_TIMEOUT_MS)))
            {
                p_mc->snap.calibrated = p_mc->cal_ok;
                log_act(p_out, MISSION_LOG_CAL_RESULT, (int16_t)p_mc->cal_ok,
                        0);
                enter(p_mc, MISSION_ST_IDLE, now_ms, p_out);
            }
            else
            {
                /* spinning, or waiting for the answers */
            }
        break;

        case MISSION_ST_EXECUTE_NAV:
            if (age >= MISSION_NAV_TIMEOUT_MS)
            {
                enter(p_mc, MISSION_ST_LINE_SEARCH, now_ms, p_out);
            }
        break;

        case MISSION_ST_OBSTACLE_HANDLING:
            if (age >= MISSION_OBST_TIMEOUT_MS)
            {
                enter_fault(p_mc, MISSION_CAUSE_TIMEOUT, 0u, now_ms, p_out);
            }
        break;

        case MISSION_ST_AVOIDING:
            if (age >= MISSION_AVOID_TIMEOUT_MS)
            {
                enter(p_mc, MISSION_ST_LINE_SEARCH, now_ms, p_out);
            }
        break;

        case MISSION_ST_LINE_RECOVERY:
            if (age >= MISSION_RECOVER_TIMEOUT_MS)
            {
                enter(p_mc, MISSION_ST_LINE_SEARCH, now_ms, p_out);
            }
        break;

        case MISSION_ST_LINE_SEARCH:
            /* the line sensors may see the line without a LOST/FOUND pair */
            if (p_in->b_on_line && (age >= 100u))
            {
                enter(p_mc, MISSION_ST_LINE_FOLLOW, now_ms, p_out);
            }
        break;

        default:
        break;
    }
}

/* ---------------------------------------------------------------------
 *  Change Requirement 1: packed messages and snapshot (see mission_core.h)
 * --------------------------------------------------------------------- */
#define EVT_TYPE_SHIFT 0u
#define EVT_TYPE_BITS  5u
#define EVT_SRC_SHIFT  5u
#define EVT_SRC_BITS   3u
#define EVT_A_SHIFT    8u
#define EVT_DATA_SHIFT 15u
#define EVT_SYM_BITS   6u

uint32_t mission_core_evt_pack (mission_types_evt_t const * p_ev)
{
    uint32_t word = 0u;

    word = app_pack_set(word, EVT_TYPE_SHIFT, EVT_TYPE_BITS,
                        app_pack_sat_u(p_ev->type, EVT_TYPE_BITS));
    word = app_pack_set(word, EVT_SRC_SHIFT, EVT_SRC_BITS,
                        app_pack_sat_u(p_ev->src, EVT_SRC_BITS));
    if (MISSION_EV_REMOTE_MAP == p_ev->type)
    {
        uint32_t const packed = (uint32_t)p_ev->data;
        uint32_t       kdx;

        for (kdx = 0u; kdx < MISSION_NAV_COUNT; kdx++)
        {
            word = app_pack_set(
                word, (uint8_t)(EVT_A_SHIFT + (kdx * EVT_SYM_BITS)),
                EVT_SYM_BITS,
                app_pack_sym_index((char)(uint8_t)(packed >> (8u * kdx))));
        }
    }
    else
    {
        word = app_pack_set(word, EVT_A_SHIFT, MISSION_EVT_A_BITS,
                            app_pack_sat_u(p_ev->arg, MISSION_EVT_A_BITS));
        word = app_pack_set(word, EVT_DATA_SHIFT, MISSION_EVT_DATA_BITS,
                            app_pack_off_s(p_ev->data, MISSION_EVT_DATA_BITS));
    }
    return word;
}

void mission_core_evt_unpack (uint32_t word, mission_types_evt_t * p_ev)
{
    p_ev->type = (uint8_t)app_pack_get(word, EVT_TYPE_SHIFT, EVT_TYPE_BITS);
    p_ev->src  = (uint8_t)app_pack_get(word, EVT_SRC_SHIFT, EVT_SRC_BITS);
    if (MISSION_EV_REMOTE_MAP == p_ev->type)
    {
        uint32_t packed = 0u;
        uint32_t kdx;

        for (kdx = 0u; kdx < MISSION_NAV_COUNT; kdx++)
        {
            uint8_t const sym = (uint8_t)app_pack_get(
                word, (uint8_t)(EVT_A_SHIFT + (kdx * EVT_SYM_BITS)),
                EVT_SYM_BITS);

            packed |= ((uint32_t)(uint8_t)app_pack_sym_char(sym)) << (8u * kdx);
        }
        p_ev->arg  = 0u;
        p_ev->data = (int32_t)packed;
    }
    else
    {
        p_ev->arg =
            (uint16_t)app_pack_get(word, EVT_A_SHIFT, MISSION_EVT_A_BITS);
        p_ev->data = app_pack_unoff_s(
            app_pack_get(word, EVT_DATA_SHIFT, MISSION_EVT_DATA_BITS),
            MISSION_EVT_DATA_BITS);
    }
}

#define REQ_TYPE_BITS  3u
#define REQ_SEQ_BITS   5u
#define REQ_PARAM_BITS 8u

uint16_t mission_core_scan_req_pack (mission_types_scan_req_t const * p_req)
{
    uint32_t word = 0u;

    word = app_pack_set(word, 0u, REQ_TYPE_BITS,
                        app_pack_sat_u(p_req->type, REQ_TYPE_BITS));
    word = app_pack_set(word, REQ_TYPE_BITS, REQ_SEQ_BITS,
                        (uint32_t)p_req->seq & app_pack_mask(REQ_SEQ_BITS));
    word = app_pack_set(
        word, REQ_TYPE_BITS + REQ_SEQ_BITS, REQ_PARAM_BITS,
        app_pack_sat_u((p_req->param < 0) ? 0u : (uint32_t)p_req->param,
                       REQ_PARAM_BITS));
    return (uint16_t)word;
}

void mission_core_scan_req_unpack (uint16_t                   word,
                                   mission_types_scan_req_t * p_req)
{
    p_req->type  = (uint8_t)app_pack_get(word, 0u, REQ_TYPE_BITS);
    p_req->seq   = (uint8_t)app_pack_get(word, REQ_TYPE_BITS, REQ_SEQ_BITS);
    p_req->param = (int16_t)app_pack_get(word, REQ_TYPE_BITS + REQ_SEQ_BITS,
                                         REQ_PARAM_BITS);
}

/* Data representation of IF-10 (mission part). */
static app_pack_field_t const g_mission_layout[] = {
    PK_FIELD(mission_types_snap_t, t_ms, 32u, PK_W),
    PK_FIELD(mission_types_snap_t, fault_bits, 20u,
             PK_U), /* SAFETY_FLT_* (16) + self-test mask << 16 */
    PK_FIELD(mission_types_snap_t, cruise_mm_s, 9u, PK_U), /* <= 450 */
    PK_FIELD(mission_types_snap_t, speed_cap_mm_s, 9u, PK_U),
    PK_FIELD(mission_types_snap_t, slow_mm, 10u,
             PK_U), /* <= 600 (slow_cm 25..60) */
    PK_FIELD(mission_types_snap_t, stop_mm, 9u,
             PK_U), /* <= 400 (stop_cm 15..40)   */
    PK_FIELD(mission_types_snap_t, lost_mm, 8u, PK_U),     /* <= 150 */
    PK_FIELD(mission_types_snap_t, junction_mm, 6u, PK_U), /* <= 40 */
    PK_FIELD(mission_types_snap_t, runs, 16u, PK_W),       /* counter */
    PK_FIELD(mission_types_snap_t, state, 4u, PK_U),       /* 11 states */
    PK_FIELD(mission_types_snap_t, auth, 2u, PK_U),        /* 3 sources */
    PK_FIELD(mission_types_snap_t, pending_nav, 3u,
             PK_UN), /* 0..3, NONE (0xFF) */
    PK_FIELD(mission_types_snap_t, pending_char, 6u,
             PK_SYM), /* Code 39 symbol or none    */
    PK_FIELD(mission_types_snap_t, nav_map[0], 6u, PK_SYM),
    PK_FIELD(mission_types_snap_t, nav_map[1], 6u, PK_SYM),
    PK_FIELD(mission_types_snap_t, nav_map[2], 6u, PK_SYM),
    PK_FIELD(mission_types_snap_t, nav_map[3], 6u, PK_SYM),
    PK_FIELD(mission_types_snap_t, calibrated, 2u,
             PK_U), /* MISSION_CAL_IR | _IMU */
    PK_FIELD(mission_types_snap_t, cal_phase, 2u, PK_U), /* 4 phases */
    PK_FIELD(mission_types_snap_t, cause, 3u, PK_U)      /* 7 causes */
};

#define MISSION_FIELDS \
    ((uint16_t)(sizeof(g_mission_layout) / sizeof(g_mission_layout[0])))

void mission_core_snap_pack (mission_types_snap_t const * p_snap,
                             uint32_t *                   p_words)
{
    app_pack_pack_rec(g_mission_layout, MISSION_FIELDS, p_snap, p_words,
                      MISSION_SNAP_WORDS);
}

void mission_core_snap_unpack (uint32_t const *       p_words,
                               mission_types_snap_t * p_snap)
{
    p_snap->reserved = 0u;
    app_pack_unpack_rec(g_mission_layout, MISSION_FIELDS, p_words, p_snap);
}

uint32_t mission_core_snap_layout_bits (void)
{
    return app_pack_layout_bits(g_mission_layout, MISSION_FIELDS);
}

/*** end of file ***/
