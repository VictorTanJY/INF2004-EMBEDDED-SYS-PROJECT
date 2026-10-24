/*
 *  motion_core.c
 *  Subsystem 2 -- MotionTask controller (pure).  See motion_core.h.
 *  Owner: Buddy 2 (Jedrek).
 */

#include <stddef.h>
#include "motion_core.h"
#include "motion_cfg.h"
#include "../app_types.h"
#include "../app_math.h"

#define PI_E6               3141593         /* pi x 10^6 */
#define DT_DEFAULT_MS       10u
#define YAW_FILTER_DIV      2

/* ------------------------------------------------------------------ */
static void push_event(motion_core_t * p_mc, uint8_t code, uint8_t seq, uint8_t src)
{
    if (p_mc->ev_count < MOTION_EV_QUEUE_LEN)
    {
        p_mc->ev[p_mc->ev_count].code = code;
        p_mc->ev[p_mc->ev_count].seq  = seq;
        p_mc->ev[p_mc->ev_count].src  = src;
        p_mc->ev_count++;
    }
}

static void reset_pid(motion_core_t * p_mc)
{
    p_mc->integ_l_um = 0;
    p_mc->integ_r_um = 0;
}

static void brake(motion_core_t * p_mc)
{
    p_mc->sp_v   = 0;
    p_mc->sp_w   = 0;
    p_mc->duty_l = 0;
    p_mc->duty_r = 0;
    reset_pid(p_mc);
}

/* End the active manoeuvre (if any) and report it. */
static void end_maneuver(motion_core_t * p_mc, bool b_ok)
{
    if (p_mc->state != (uint8_t)MOTION_ST_MANEUVER)
    {
        return;
    }
    if (b_ok)
    {
        p_mc->last_done_seq = p_mc->m_seq;
        p_mc->last_done_src = p_mc->m_src;
        p_mc->done_count++;
        push_event(p_mc, (uint8_t)MOTION_EV_DONE, p_mc->m_seq, p_mc->m_src);
    }
    else
    {
        p_mc->last_fail_seq = p_mc->m_seq;
        p_mc->fail_count++;
        push_event(p_mc, (uint8_t)MOTION_EV_FAILED, p_mc->m_seq, p_mc->m_src);
    }
    p_mc->phase = (uint8_t)MOTION_PH_NONE;
    p_mc->state = (uint8_t)MOTION_ST_IDLE_BRAKED;
}

static int64_t counts_to_um(int32_t counts)
{
    return ((int64_t)counts * MOTION_WHEEL_CIRC_UM) / MOTION_COUNTS_PER_REV;
}

/* Wheel arc (um) for a turn of angle_cdeg about the robot centre. */
static int64_t turn_arc_um(int32_t angle_cdeg)
{
    return ((int64_t)app_abs_i32(angle_cdeg) * MOTION_TRACK_MM * PI_E6) / 36000000;
}

uint32_t motion_core_expected_ms(int64_t distance_um, int32_t vmax_mm_s)
{
    int64_t t_ms;

    if (vmax_mm_s <= 0)
    {
        return 0u;
    }
    /* um / (mm/s) = ms; plus the time lost accelerating and braking */
    t_ms = (distance_um / vmax_mm_s) + (((int64_t)vmax_mm_s * 1000) / MOTION_ACCEL_MM_S2);
    return (t_ms > 600000) ? 600000u : (uint32_t)t_ms;
}

static void start_maneuver(motion_core_t * p_mc, motion_cmd_t const * p_cmd,
                           int64_t target_um, int32_t vmax, int8_t dir, uint32_t now_ms)
{
    int32_t v0;

    p_mc->m_cmd       = p_cmd->cmd;
    p_mc->m_seq       = p_cmd->seq;
    p_mc->m_src       = p_cmd->src;
    p_mc->m_dir       = dir;
    p_mc->m_target_um = target_um;
    p_mc->m_start_l_um = p_mc->dist_l_um;
    p_mc->m_start_r_um = p_mc->dist_r_um;
    p_mc->m_vmax      = vmax;
    p_mc->m_start_ms  = now_ms;
    p_mc->m_timeout_ms = (2u * motion_core_expected_ms(target_um, vmax)) + MOTION_TIMEOUT_MARGIN_MS;

    /* A straight MOVE continues smoothly from the current forward speed. */
    v0 = (p_cmd->cmd == (uint8_t)MOTION_CMD_MOVE) ? (p_mc->sp_v * dir) : 0;
    p_mc->m_prof_v = app_clamp_i32(v0, 0, vmax);
    p_mc->phase    = (uint8_t)MOTION_PH_ACCEL;
    p_mc->state    = (uint8_t)MOTION_ST_MANEUVER;
}

/* ------------------------------------------------------------------ */
void motion_core_init(motion_core_t * p_mc, uint32_t now_ms)
{
    uint8_t * p_byte = (uint8_t *)p_mc;
    size_t    i;

    for (i = 0u; i < sizeof(*p_mc); i++)
    {
        p_byte[i] = 0u;
    }
    p_mc->state   = (uint8_t)MOTION_ST_IDLE_BRAKED;
    p_mc->auth    = (uint8_t)MOTION_SRC_MISSION;
    p_mc->prev_ms = now_ms;
}

motion_result_t motion_core_command(motion_core_t * p_mc, motion_cmd_t const * p_cmd,
                                    uint32_t now_ms)
{
    motion_result_t res = MOTION_RES_ACCEPTED;

    if (p_cmd->src >= (uint8_t)MOTION_SRC_COUNT)
    {
        return MOTION_RES_INVALID;
    }

    if (p_cmd->cmd == (uint8_t)MOTION_CMD_CLEAR_ESTOP)
    {
        if (p_cmd->src != (uint8_t)MOTION_SRC_MISSION)
        {
            p_mc->rejected++;
            return MOTION_RES_REJECTED_AUTH;
        }
        return (p_mc->state == (uint8_t)MOTION_ST_ESTOP_LATCHED) ? MOTION_RES_CLEAR_ESTOP
                                                                 : MOTION_RES_ACCEPTED;
    }
    if (p_mc->state == (uint8_t)MOTION_ST_ESTOP_LATCHED)
    {
        p_mc->rejected++;
        return MOTION_RES_REJECTED_ESTOP;
    }

    switch (p_cmd->cmd)
    {
        case MOTION_CMD_SET_AUTH:
            if (p_cmd->src != (uint8_t)MOTION_SRC_MISSION)
            {
                res = MOTION_RES_REJECTED_AUTH;
            }
            else if ((p_cmd->p0 < 0) || (p_cmd->p0 >= (int16_t)MOTION_SRC_COUNT))
            {
                res = MOTION_RES_INVALID;
            }
            else
            {
                p_mc->auth = (uint8_t)p_cmd->p0;
            }
            break;

        case MOTION_CMD_STOP:
            if ((p_cmd->src != (uint8_t)MOTION_SRC_MISSION) && (p_cmd->src != p_mc->auth))
            {
                res = MOTION_RES_REJECTED_AUTH;
            }
            else if (p_mc->state != (uint8_t)MOTION_ST_IDLE_BRAKED)
            {
                end_maneuver(p_mc, false);
                p_mc->state = (uint8_t)MOTION_ST_STOPPING;
            }
            else
            {
                /* already braked */
            }
            break;

        case MOTION_CMD_DRIVE:
            if (p_cmd->src != p_mc->auth)
            {
                res = MOTION_RES_REJECTED_AUTH;
            }
            else
            {
                end_maneuver(p_mc, false);
                p_mc->drive_v       = app_clamp_i32(p_cmd->p0, -MOTION_V_MAX_MM_S, MOTION_V_MAX_MM_S);
                p_mc->drive_w       = app_clamp_i32(p_cmd->p1, -6000, 6000);
                p_mc->last_drive_ms = now_ms;
                p_mc->state         = (uint8_t)MOTION_ST_VELOCITY_TRACK;
            }
            break;

        case MOTION_CMD_MOVE:
        case MOTION_CMD_TURN:
            if (p_cmd->src != p_mc->auth)
            {
                res = MOTION_RES_REJECTED_AUTH;
            }
            else if (p_cmd->p1 <= 0)
            {
                res = MOTION_RES_INVALID;
            }
            else
            {
                int8_t  dir = (p_cmd->p0 < 0) ? (int8_t)-1 : (int8_t)1;
                int64_t target;
                int32_t vmax;

                end_maneuver(p_mc, false);
                if (p_cmd->cmd == (uint8_t)MOTION_CMD_MOVE)
                {
                    target = (int64_t)app_abs_i32(p_cmd->p0) * 1000;
                    vmax   = app_min_i32(p_cmd->p1, MOTION_V_MAX_MM_S);
                }
                else
                {
                    /* p0 in 0.1 deg; p1 in deg/s -> wheel speed in mm/s */
                    target = turn_arc_um((int32_t)p_cmd->p0 * 10);
                    vmax   = (int32_t)(((int64_t)p_cmd->p1 * MOTION_TRACK_MM * PI_E6) / 360000000);
                    vmax   = app_clamp_i32(vmax, MOTION_PROFILE_V_MIN, MOTION_V_MAX_MM_S);
                }
                start_maneuver(p_mc, p_cmd, target, vmax, dir, now_ms);
                if (target == 0)
                {
                    end_maneuver(p_mc, true);   /* nothing to do */
                }
            }
            break;

        default:
            res = MOTION_RES_INVALID;
            break;
    }

    if ((res == MOTION_RES_REJECTED_AUTH) || (res == MOTION_RES_INVALID))
    {
        p_mc->rejected++;
    }
    return res;
}

void motion_core_estop_cleared(motion_core_t * p_mc)
{
    if (p_mc->state == (uint8_t)MOTION_ST_ESTOP_LATCHED)
    {
        brake(p_mc);
        p_mc->state = (uint8_t)MOTION_ST_IDLE_BRAKED;
        p_mc->auth  = (uint8_t)MOTION_SRC_MISSION;
    }
}

/* ------------------------------------------------------------------ */
/* Ramp value toward target by at most step (step >= 0). */
static int32_t ramp(int32_t value, int32_t target, int32_t step)
{
    if (value < target)
    {
        return app_min_i32(value + step, target);
    }
    return app_max_i32(value - step, target);
}

static int16_t pid_wheel(int32_t sp, int32_t v, int32_t * p_integ_um, uint32_t dt_ms)
{
    int32_t const integ_max = (int32_t)(((int64_t)MOTION_I_MAX * 1000000) / MOTION_KI_MILLI);
    int32_t       e;
    int32_t       ff;
    int32_t       u;

    if (sp == 0)
    {
        *p_integ_um = 0;
        return 0;                       /* wheel braked */
    }
    e  = sp - v;
    ff = ((MOTION_KFF_MILLI * sp) / 1000) + ((sp > 0) ? MOTION_MIN_DUTY : -MOTION_MIN_DUTY);
    u  = ff + ((MOTION_KP_MILLI * e) / 1000) +
         (int32_t)(((int64_t)MOTION_KI_MILLI * *p_integ_um) / 1000000);

    /* Anti-windup: integrate only while not saturated, or when the error
       pulls the output back out of saturation. */
    if ((app_abs_i32(u) < MOTION_DUTY_MAX) || ((e > 0) != (u > 0)))
    {
        *p_integ_um = app_clamp_i32(*p_integ_um + (e * (int32_t)dt_ms), -integ_max, integ_max);
        u = ff + ((MOTION_KP_MILLI * e) / 1000) +
            (int32_t)(((int64_t)MOTION_KI_MILLI * *p_integ_um) / 1000000);
    }
    return (int16_t)app_clamp_i32(u, -MOTION_DUTY_MAX, MOTION_DUTY_MAX);
}

static void update_odometry(motion_core_t * p_mc, int32_t raw_l, int32_t raw_r,
                            uint32_t dt_ms, uint32_t now_ms)
{
    int32_t const l = MOTION_ENC_SIGN_L * raw_l;
    int32_t const r = MOTION_ENC_SIGN_R * raw_r;
    int32_t       dcl;
    int32_t       dcr;
    int64_t       dl;
    int64_t       dr;
    int64_t       ds;
    int32_t       h_old;
    int32_t       h_mid;
    int32_t       vraw;

    if (!p_mc->b_have_prev)
    {
        p_mc->enc_l       = l;
        p_mc->enc_r       = r;
        p_mc->b_have_prev = true;
        return;
    }
    /* Modular difference: correct across the 32-bit counter wrap. */
    dcl = (int32_t)((uint32_t)l - (uint32_t)p_mc->enc_l);
    dcr = (int32_t)((uint32_t)r - (uint32_t)p_mc->enc_r);
    p_mc->enc_l = l;
    p_mc->enc_r = r;

    dl = counts_to_um(dcl);
    dr = counts_to_um(dcr);
    p_mc->dist_l_um += dl;
    p_mc->dist_r_um += dr;

    h_old = p_mc->heading_cdeg;
    p_mc->heading_cdeg = (int32_t)(((p_mc->dist_r_um - p_mc->dist_l_um) * 18000000) /
                                   ((int64_t)PI_E6 * MOTION_TRACK_MM));
    h_mid = h_old + ((p_mc->heading_cdeg - h_old) / 2);
    ds    = (dl + dr) / 2;
    p_mc->x_um += (ds * app_cos_q15(h_mid)) / 32768;
    p_mc->y_um += (ds * app_sin_q15(h_mid)) / 32768;

    vraw = (int32_t)(dl / (int64_t)dt_ms);         /* um / ms = mm/s */
    p_mc->v_l += (vraw - p_mc->v_l) / 2;
    vraw = (int32_t)(dr / (int64_t)dt_ms);
    p_mc->v_r += (vraw - p_mc->v_r) / 2;
    vraw = ((p_mc->heading_cdeg - h_old) * 1000) / (int32_t)dt_ms;
    p_mc->yaw_rate_cdeg_s += (vraw - p_mc->yaw_rate_cdeg_s) / YAW_FILTER_DIV;

    /* Encoder glitch check (report only). */
    if ((app_abs_i32(dcl) > MOTION_ENC_GLITCH_COUNTS) || (app_abs_i32(dcr) > MOTION_ENC_GLITCH_COUNTS))
    {
        if (!p_mc->b_enc_fault)
        {
            push_event(p_mc, (uint8_t)MOTION_EV_ENC_FAULT, 0u, 0u);
        }
        p_mc->b_enc_fault = true;
        p_mc->enc_glitch_ms = now_ms;
    }
}

static void check_stall(motion_core_t * p_mc, uint32_t now_ms)
{
    bool const b_l = (app_abs_i32(p_mc->duty_l) > MOTION_STALL_DUTY) &&
                     (app_abs_i32(p_mc->v_l) < MOTION_STALL_SPEED_MM_S);
    bool const b_r = (app_abs_i32(p_mc->duty_r) > MOTION_STALL_DUTY) &&
                     (app_abs_i32(p_mc->v_r) < MOTION_STALL_SPEED_MM_S);

    if (b_l || b_r)
    {
        if (!p_mc->b_stall_timing)
        {
            p_mc->b_stall_timing = true;
            p_mc->stall_since_ms = now_ms;
        }
        else if ((app_elapsed_ms(now_ms, p_mc->stall_since_ms) >= MOTION_STALL_MS) &&
                 !p_mc->b_stall_reported)
        {
            p_mc->b_stall_reported = true;
            push_event(p_mc, (uint8_t)MOTION_EV_STALL, 0u, 0u);
        }
        else
        {
            /* still timing, or already reported */
        }
    }
    else
    {
        p_mc->b_stall_timing   = false;
        p_mc->b_stall_reported = false;
    }
}

/* MANEUVER: set sp_v / sp_w from the profile.  Returns false when the
   manoeuvre ended this step (outputs already braked). */
static bool step_maneuver(motion_core_t * p_mc, uint32_t now_ms, uint32_t dt_ms)
{
    int64_t const rel_l = p_mc->dist_l_um - p_mc->m_start_l_um;
    int64_t const rel_r = p_mc->dist_r_um - p_mc->m_start_r_um;
    bool const    b_move = (p_mc->m_cmd == (uint8_t)MOTION_CMD_MOVE);
    int64_t       progress;
    int64_t       remaining;
    int64_t       tol;
    int32_t       v_acc;
    int32_t       v_dec;
    int32_t       v;

    if (p_mc->phase == (uint8_t)MOTION_PH_SETTLE)
    {
        brake(p_mc);
        if (app_elapsed_ms(now_ms, p_mc->m_settle_ms) >= MOTION_SETTLE_MS)
        {
            end_maneuver(p_mc, true);
        }
        return false;
    }

    progress  = b_move ? (p_mc->m_dir * (rel_l + rel_r)) / 2
                       : (p_mc->m_dir * (rel_r - rel_l)) / 2;
    remaining = p_mc->m_target_um - progress;
    tol       = b_move ? (int64_t)MOTION_POS_TOL_UM : turn_arc_um(MOTION_ANG_TOL_CDEG);

    if (remaining <= tol)
    {
        p_mc->phase      = (uint8_t)MOTION_PH_SETTLE;
        p_mc->m_settle_ms = now_ms;
        brake(p_mc);
        return false;
    }
    if (app_elapsed_ms(now_ms, p_mc->m_start_ms) >= p_mc->m_timeout_ms)
    {
        brake(p_mc);
        end_maneuver(p_mc, false);
        return false;
    }

    /* Trapezoid: limited by acceleration, by the speed from which the
       remaining distance still allows braking, and by the cruise speed. */
    v_acc = p_mc->m_prof_v + (int32_t)(((uint32_t)MOTION_ACCEL_MM_S2 * dt_ms) / 1000u);
    v_dec = (int32_t)app_isqrt64((uint64_t)((2 * (int64_t)MOTION_ACCEL_MM_S2 * remaining) / 1000));
    v     = app_min_i32(app_min_i32(v_acc, v_dec), p_mc->m_vmax);
    v     = app_max_i32(v, MOTION_PROFILE_V_MIN);
    if (v >= p_mc->m_vmax)
    {
        p_mc->phase = (uint8_t)MOTION_PH_CRUISE;
    }
    else if (v_dec <= v_acc)
    {
        p_mc->phase = (uint8_t)MOTION_PH_DECEL;
    }
    else
    {
        p_mc->phase = (uint8_t)MOTION_PH_ACCEL;
    }
    p_mc->m_prof_v = v;

    if (b_move)
    {
        /* Straight-line correction: steer against the wheel-distance difference. */
        p_mc->sp_v = p_mc->m_dir * v;
        p_mc->sp_w = app_clamp_i32((int32_t)(((rel_l - rel_r) * MOTION_STRAIGHT_K) / 1000),
                                   -1000, 1000);
    }
    else
    {
        /* Spin about the centre; pull any translation back to zero. */
        p_mc->sp_w = (int32_t)(((int64_t)p_mc->m_dir * v * 2000) / MOTION_TRACK_MM);
        p_mc->sp_v = (int32_t)((-4 * ((rel_l + rel_r) / 2)) / 1000);
    }
    return true;
}

void motion_core_step(motion_core_t * p_mc, int32_t enc_l, int32_t enc_r,
                      bool b_estop, uint32_t now_ms)
{
    uint32_t dt_ms = app_elapsed_ms(now_ms, p_mc->prev_ms);
    bool     b_drive = true;

    if ((dt_ms == 0u) || (dt_ms > 100u))
    {
        dt_ms = DT_DEFAULT_MS;          /* first call or a long gap */
    }
    update_odometry(p_mc, enc_l, enc_r, dt_ms, now_ms);
    p_mc->prev_ms = now_ms;

    if (p_mc->b_enc_fault && (app_elapsed_ms(now_ms, p_mc->enc_glitch_ms) >= 1000u))
    {
        p_mc->b_enc_fault = false;
        push_event(p_mc, (uint8_t)MOTION_EV_ENC_OK, 0u, 0u);
    }

    if (b_estop && (p_mc->state != (uint8_t)MOTION_ST_ESTOP_LATCHED))
    {
        end_maneuver(p_mc, false);
        p_mc->state = (uint8_t)MOTION_ST_ESTOP_LATCHED;
        p_mc->phase = (uint8_t)MOTION_PH_NONE;
    }

    if ((p_mc->state == (uint8_t)MOTION_ST_VELOCITY_TRACK) &&
        (app_elapsed_ms(now_ms, p_mc->last_drive_ms) > MOTION_DRIVE_TIMEOUT_MS))
    {
        p_mc->state = (uint8_t)MOTION_ST_STOPPING;     /* line follower went quiet */
    }

    switch (p_mc->state)
    {
        case MOTION_ST_VELOCITY_TRACK:
            p_mc->sp_v = ramp(p_mc->sp_v, p_mc->drive_v,
                              (int32_t)(((uint32_t)MOTION_ACCEL_MM_S2 * dt_ms) / 1000u));
            p_mc->sp_w = p_mc->drive_w;
            break;

        case MOTION_ST_STOPPING:
        {
            int32_t const dv = (int32_t)(((uint32_t)MOTION_STOP_DECEL_MM_S2 * dt_ms) / 1000u);
            int32_t const dw = (dv * 2000) / MOTION_TRACK_MM;

            p_mc->sp_v = ramp(p_mc->sp_v, 0, dv);
            p_mc->sp_w = ramp(p_mc->sp_w, 0, dw);
            if ((p_mc->sp_v == 0) && (p_mc->sp_w == 0))
            {
                brake(p_mc);
                p_mc->state = (uint8_t)MOTION_ST_IDLE_BRAKED;
                b_drive = false;
            }
            break;
        }

        case MOTION_ST_MANEUVER:
            b_drive = step_maneuver(p_mc, now_ms, dt_ms);
            break;

        default:                        /* IDLE_BRAKED, ESTOP_LATCHED */
            brake(p_mc);
            b_drive = false;
            break;
    }

    if (b_drive)
    {
        int32_t const half_w = (int32_t)(((int64_t)p_mc->sp_w * MOTION_TRACK_MM) / 2000);
        int32_t const sp_l   = app_clamp_i32(p_mc->sp_v - half_w, -MOTION_V_MAX_MM_S, MOTION_V_MAX_MM_S);
        int32_t const sp_r   = app_clamp_i32(p_mc->sp_v + half_w, -MOTION_V_MAX_MM_S, MOTION_V_MAX_MM_S);

        p_mc->duty_l = pid_wheel(sp_l, p_mc->v_l, &p_mc->integ_l_um, dt_ms);
        p_mc->duty_r = pid_wheel(sp_r, p_mc->v_r, &p_mc->integ_r_um, dt_ms);
    }
    check_stall(p_mc, now_ms);
}

void motion_core_snapshot(motion_core_t const * p_mc, motion_odom_t * p_out)
{
    p_out->enc_l           = p_mc->enc_l;
    p_out->enc_r           = p_mc->enc_r;
    p_out->dist_mm         = (int32_t)((p_mc->dist_l_um + p_mc->dist_r_um) / 2000);
    p_out->x_mm            = (int32_t)(p_mc->x_um / 1000);
    p_out->y_mm            = (int32_t)(p_mc->y_um / 1000);
    p_out->heading_cdeg    = p_mc->heading_cdeg;
    p_out->v_l_mm_s        = (int16_t)app_clamp_i32(p_mc->v_l, -32768, 32767);
    p_out->v_r_mm_s        = (int16_t)app_clamp_i32(p_mc->v_r, -32768, 32767);
    p_out->v_mm_s          = (int16_t)app_clamp_i32((p_mc->v_l + p_mc->v_r) / 2, -32768, 32767);
    p_out->yaw_rate_cdeg_s = (int16_t)app_clamp_i32(p_mc->yaw_rate_cdeg_s, -32768, 32767);
    p_out->duty_l          = p_mc->duty_l;
    p_out->duty_r          = p_mc->duty_r;
    p_out->rejected        = p_mc->rejected;
    p_out->state           = p_mc->state;
    p_out->phase           = p_mc->phase;
    p_out->auth            = p_mc->auth;
    p_out->last_done_seq   = p_mc->last_done_seq;
    p_out->last_done_src   = p_mc->last_done_src;
    p_out->last_fail_seq   = p_mc->last_fail_seq;
    p_out->done_count      = p_mc->done_count;
    p_out->fail_count      = p_mc->fail_count;
}

bool motion_core_pop_event(motion_core_t * p_mc, motion_event_t * p_ev)
{
    uint8_t i;

    if (p_mc->ev_count == 0u)
    {
        return false;
    }
    *p_ev = p_mc->ev[0];
    for (i = 1u; i < p_mc->ev_count; i++)
    {
        p_mc->ev[i - 1u] = p_mc->ev[i];
    }
    p_mc->ev_count--;
    return true;
}
