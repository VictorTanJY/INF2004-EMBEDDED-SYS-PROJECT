/** @file motion_core.c
 *
 *  @brief Subsystem 2 -- MotionTask controller (pure).  See motion_core.h.
 *  Owner: Buddy 2 (Jedrek).
 */

#include <stddef.h>
#include "motion_core.h"
#include "motion_cfg.h"
#include "../app_types.h"
#include "../app_math.h"
#include "../app_pack.h"

#define PI_E6          3141593 /* pi x 10^6 */
#define DT_DEFAULT_MS  10u
#define YAW_FILTER_DIV 2

/* ---------------------------------------------------------------------
 *  Change Requirement 1: the command word (layout in motion_types.h)
 * --------------------------------------------------------------------- */
#define CMD_SRC_SHIFT 0u
#define CMD_SRC_BITS  2u
#define CMD_CMD_SHIFT 2u
#define CMD_CMD_BITS  3u
#define CMD_SEQ_SHIFT 5u
#define CMD_P0_SHIFT  10u
#define DRIVE_V_BITS  10u
#define DRIVE_W_BITS  12u
#define AMOUNT_BITS   13u /* MOVE mm, TURN 0.1 deg   */
#define RATE_BITS     9u  /* MOVE mm/s, TURN deg/s   */

/* Cast (uint16_t)(sizeof ...): the layout table has fewer than 64 entries. */
#define ODOM_FIELDS \
    ((uint16_t)(sizeof(g_odom_layout) / sizeof(g_odom_layout[0])))

/* ---------------------------------------------------------------------
 *  Change Requirement 1: the shared odometry snapshot.  This table is the
 *  data representation of IF-02: bits stored per field and why.
 * --------------------------------------------------------------------- */
static app_pack_field_t const g_odom_layout[] = {
    {offsetof(motion_types_odom_t, t_ms),
     sizeof(((motion_types_odom_t){0}).t_ms), 32u,
     PK_W}, /* ms since boot, wraps */
    {offsetof(motion_types_odom_t, enc_l),
     sizeof(((motion_types_odom_t){0}).enc_l), 32u,
     PK_W}, /* running count since power-on  */
    {offsetof(motion_types_odom_t, enc_r),
     sizeof(((motion_types_odom_t){0}).enc_r), 32u, PK_W},
    {offsetof(motion_types_odom_t, dist_mm),
     sizeof(((motion_types_odom_t){0}).dist_mm), 32u,
     PK_W}, /* running, readers take differences */
    {offsetof(motion_types_odom_t, x_mm),
     sizeof(((motion_types_odom_t){0}).x_mm), 24u, PK_S}, /* +-8.3 km */
    {offsetof(motion_types_odom_t, y_mm),
     sizeof(((motion_types_odom_t){0}).y_mm), 24u, PK_S},
    {offsetof(motion_types_odom_t, heading_cdeg),
     sizeof(((motion_types_odom_t){0}).heading_cdeg), 32u,
     PK_W}, /* accumulated, never wrapped    */
    {offsetof(motion_types_odom_t, v_l_mm_s),
     sizeof(((motion_types_odom_t){0}).v_l_mm_s), 10u,
     PK_S}, /* +-511 (set-points clamp at 450) */
    {offsetof(motion_types_odom_t, v_r_mm_s),
     sizeof(((motion_types_odom_t){0}).v_r_mm_s), 10u, PK_S},
    {offsetof(motion_types_odom_t, v_mm_s),
     sizeof(((motion_types_odom_t){0}).v_mm_s), 10u, PK_S},
    {offsetof(motion_types_odom_t, yaw_rate_cdeg_s),
     sizeof(((motion_types_odom_t){0}).yaw_rate_cdeg_s), 16u,
     PK_S}, /* the int16_t range, +-327 deg/s */
    {offsetof(motion_types_odom_t, duty_l),
     sizeof(((motion_types_odom_t){0}).duty_l), 11u,
     PK_S}, /* +-1000 per mille */
    {offsetof(motion_types_odom_t, duty_r),
     sizeof(((motion_types_odom_t){0}).duty_r), 11u, PK_S},
    {offsetof(motion_types_odom_t, rejected),
     sizeof(((motion_types_odom_t){0}).rejected), 16u, PK_W}, /* counter */
    {offsetof(motion_types_odom_t, state),
     sizeof(((motion_types_odom_t){0}).state), 3u, PK_U}, /* 5 states */
    {offsetof(motion_types_odom_t, phase),
     sizeof(((motion_types_odom_t){0}).phase), 3u, PK_U}, /* 5 phases */
    {offsetof(motion_types_odom_t, auth),
     sizeof(((motion_types_odom_t){0}).auth), 2u, PK_U}, /* 3 sources */
    {offsetof(motion_types_odom_t, last_done_seq),
     sizeof(((motion_types_odom_t){0}).last_done_seq), 5u,
     PK_W}, /* MOTION_SEQ_BITS */
    {offsetof(motion_types_odom_t, last_done_src),
     sizeof(((motion_types_odom_t){0}).last_done_src), 2u, PK_U},
    {offsetof(motion_types_odom_t, last_fail_seq),
     sizeof(((motion_types_odom_t){0}).last_fail_seq), 5u, PK_W},
    {offsetof(motion_types_odom_t, done_count),
     sizeof(((motion_types_odom_t){0}).done_count), 8u,
     PK_W}, /* counters, readers compare     */
    {offsetof(motion_types_odom_t, fail_count),
     sizeof(((motion_types_odom_t){0}).fail_count), 8u, PK_W},
    {offsetof(motion_types_odom_t, hw_ok),
     sizeof(((motion_types_odom_t){0}).hw_ok), 1u, PK_U}};

static void    push_event(motion_core_t * p_mc, uint8_t code, uint8_t seq,
                          uint8_t src);
static void    reset_pid(motion_core_t * p_mc);
static void    brake(motion_core_t * p_mc);
static void    end_maneuver(motion_core_t * p_mc, bool b_ok);
static int64_t counts_to_um(int32_t counts);
static int64_t turn_arc_um(int32_t angle_cdeg);
static void start_maneuver(motion_core_t *            p_mc,
                           motion_types_cmd_t const * p_cmd, int64_t target_um,
                           int32_t vmax, int8_t dir, uint32_t now_ms);
static int32_t ramp(int32_t value, int32_t target, int32_t step);
static int16_t pid_wheel(int32_t setpoint, int32_t speed, int32_t * p_integ_um,
                         int8_t * p_dir, uint32_t dt_ms);
static int32_t pid_output(int32_t setpoint, int32_t speed, int32_t * p_integ_um,
                          int32_t integ_max, uint32_t dt_ms);
static void update_odometry(motion_core_t * p_mc, int32_t raw_l, int32_t raw_r,
                            uint32_t dt_ms, uint32_t now_ms);
static void odometry_step(motion_core_t * p_mc, int32_t enc_left,
                          int32_t enc_right, uint32_t dt_ms, uint32_t now_ms);
static void check_stall(motion_core_t * p_mc, uint32_t now_ms);
static void maneuver_drive(motion_core_t * p_mc, int64_t rel_l, int64_t rel_r,
                           int64_t remaining, uint32_t dt_ms);
static bool step_maneuver(motion_core_t * p_mc, uint32_t now_ms,
                          uint32_t dt_ms);
static uint8_t              p0_bits(uint8_t cmd);
static motion_core_result_t command_run(motion_core_t *            p_mc,
                                        motion_types_cmd_t const * p_cmd,
                                        uint32_t                   now_ms);
static void command_move(motion_core_t * p_mc, motion_types_cmd_t const * p_cmd,
                         uint32_t now_ms);

uint32_t motion_core_expected_ms (int64_t distance_um, int32_t vmax_mm_s)
{
    int64_t  t_ms;
    uint32_t expected_ms = 0u;

    if (vmax_mm_s > 0)
    {
        /* um / (mm/s) = ms; plus the time lost accelerating and braking */
        /* Casts (int64_t)vmax_mm_s: every int32_t value fits; (uint32_t)t_ms:
           t_ms <= 600000 (tested in the same statement). */
        t_ms        = (distance_um / vmax_mm_s)
                      + (((int64_t)vmax_mm_s * 1000) / MOTION_ACCEL_MM_S2);
        expected_ms = (t_ms > 600000) ? 600000u : (uint32_t)t_ms;
    }

    return expected_ms;
}

/* ------------------------------------------------------------------ */
void motion_core_init (motion_core_t * p_mc, uint32_t now_ms)
{
    /* Cast (uint8_t *)p_mc: byte view of the struct for clearing; no value
       converted. */
    uint8_t * p_byte = (uint8_t *)p_mc;
    size_t    idx;

    for (idx = 0u; idx < sizeof(*p_mc); idx++)
    {
        p_byte[idx] = 0u;
    }

    p_mc->state   = MOTION_ST_IDLE_BRAKED;
    p_mc->auth    = MOTION_SRC_MISSION;
    p_mc->prev_ms = now_ms;
}

motion_core_result_t motion_core_command (motion_core_t *            p_mc,
                                          motion_types_cmd_t const * p_cmd,
                                          uint32_t                   now_ms)
{
    motion_core_result_t res;

    if (p_cmd->src >= MOTION_SRC_COUNT)
    {
        res = MOTION_RES_INVALID;
    }
    else if (MOTION_CMD_CLEAR_ESTOP == p_cmd->cmd)
    {
        if (p_cmd->src != MOTION_SRC_MISSION)
        {
            p_mc->rejected++;
            res = MOTION_RES_REJECTED_AUTH;
        }
        else
        {
            res = (MOTION_ST_ESTOP_LATCHED == p_mc->state)
                      ? MOTION_RES_CLEAR_ESTOP
                      : MOTION_RES_ACCEPTED;
        }
    }
    else if (MOTION_ST_ESTOP_LATCHED == p_mc->state)
    {
        p_mc->rejected++;
        res = MOTION_RES_REJECTED_ESTOP;
    }
    else
    {
        res = command_run(p_mc, p_cmd, now_ms);

        if ((MOTION_RES_REJECTED_AUTH == res) || (MOTION_RES_INVALID == res))
        {
            p_mc->rejected++;
        }
    }

    return res;
}

void motion_core_estop_cleared (motion_core_t * p_mc)
{
    if (MOTION_ST_ESTOP_LATCHED == p_mc->state)
    {
        brake(p_mc);
        p_mc->state = MOTION_ST_IDLE_BRAKED;
        p_mc->auth  = MOTION_SRC_MISSION;
    }
}

void motion_core_step (motion_core_t * p_mc, int32_t enc_l, int32_t enc_r,
                       bool b_estop, uint32_t now_ms)
{
    uint32_t dt_ms   = app_types_elapsed_ms(now_ms, p_mc->prev_ms);
    bool     b_drive = true;

    if ((0u == dt_ms) || (dt_ms > 100u))
    {
        dt_ms = DT_DEFAULT_MS; /* first call or a long gap */
    }

    update_odometry(p_mc, enc_l, enc_r, dt_ms, now_ms);
    p_mc->prev_ms = now_ms;

    if (p_mc->b_enc_fault
        && (app_types_elapsed_ms(now_ms, p_mc->enc_glitch_ms) >= 1000u))
    {
        p_mc->b_enc_fault = false;
        push_event(p_mc, MOTION_EV_ENC_OK, 0u, 0u);
    }

    if (b_estop && (p_mc->state != MOTION_ST_ESTOP_LATCHED))
    {
        end_maneuver(p_mc, false);
        p_mc->state = MOTION_ST_ESTOP_LATCHED;
        p_mc->phase = MOTION_PH_NONE;
    }

    if ((MOTION_ST_VELOCITY_TRACK == p_mc->state)
        && (app_types_elapsed_ms(now_ms, p_mc->last_drive_ms)
            > MOTION_DRIVE_TIMEOUT_MS))
    {
        p_mc->state = MOTION_ST_STOPPING; /* line follower went quiet */
    }

    switch (p_mc->state)
    {
        case MOTION_ST_VELOCITY_TRACK:
            /* Casts (int32_t)(((uint32_t)MOTION_ACCEL_M ...)): an acceleration
               step: MOTION_ACCEL_MM_S2 x dt_ms (<= 100 ms) / 1000;
               (uint32_t)MOTION_ACCEL_MM_S2: a constant that fits. */
            p_mc->sp_v =
                ramp(p_mc->sp_v, p_mc->drive_v,
                     (int32_t)(((uint32_t)MOTION_ACCEL_MM_S2 * dt_ms) / 1000u));
            p_mc->sp_w = p_mc->drive_w;
        break;

        case MOTION_ST_STOPPING:
        {
            /* Casts (int32_t)(((uint32_t)MOTION_STOP_DE ...)): a deceleration
               step: MOTION_STOP_DECEL_MM_S2 x dt_ms (<= 100 ms) / 1000;
               (uint32_t)MOTION_STOP_DECEL_MM_S2: a constant that fits. */
            int32_t const dv_step =
                (int32_t)(((uint32_t)MOTION_STOP_DECEL_MM_S2 * dt_ms) / 1000u);
            int32_t const dw_step = (dv_step * 2000) / MOTION_TRACK_MM;

            p_mc->sp_v = ramp(p_mc->sp_v, 0, dv_step);
            p_mc->sp_w = ramp(p_mc->sp_w, 0, dw_step);

            if ((0 == p_mc->sp_v) && (0 == p_mc->sp_w))
            {
                brake(p_mc);
                p_mc->state = MOTION_ST_IDLE_BRAKED;
                b_drive     = false;
            }
        }
        break;

        case MOTION_ST_MANEUVER:
            b_drive = step_maneuver(p_mc, now_ms, dt_ms);
        break;

        default: /* IDLE_BRAKED, ESTOP_LATCHED */
            brake(p_mc);
            b_drive = false;
        break;
    }

    if (b_drive)
    {
        /* Casts (int32_t)(((int64_t)p_mc->sp_w * ...)): the value is
           -118111600..118111600; (int64_t)p_mc->sp_w: every int32_t value
           fits. */
        int32_t const half_w =
            (int32_t)(((int64_t)p_mc->sp_w * MOTION_TRACK_MM) / 2000);
        int32_t const sp_l = app_types_clamp_i32(
            p_mc->sp_v - half_w, -MOTION_V_MAX_MM_S, MOTION_V_MAX_MM_S);
        int32_t const sp_r = app_types_clamp_i32(
            p_mc->sp_v + half_w, -MOTION_V_MAX_MM_S, MOTION_V_MAX_MM_S);

        p_mc->duty_l =
            pid_wheel(sp_l, p_mc->v_l, &p_mc->integ_l_um, &p_mc->dir_l, dt_ms);
        p_mc->duty_r =
            pid_wheel(sp_r, p_mc->v_r, &p_mc->integ_r_um, &p_mc->dir_r, dt_ms);
    }

    check_stall(p_mc, now_ms);
}

void motion_core_snapshot (motion_core_t const * p_mc,
                           motion_types_odom_t * p_out)
{
    /* Casts (int32_t)((p_mc->dist_l_um + ...)): distance in mm: fits int32_t
       for the first 2147 km; (int32_t)(p_mc->x_um / 1000),
       (int32_t)(p_mc->y_um / 1000): position in mm: fits int32_t for 2147 km
       from the start; (int16_t)app_types_clamp_i32(...): the value is
       -32768..32767. */
    p_out->enc_l        = p_mc->enc_l;
    p_out->enc_r        = p_mc->enc_r;
    p_out->dist_mm      = (int32_t)((p_mc->dist_l_um + p_mc->dist_r_um) / 2000);
    p_out->x_mm         = (int32_t)(p_mc->x_um / 1000);
    p_out->y_mm         = (int32_t)(p_mc->y_um / 1000);
    p_out->heading_cdeg = p_mc->heading_cdeg;
    p_out->v_l_mm_s = (int16_t)app_types_clamp_i32(p_mc->v_l, -32768, 32767);
    p_out->v_r_mm_s = (int16_t)app_types_clamp_i32(p_mc->v_r, -32768, 32767);
    p_out->v_mm_s   = (int16_t)app_types_clamp_i32((p_mc->v_l + p_mc->v_r) / 2,
                                                   -32768, 32767);
    p_out->yaw_rate_cdeg_s =
        (int16_t)app_types_clamp_i32(p_mc->yaw_rate_cdeg_s, -32768, 32767);
    p_out->duty_l        = p_mc->duty_l;
    p_out->duty_r        = p_mc->duty_r;
    p_out->rejected      = p_mc->rejected;
    p_out->state         = p_mc->state;
    p_out->phase         = p_mc->phase;
    p_out->auth          = p_mc->auth;
    p_out->last_done_seq = p_mc->last_done_seq;
    p_out->last_done_src = p_mc->last_done_src;
    p_out->last_fail_seq = p_mc->last_fail_seq;
    p_out->done_count    = p_mc->done_count;
    p_out->fail_count    = p_mc->fail_count;
}

bool motion_core_pop_event (motion_core_t * p_mc, motion_core_event_t * p_ev)
{
    uint8_t    idx;
    bool const b_have = (p_mc->ev_count != 0u);

    if (b_have)
    {
        *p_ev = p_mc->events[0];
        for (idx = 1u; idx < p_mc->ev_count; idx++)
        {
            p_mc->events[idx - 1u] = p_mc->events[idx];
        }

        p_mc->ev_count--;
    }

    return b_have;
}

uint32_t motion_core_cmd_pack (motion_types_cmd_t const * p_cmd)
{
    uint8_t const cmd       = p_cmd->cmd;
    uint8_t const arg0_bits = p0_bits(cmd);
    uint32_t      word      = 0u;

    /* Cast (uint32_t)p_cmd->seq: every uint8_t value fits. */
    word = app_pack_set(word, CMD_SRC_SHIFT, CMD_SRC_BITS,
                        app_pack_sat_u(p_cmd->src, CMD_SRC_BITS));
    word = app_pack_set(word, CMD_CMD_SHIFT, CMD_CMD_BITS,
                        app_pack_sat_u(cmd, CMD_CMD_BITS));
    word = app_pack_set(word, CMD_SEQ_SHIFT, MOTION_SEQ_BITS,
                        (uint32_t)p_cmd->seq & MOTION_SEQ_MASK);

    if (MOTION_CMD_DRIVE == cmd)
    {
        word = app_pack_set(word, CMD_P0_SHIFT, DRIVE_V_BITS,
                            app_pack_off_s(p_cmd->arg0, DRIVE_V_BITS));
        word = app_pack_set(word, CMD_P0_SHIFT + DRIVE_V_BITS, DRIVE_W_BITS,
                            app_pack_off_s(p_cmd->arg1, DRIVE_W_BITS));
    }
    else if (AMOUNT_BITS == arg0_bits)
    {
        /* Cast (uint32_t)p_cmd->arg1: arg1 >= 0 on this branch (tested in the
           same statement). */
        word = app_pack_set(word, CMD_P0_SHIFT, AMOUNT_BITS,
                            app_pack_off_s(p_cmd->arg0, AMOUNT_BITS));
        word = app_pack_set(
            word, CMD_P0_SHIFT + AMOUNT_BITS, RATE_BITS,
            app_pack_sat_u((p_cmd->arg1 < 0) ? 0u : (uint32_t)p_cmd->arg1,
                           RATE_BITS));
    }
    else if (arg0_bits > 0u)
    {
        /* Cast (uint32_t)p_cmd->arg0: arg0 >= 0 on this branch (tested in the
           same statement). */
        word = app_pack_set(
            word, CMD_P0_SHIFT, arg0_bits,
            app_pack_sat_u((p_cmd->arg0 < 0) ? 0u : (uint32_t)p_cmd->arg0,
                           arg0_bits));
    }
    else
    {
        /* STOP, CLEAR_ESTOP: no payload */
    }

    return word;
}

void motion_core_cmd_unpack (uint32_t word, motion_types_cmd_t * p_cmd)
{
    /* Cast (uint8_t)app_pack_get(word ...): the value is 0..7. */
    uint8_t const cmd =
        (uint8_t)app_pack_get(word, CMD_CMD_SHIFT, CMD_CMD_BITS);
    uint8_t const arg0_bits = p0_bits(cmd);

    /* Casts (uint8_t)app_pack_get(word ...): the value is 0..3;
       (uint8_t)app_pack_get(word ...): the value is 0..31. */
    p_cmd->src = (uint8_t)app_pack_get(word, CMD_SRC_SHIFT, CMD_SRC_BITS);
    p_cmd->cmd = cmd;
    p_cmd->seq = (uint8_t)app_pack_get(word, CMD_SEQ_SHIFT, MOTION_SEQ_BITS);
    p_cmd->reserved = 0u;
    p_cmd->arg0     = 0;
    p_cmd->arg1     = 0;

    if (MOTION_CMD_DRIVE == cmd)
    {
        /* Casts (int16_t)app_pack_unoff_s(...): the value is -512..511;
           (int16_t)app_pack_unoff_s(...): the value is -2048..2047. */
        p_cmd->arg0 = (int16_t)app_pack_unoff_s(
            app_pack_get(word, CMD_P0_SHIFT, DRIVE_V_BITS), DRIVE_V_BITS);
        p_cmd->arg1 = (int16_t)app_pack_unoff_s(
            app_pack_get(word, CMD_P0_SHIFT + DRIVE_V_BITS, DRIVE_W_BITS),
            DRIVE_W_BITS);
    }
    else if (AMOUNT_BITS == arg0_bits)
    {
        /* Casts (int16_t)app_pack_unoff_s(...): the value is -4096..4095;
           (int16_t)app_pack_get(word ...): the value is 0..511. */
        p_cmd->arg0 = (int16_t)app_pack_unoff_s(
            app_pack_get(word, CMD_P0_SHIFT, AMOUNT_BITS), AMOUNT_BITS);
        p_cmd->arg1 =
            (int16_t)app_pack_get(word, CMD_P0_SHIFT + AMOUNT_BITS, RATE_BITS);
    }
    else if (arg0_bits > 0u)
    {
        /* Cast (int16_t)app_pack_get(word ...): a field of arg0_bits (<= 13)
           bits. */
        p_cmd->arg0 = (int16_t)app_pack_get(word, CMD_P0_SHIFT, arg0_bits);
    }
    else
    {
        /* no payload */
    }
}

void motion_core_odom_pack (motion_types_odom_t const * p_odom,
                            uint32_t *                  p_words)
{
    app_pack_pack_rec(g_odom_layout, ODOM_FIELDS, p_odom, p_words,
                      MOTION_ODOM_WORDS);
}

void motion_core_odom_unpack (uint32_t const *      p_words,
                              motion_types_odom_t * p_odom)
{
    p_odom->reserved = 0u;
    app_pack_unpack_rec(g_odom_layout, ODOM_FIELDS, p_words, p_odom);
}

uint32_t motion_core_odom_layout_bits (void)
{
    return app_pack_layout_bits(g_odom_layout, ODOM_FIELDS);
}

/** A command other than CLEAR_ESTOP, with no E-stop latched. */
static motion_core_result_t command_run (motion_core_t *            p_mc,
                                         motion_types_cmd_t const * p_cmd,
                                         uint32_t                   now_ms)
{
    motion_core_result_t res = MOTION_RES_ACCEPTED;

    switch (p_cmd->cmd)
    {
        case MOTION_CMD_SET_AUTH:
            /* Cast (int16_t)MOTION_SRC_COUNT: a constant that fits. */
            if (p_cmd->src != MOTION_SRC_MISSION)
            {
                res = MOTION_RES_REJECTED_AUTH;
            }
            else if ((p_cmd->arg0 < 0)
                     || (p_cmd->arg0 >= (int16_t)MOTION_SRC_COUNT))
            {
                res = MOTION_RES_INVALID;
            }
            else
            {
                /* Cast (uint8_t)p_cmd->arg0: arg0 was checked to be
                   0..MOTION_SRC_COUNT-1 above. */
                p_mc->auth = (uint8_t)p_cmd->arg0;
            }
        break;

        case MOTION_CMD_STOP:
            if ((p_cmd->src != MOTION_SRC_MISSION)
                && (p_cmd->src != p_mc->auth))
            {
                res = MOTION_RES_REJECTED_AUTH;
            }
            else if (p_mc->state != MOTION_ST_IDLE_BRAKED)
            {
                end_maneuver(p_mc, false);
                p_mc->state = MOTION_ST_STOPPING;
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
                p_mc->drive_v = app_types_clamp_i32(
                    p_cmd->arg0, -MOTION_V_MAX_MM_S, MOTION_V_MAX_MM_S);
                p_mc->drive_w = app_types_clamp_i32(p_cmd->arg1, -6000, 6000);
                p_mc->last_drive_ms = now_ms;
                p_mc->state         = MOTION_ST_VELOCITY_TRACK;
            }
        break;

        case MOTION_CMD_MOVE:
        case MOTION_CMD_TURN:
            if (p_cmd->src != p_mc->auth)
            {
                res = MOTION_RES_REJECTED_AUTH;
            }
            else if (p_cmd->arg1 <= 0)
            {
                res = MOTION_RES_INVALID;
            }
            else
            {
                command_move(p_mc, p_cmd, now_ms);
            }
        break;

        default:
            res = MOTION_RES_INVALID;
        break;
    }

    return res;
}

/** MOVE / TURN from the authorised source with a valid speed. */
static void command_move (motion_core_t *            p_mc,
                          motion_types_cmd_t const * p_cmd, uint32_t now_ms)
{
    int8_t  dir = (p_cmd->arg0 < 0) ? -1 : 1;
    int64_t target;
    int32_t vmax;

    end_maneuver(p_mc, false);

    if (MOTION_CMD_MOVE == p_cmd->cmd)
    {
        /* Cast (int64_t)app_types_abs_i32(...): every int32_t value fits. */
        target = (int64_t)app_types_abs_i32(p_cmd->arg0) * 1000;
        vmax   = app_types_min_i32(p_cmd->arg1, MOTION_V_MAX_MM_S);
    }
    else
    {
        /* p0 in 0.1 deg; p1 in deg/s -> wheel speed in mm/s */
        /* Casts (int32_t)p_cmd->arg0, (int64_t)p_cmd->arg1: every int16_t value
           fits; (int32_t)(((int64_t)p_cmd->arg1 * ...)): the value is
           -31455..31454. */
        target = turn_arc_um((int32_t)p_cmd->arg0 * 10);
        vmax   = (int32_t)(((int64_t)p_cmd->arg1 * MOTION_TRACK_MM * PI_E6)
                           / 360000000);
        vmax =
            app_types_clamp_i32(vmax, MOTION_PROFILE_V_MIN, MOTION_V_MAX_MM_S);
    }

    start_maneuver(p_mc, p_cmd, target, vmax, dir, now_ms);

    if (0 == target)
    {
        end_maneuver(p_mc, true); /* nothing to do */
    }
}

/* ------------------------------------------------------------------ */
/** Queue a completion event (code, sequence, source) for MotionTask to report;
    dropped if the queue is full. */
static void push_event (motion_core_t * p_mc, uint8_t code, uint8_t seq,
                        uint8_t src)
{
    if (p_mc->ev_count < MOTION_EV_QUEUE_LEN)
    {
        p_mc->events[p_mc->ev_count].code = code;
        p_mc->events[p_mc->ev_count].seq  = seq;
        p_mc->events[p_mc->ev_count].src  = src;
        p_mc->ev_count++;
    }
}

/** Clear both wheels' integrators and remembered directions. */
static void reset_pid (motion_core_t * p_mc)
{
    p_mc->integ_l_um = 0;
    p_mc->integ_r_um = 0;
    p_mc->dir_l      = 0;
    p_mc->dir_r      = 0;
}

/** Set-points and duties to zero, controller reset: the wheels brake. */
static void brake (motion_core_t * p_mc)
{
    p_mc->sp_v   = 0;
    p_mc->sp_w   = 0;
    p_mc->duty_l = 0;
    p_mc->duty_r = 0;
    reset_pid(p_mc);
}

/** End the active manoeuvre (if any) and report it. */
static void end_maneuver (motion_core_t * p_mc, bool b_ok)
{
    if (MOTION_ST_MANEUVER == p_mc->state)
    {
        if (b_ok)
        {
            p_mc->last_done_seq = p_mc->m_seq;
            p_mc->last_done_src = p_mc->m_src;
            p_mc->done_count++;
            push_event(p_mc, MOTION_EV_DONE, p_mc->m_seq, p_mc->m_src);
        }
        else
        {
            p_mc->last_fail_seq = p_mc->m_seq;
            p_mc->fail_count++;
            push_event(p_mc, MOTION_EV_FAILED, p_mc->m_seq, p_mc->m_src);
        }

        p_mc->phase = MOTION_PH_NONE;
        p_mc->state = MOTION_ST_IDLE_BRAKED;
    }
}

/** Wheel travel of counts encoder counts, um. */
static int64_t counts_to_um (int32_t counts)
{
    /* Cast (int64_t)counts: every int32_t value fits. */
    return ((int64_t)counts * MOTION_WHEEL_CIRC_UM) / MOTION_COUNTS_PER_REV;
}

/** Wheel arc (um) for a turn of angle_cdeg about the robot centre. */
static int64_t turn_arc_um (int32_t angle_cdeg)
{
    /* Cast (int64_t)app_types_abs_i32(...): every int32_t value fits. */
    return ((int64_t)app_types_abs_i32(angle_cdeg) * MOTION_TRACK_MM * PI_E6)
           / 36000000;
}

/** Start a MOVE or TURN: target_um of wheel travel at up to vmax mm/s in
    direction dir, with its time limit. */
static void start_maneuver (motion_core_t *            p_mc,
                            motion_types_cmd_t const * p_cmd, int64_t target_um,
                            int32_t vmax, int8_t dir, uint32_t now_ms)
{
    int32_t speed_start;

    p_mc->m_cmd        = p_cmd->cmd;
    p_mc->m_seq        = p_cmd->seq;
    p_mc->m_src        = p_cmd->src;
    p_mc->m_dir        = dir;
    p_mc->m_target_um  = target_um;
    p_mc->m_start_l_um = p_mc->dist_l_um;
    p_mc->m_start_r_um = p_mc->dist_r_um;
    p_mc->m_vmax       = vmax;
    p_mc->m_start_ms   = now_ms;
    p_mc->m_timeout_ms = (2u * motion_core_expected_ms(target_um, vmax))
                         + MOTION_TIMEOUT_MARGIN_MS;

    /* A straight MOVE continues smoothly from the current forward speed. */
    speed_start    = (MOTION_CMD_MOVE == p_cmd->cmd) ? (p_mc->sp_v * dir) : 0;
    p_mc->m_prof_v = app_types_clamp_i32(speed_start, 0, vmax);
    p_mc->phase    = MOTION_PH_ACCEL;
    p_mc->state    = MOTION_ST_MANEUVER;
}

/* ------------------------------------------------------------------ */
/** Ramp value toward target by at most step (step >= 0). */
static int32_t ramp (int32_t value, int32_t target, int32_t step)
{
    return (value < target) ? app_types_min_i32(value + step, target)
                            : app_types_max_i32(value - step, target);
}

/** PI speed controller with feed-forward for one wheel: duty (per mille) for
    setpoint and measured speed in mm/s, over dt_ms. */
static int16_t pid_wheel (int32_t setpoint, int32_t speed, int32_t * p_integ_um,
                          int8_t * p_dir, uint32_t dt_ms)
{
    /* Casts (int32_t)(((int64_t)MOTION_I_MAX * ...)): the constant 0x1046A
       fits; (int64_t)MOTION_I_MAX: a constant that fits. */
    int32_t const integ_max =
        (int32_t)(((int64_t)MOTION_I_MAX * 1000000) / MOTION_KI_MILLI);
    int8_t const dir    = (setpoint > 0) ? 1 : -1;
    int32_t      output = 0; /* wheel braked */

    /* Below a minimum target the wheel is simply braked: otherwise the
       static-friction offset flips between + and - MOTION_MIN_DUTY while
       the steering asks the inner wheel for about 0 mm/s (a jolt each
       time). */
    if (app_types_abs_i32(setpoint) < MOTION_MIN_TARGET_MM_S)
    {
        *p_integ_um = 0;
        *p_dir      = 0;
    }
    else
    {
        /* A change of direction starts the controller clean. */
        if (dir != *p_dir)
        {
            *p_integ_um = 0;
            *p_dir      = dir;
        }

        output = pid_output(setpoint, speed, p_integ_um, integ_max, dt_ms);
    }

    /* Cast (int16_t)app_types_clamp_i32(...): the value is -1000..1000. */
    return (int16_t)app_types_clamp_i32(output, -MOTION_DUTY_MAX,
                                        MOTION_DUTY_MAX);
}

/** PI output with feed-forward and anti-windup for one wheel (pid_wheel()),
    before the duty limit. */
static int32_t pid_output (int32_t setpoint, int32_t speed,
                           int32_t * p_integ_um, int32_t integ_max,
                           uint32_t dt_ms)
{
    int32_t error;
    int32_t feedfwd;
    int32_t output;

    /* Casts (int32_t)(((int64_t)MOTION_KI_MILLI ...)): the value is
       -12884901..12884901; (int64_t)MOTION_KI_MILLI: a constant that fits. */
    error   = setpoint - speed;
    feedfwd = ((MOTION_KFF_MILLI * setpoint) / 1000)
              + ((setpoint > 0) ? MOTION_MIN_DUTY : -MOTION_MIN_DUTY);
    output  = feedfwd + ((MOTION_KP_MILLI * error) / 1000)
              + (int32_t)(((int64_t)MOTION_KI_MILLI * *p_integ_um) / 1000000);

    /* Anti-windup: integrate only while not saturated, or when the error
       pulls the output back out of saturation. */
    if ((app_types_abs_i32(output) < MOTION_DUTY_MAX)
        || ((error > 0) != (output > 0)))
    {
        /* Casts (int32_t)dt_ms: dt_ms is one control period (<= 100 ms);
           (int32_t)(((int64_t)MOTION_KI_MILLI ...)): the value is
           -12884901..12884901; (int64_t)MOTION_KI_MILLI: a constant that
           fits. */
        *p_integ_um = app_types_clamp_i32(
            *p_integ_um + (error * (int32_t)dt_ms), -integ_max, integ_max);
        output =
            feedfwd + ((MOTION_KP_MILLI * error) / 1000)
            + (int32_t)(((int64_t)MOTION_KI_MILLI * *p_integ_um) / 1000000);
    }

    return output;
}

/** Wheel speeds, distance, heading and position from the new encoder counts,
    dt_ms after the last. */
static void update_odometry (motion_core_t * p_mc, int32_t raw_l, int32_t raw_r,
                             uint32_t dt_ms, uint32_t now_ms)
{
    int32_t const enc_left  = MOTION_ENC_SIGN_L * raw_l;
    int32_t const enc_right = MOTION_ENC_SIGN_R * raw_r;

    if (!p_mc->b_have_prev)
    {
        p_mc->enc_l       = enc_left;
        p_mc->enc_r       = enc_right;
        p_mc->b_have_prev = true;
    }
    else
    {
        odometry_step(p_mc, enc_left, enc_right, dt_ms, now_ms);
    }
}

/** update_odometry() after the first sample: enc_left and enc_right are the
    signed counts. */
static void odometry_step (motion_core_t * p_mc, int32_t enc_left,
                           int32_t enc_right, uint32_t dt_ms, uint32_t now_ms)
{
    int32_t dcl;
    int32_t dcr;
    int64_t delta_left;
    int64_t delta_right;
    int64_t delta_mid;
    int32_t h_old;
    int32_t h_mid;
    int32_t vraw;

    /* Modular difference: correct across the 32-bit counter wrap. */
    /* Casts (int32_t)((uint32_t)enc_left - ...),
       (int32_t)((uint32_t)enc_right - ...): difference of two wrapping 32-bit
       counters, read as signed: they are less than 2^31 apart;
       (uint32_t)enc_left, (uint32_t)p_mc->enc_l, (uint32_t)enc_right,
       (uint32_t)p_mc->enc_r: counts as a wrapping 32-bit pattern for the
       difference. */
    dcl         = (int32_t)((uint32_t)enc_left - (uint32_t)p_mc->enc_l);
    dcr         = (int32_t)((uint32_t)enc_right - (uint32_t)p_mc->enc_r);
    p_mc->enc_l = enc_left;
    p_mc->enc_r = enc_right;

    delta_left  = counts_to_um(dcl);
    delta_right = counts_to_um(dcr);
    p_mc->dist_l_um += delta_left;
    p_mc->dist_r_um += delta_right;

    /* Casts (int32_t)(((p_mc->dist_r_um - ...))): the heading change of one 10
       ms period, a few degrees in 0.01 deg; (int64_t)PI_E6: a constant that
       fits. */
    h_old = p_mc->heading_cdeg;
    p_mc->heading_cdeg =
        (int32_t)(((p_mc->dist_r_um - p_mc->dist_l_um) * 18000000)
                  / ((int64_t)PI_E6 * MOTION_TRACK_MM));
    h_mid     = h_old + ((p_mc->heading_cdeg - h_old) / 2);
    delta_mid = (delta_left + delta_right) / 2;
    p_mc->x_um += (delta_mid * app_math_cos_q15(h_mid)) / 32768;
    p_mc->y_um += (delta_mid * app_math_sin_q15(h_mid)) / 32768;

    /* Casts (int32_t)(delta_left / ...), (int32_t)(delta_right / ...): wheel
       speed in mm/s: travel of one period over its length; (int64_t)dt_ms:
       every uint32_t value fits; (int32_t)dt_ms: dt_ms is one control period
       (<= 100 ms). */
    vraw = (int32_t)(delta_left / (int64_t)dt_ms); /* um / ms = mm/s */
    p_mc->v_l += (vraw - p_mc->v_l) / 2;
    vraw = (int32_t)(delta_right / (int64_t)dt_ms);
    p_mc->v_r += (vraw - p_mc->v_r) / 2;
    vraw = ((p_mc->heading_cdeg - h_old) * 1000) / (int32_t)dt_ms;
    p_mc->yaw_rate_cdeg_s += (vraw - p_mc->yaw_rate_cdeg_s) / YAW_FILTER_DIV;

    /* Encoder glitch check (report only). */
    if ((app_types_abs_i32(dcl) > MOTION_ENC_GLITCH_COUNTS)
        || (app_types_abs_i32(dcr) > MOTION_ENC_GLITCH_COUNTS))
    {
        if (!p_mc->b_enc_fault)
        {
            push_event(p_mc, MOTION_EV_ENC_FAULT, 0u, 0u);
        }

        p_mc->b_enc_fault   = true;
        p_mc->enc_glitch_ms = now_ms;
    }
}

/** Report MOTION_FAILED (stall) when a wheel driven hard has not moved for
    MOTION_STALL_MS. */
static void check_stall (motion_core_t * p_mc, uint32_t now_ms)
{
    bool const b_l =
        (app_types_abs_i32(p_mc->duty_l) > MOTION_STALL_DUTY)
        && (app_types_abs_i32(p_mc->v_l) < MOTION_STALL_SPEED_MM_S);
    bool const b_r =
        (app_types_abs_i32(p_mc->duty_r) > MOTION_STALL_DUTY)
        && (app_types_abs_i32(p_mc->v_r) < MOTION_STALL_SPEED_MM_S);

    if (b_l || b_r)
    {
        if (!p_mc->b_stall_timing)
        {
            p_mc->b_stall_timing = true;
            p_mc->stall_since_ms = now_ms;
        }
        else if ((app_types_elapsed_ms(now_ms, p_mc->stall_since_ms)
                  >= MOTION_STALL_MS)
                 && !p_mc->b_stall_reported)
        {
            p_mc->b_stall_reported = true;
            push_event(p_mc, MOTION_EV_STALL, 0u, 0u);
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

/** MANEUVER: set sp_v / sp_w from the profile.  Returns false when the
   manoeuvre ended this step (outputs already braked). */
static bool step_maneuver (motion_core_t * p_mc, uint32_t now_ms,
                           uint32_t dt_ms)
{
    int64_t const rel_l   = p_mc->dist_l_um - p_mc->m_start_l_um;
    int64_t const rel_r   = p_mc->dist_r_um - p_mc->m_start_r_um;
    bool const    b_move  = (MOTION_CMD_MOVE == p_mc->m_cmd);
    bool          b_drive = false;
    int64_t       progress;
    int64_t       remaining;
    int64_t       tol;

    if (MOTION_PH_SETTLE == p_mc->phase)
    {
        brake(p_mc);

        if (app_types_elapsed_ms(now_ms, p_mc->m_settle_ms) >= MOTION_SETTLE_MS)
        {
            end_maneuver(p_mc, true);
        }
    }
    else
    {
        progress  = b_move ? ((p_mc->m_dir * (rel_l + rel_r)) / 2)
                           : ((p_mc->m_dir * (rel_r - rel_l)) / 2);
        remaining = p_mc->m_target_um - progress;
        tol = b_move ? MOTION_POS_TOL_UM : turn_arc_um(MOTION_ANG_TOL_CDEG);

        if (remaining <= tol)
        {
            p_mc->phase       = MOTION_PH_SETTLE;
            p_mc->m_settle_ms = now_ms;
            brake(p_mc);
        }
        else if (app_types_elapsed_ms(now_ms, p_mc->m_start_ms)
                 >= p_mc->m_timeout_ms)
        {
            brake(p_mc);
            end_maneuver(p_mc, false);
        }
        else
        {
            maneuver_drive(p_mc, rel_l, rel_r, remaining, dt_ms);
            b_drive = true;
        }
    }

    return b_drive;
}

/** step_maneuver() while driving: speed profile and wheel set-points from the
    wheel travel since the start (rel_l, rel_r) and the remaining distance. */
static void maneuver_drive (motion_core_t * p_mc, int64_t rel_l, int64_t rel_r,
                            int64_t remaining, uint32_t dt_ms)
{
    bool const b_move = (MOTION_CMD_MOVE == p_mc->m_cmd);
    int32_t    v_acc;
    int32_t    v_dec;
    int32_t    speed;

    /* Trapezoid: limited by acceleration, by the speed from which the
       remaining distance still allows braking, and by the cruise speed. */
    /* Casts (int32_t)(((uint32_t)MOTION_ACCEL_M ...)): an acceleration step:
       MOTION_ACCEL_MM_S2 x dt_ms (<= 100 ms) / 1000;
       (uint32_t)MOTION_ACCEL_MM_S2, (int64_t)MOTION_ACCEL_MM_S2: a constant
       that fits; (int32_t)app_math_isqrt64(...): the square root of a speed
       squared in (mm/s)^2: a speed below 2^31; (uint64_t)((2 * ...)): remaining
       >= 0 here, so the product is not negative. */
    v_acc = p_mc->m_prof_v
            + (int32_t)(((uint32_t)MOTION_ACCEL_MM_S2 * dt_ms) / 1000u);
    v_dec = (int32_t)app_math_isqrt64(
        (uint64_t)((2 * (int64_t)MOTION_ACCEL_MM_S2 * remaining) / 1000));
    speed = app_types_min_i32(app_types_min_i32(v_acc, v_dec), p_mc->m_vmax);
    speed = app_types_max_i32(speed, MOTION_PROFILE_V_MIN);

    if (speed >= p_mc->m_vmax)
    {
        p_mc->phase = MOTION_PH_CRUISE;
    }
    else if (v_dec <= v_acc)
    {
        p_mc->phase = MOTION_PH_DECEL;
    }
    else
    {
        p_mc->phase = MOTION_PH_ACCEL;
    }

    p_mc->m_prof_v = speed;

    if (b_move)
    {
        /* Straight-line correction: steer against the wheel-distance
         * difference. */
        /* Cast (int32_t)(((rel_l - rel_r) * ...)): the wheel travel difference
           of one move (< 1 m) x MOTION_STRAIGHT_K / 1000. */
        p_mc->sp_v = p_mc->m_dir * speed;
        p_mc->sp_w = app_types_clamp_i32(
            (int32_t)(((rel_l - rel_r) * MOTION_STRAIGHT_K) / 1000), -1000,
            1000);
    }
    else
    {
        /* Spin about the centre; pull any translation back to zero. */
        /* Casts (int32_t)(((int64_t)p_mc->m_dir * ...)): a turn rate from speed
           (<= MOTION_V_MAX_MM_S) and the track width; (int64_t)p_mc->m_dir:
           every int8_t value fits; (int32_t)((-4 * ((rel_l + rel_r) / ...))): a
           correction from the travel of one move (< 1 m) / 1000. */
        p_mc->sp_w =
            (int32_t)(((int64_t)p_mc->m_dir * speed * 2000) / MOTION_TRACK_MM);
        p_mc->sp_v = (int32_t)((-4 * ((rel_l + rel_r) / 2)) / 1000);
    }
}

/** Width of the first argument field of a packed command of this kind, bits. */
static uint8_t p0_bits (uint8_t cmd)
{
    uint8_t bits;

    if (MOTION_CMD_DRIVE == cmd)
    {
        bits = DRIVE_V_BITS;
    }
    else if ((MOTION_CMD_MOVE == cmd) || (MOTION_CMD_TURN == cmd))
    {
        bits = AMOUNT_BITS;
    }
    else
    {
        bits = (MOTION_CMD_SET_AUTH == cmd) ? CMD_SRC_BITS : 0u;
    }

    return bits;
}

/*** end of file ***/
