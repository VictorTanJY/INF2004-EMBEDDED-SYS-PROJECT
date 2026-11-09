/*
 *  motion_api.c
 *  Subsystem 2 -- the named motion API.  See motion_api.h.
 *  Each function checks its range and posts one MOTION_CMD_* command.
 */

#include "motion_api.h"

static int post_signed(uint8_t src, uint8_t seq, uint8_t cmd, int32_t sign,
                       uint16_t amount, uint16_t limit, uint16_t rate)
{
    /* amount 0 is allowed: MotionTask completes it at once (MOTION_DONE). */
    if ((amount > limit) || (rate == 0u) || (rate > (uint16_t)INT16_MAX))
    {
        return MOTION_API_E_PAR;
    }
    return motion_post(src, seq, cmd, (int16_t)(sign * (int32_t)amount), (int16_t)rate);
}

int motion_move_forward(uint8_t src, uint8_t seq, uint16_t distance_mm, uint16_t speed_mm_s)
{
    return post_signed(src, seq, (uint8_t)MOTION_CMD_MOVE, 1, distance_mm,
                       (uint16_t)MOTION_API_MAX_MM, speed_mm_s);
}

int motion_move_backward(uint8_t src, uint8_t seq, uint16_t distance_mm, uint16_t speed_mm_s)
{
    return post_signed(src, seq, (uint8_t)MOTION_CMD_MOVE, -1, distance_mm,
                       (uint16_t)MOTION_API_MAX_MM, speed_mm_s);
}

int motion_turn_left(uint8_t src, uint8_t seq, uint16_t angle_ddeg, uint16_t rate_deg_s)
{
    return post_signed(src, seq, (uint8_t)MOTION_CMD_TURN, 1, angle_ddeg,
                       (uint16_t)MOTION_API_MAX_DDEG, rate_deg_s);
}

int motion_turn_right(uint8_t src, uint8_t seq, uint16_t angle_ddeg, uint16_t rate_deg_s)
{
    return post_signed(src, seq, (uint8_t)MOTION_CMD_TURN, -1, angle_ddeg,
                       (uint16_t)MOTION_API_MAX_DDEG, rate_deg_s);
}

int motion_stop(uint8_t src, uint8_t seq)
{
    return motion_post(src, seq, (uint8_t)MOTION_CMD_STOP, 0, 0);
}

int motion_drive(uint8_t src, uint8_t seq, int16_t v_mm_s, int16_t omega_mrad_s)
{
    return motion_post(src, seq, (uint8_t)MOTION_CMD_DRIVE, v_mm_s, omega_mrad_s);
}
