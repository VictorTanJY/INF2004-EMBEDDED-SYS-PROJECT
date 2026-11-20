/** @file motion_api.c
 *
 *  @brief Subsystem 2 -- the named motion API.  See motion_api.h.
 *  Each function checks its range and posts one MOTION_CMD_* command.
 */

#include "motion_api.h"

static motion_api_err_t post_signed(uint8_t src, uint8_t seq, uint8_t cmd,
                                    int32_t sign, uint16_t amount,
                                    uint16_t limit, uint16_t rate);

motion_api_err_t motion_api_move_forward (uint8_t src, uint8_t seq,
                                          uint16_t distance_mm,
                                          uint16_t speed_mm_s)
{
    return post_signed(src, seq, MOTION_CMD_MOVE, 1, distance_mm,
                       MOTION_API_MAX_MM, speed_mm_s);
}

motion_api_err_t motion_api_move_backward (uint8_t src, uint8_t seq,
                                           uint16_t distance_mm,
                                           uint16_t speed_mm_s)
{
    return post_signed(src, seq, MOTION_CMD_MOVE, -1, distance_mm,
                       MOTION_API_MAX_MM, speed_mm_s);
}

motion_api_err_t motion_api_turn_left (uint8_t src, uint8_t seq,
                                       uint16_t angle_ddeg, uint16_t rate_deg_s)
{
    return post_signed(src, seq, MOTION_CMD_TURN, 1, angle_ddeg,
                       MOTION_API_MAX_DDEG, rate_deg_s);
}

motion_api_err_t motion_api_turn_right (uint8_t src, uint8_t seq,
                                        uint16_t angle_ddeg,
                                        uint16_t rate_deg_s)
{
    return post_signed(src, seq, MOTION_CMD_TURN, -1, angle_ddeg,
                       MOTION_API_MAX_DDEG, rate_deg_s);
}

motion_api_err_t motion_api_stop (uint8_t src, uint8_t seq)
{
    return subsystem2_post(src, seq, MOTION_CMD_STOP, 0, 0);
}

motion_api_err_t motion_api_drive (uint8_t src, uint8_t seq, int16_t v_mm_s,
                                   int16_t omega_mrad_s)
{
    return subsystem2_post(src, seq, MOTION_CMD_DRIVE, v_mm_s, omega_mrad_s);
}

/** Check amount against limit and rate against MOTION_API_MAX_RATE, then post
    cmd with sign applied to amount.  Returns subsystem2_post()'s result, or
    MOTION_API_E_PAR. */
static motion_api_err_t post_signed (uint8_t src, uint8_t seq, uint8_t cmd,
                                     int32_t sign, uint16_t amount,
                                     uint16_t limit, uint16_t rate)
{
    motion_api_err_t err = MOTION_API_E_PAR;

    /* amount 0 is allowed: MotionTask completes it at once (MOTION_DONE). */
    /* Cast (uint16_t)MOTION_API_MAX_RATE: a constant that fits. */
    if ((amount <= limit) && (rate != 0u)
        && (rate <= (uint16_t)MOTION_API_MAX_RATE))
    {
        /* Casts (int16_t)(sign * (int32_t)amount): amount <= limit (<= 4095,
           tested above); (int32_t)amount: every uint16_t value fits;
           (int16_t)rate: rate <= MOTION_API_MAX_RATE (511, tested above). */
        err = subsystem2_post(src, seq, cmd, (int16_t)(sign * (int32_t)amount),
                              (int16_t)rate);
    }
    return err;
}

/*** end of file ***/
