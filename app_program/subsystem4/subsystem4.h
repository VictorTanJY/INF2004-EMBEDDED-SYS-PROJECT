/*
 *  subsystem4.h
 *  Subsystem 4 -- IMU terrain monitoring (ImuTask) and fault supervision
 *  (SafetyTask).  Owner: Buddy 4 (Purvez).
 *
 *  PUBLIC interface of subsystem 4.  Week 6 design review: Sections 3,
 *  4 and 6.3.
 *
 *  STATUS: interface + object creation only (shared base).
 */

#ifndef SUBSYSTEM4_H
#define SUBSYSTEM4_H

#include <stdint.h>
#include "../app.h"

#define S4_PRI_SAFETY           5           /* SafetyTask, core 1 (highest app task) */
#define S4_CORE_SAFETY          APP_CORE1
#define S4_PRI_IMU              10          /* ImuTask, core 0 */
#define S4_CORE_IMU             APP_CORE0

#define S4_PIN_IMU_SDA          2u          /* I2C1 */
#define S4_PIN_IMU_SCL          3u
#define S4_PIN_IMU_INT1         19u         /* high-g interrupt, bank-0 IRQ on core 1 */

/* ---------------------------------------------------------------------
 *  IF-05  flg_safety: fault bits (owned by S4).
 *
 *  Setters use tk_set_flg().  Only SafetyTask waits on this flag, with
 *  TWF_ORW | TWF_BITCLR on SAFETY_BRAKE_MASK, so it takes the bits that
 *  woke it and cannot spin on bits that stay set (Tribunal 1 gap 1).
 *  It ORs them into its latched-fault word (safety_get_latched()),
 *  cleared only by an operator RESET.  Every other task reads the flag
 *  with tk_ref_flg(), never tk_wai_flg().
 *
 *  Report-only bits (outside SAFETY_BRAKE_MASK) never wake SafetyTask;
 *  the task that sets one also clears it (tk_clr_flg) when the cause ends.
 * --------------------------------------------------------------------- */
#define SAFETY_FLT_IMPACT       0x0001u     /* imu_int1_isr                  */
#define SAFETY_FLT_TOO_CLOSE    0x0002u     /* ObstacleTask, d < 8 cm        */
#define SAFETY_FLT_STALL        0x0004u     /* MotionTask                    */
#define SAFETY_FLT_TILT         0x0008u     /* ImuTask, |pitch| or |roll| > 30 deg */
#define SAFETY_FLT_ESTOP        0x0010u     /* CommsTask, remote E-stop      */
#define SAFETY_FLT_HEARTBEAT    0x0020u     /* SafetyTask itself (latched only) */
#define SAFETY_FLT_ENC_FAULT    0x0040u     /* report-only: MotionTask       */
#define SAFETY_FLT_IMU_FAULT    0x0080u     /* report-only: ImuTask          */
#define SAFETY_FLT_NO_ECHO      0x0100u     /* report-only: ObstacleTask     */
#define SAFETY_FLT_WIFI_LOSS    0x0200u     /* report-only: CommsTask        */

#define SAFETY_BRAKE_MASK       (SAFETY_FLT_IMPACT | SAFETY_FLT_TOO_CLOSE | \
                                 SAFETY_FLT_STALL  | SAFETY_FLT_TILT      | \
                                 SAFETY_FLT_ESTOP)

IMPORT ID g_flg_safety;         /* flg_safety in the design documents */

/* Latched faults (SAFETY_FLT_* bits), written only by SafetyTask. */
IMPORT uint32_t safety_get_latched(void);

/* ---------------------------------------------------------------------
 *  Task heartbeats.  Each monitored task calls safety_heartbeat() on
 *  every pass.  One 32-bit word per task, one writer each: an aligned
 *  word store is indivisible, so no lock is needed.  SafetyTask checks
 *  them every 20 ms against the timeouts below (Week 6 Section 3).
 * --------------------------------------------------------------------- */
typedef enum
{
    SAFETY_HB_MOTION   = 0,     /* timeout 50 ms  */
    SAFETY_HB_LINE     = 1,     /* timeout 50 ms  */
    SAFETY_HB_MISSION  = 2,     /* timeout 300 ms */
    SAFETY_HB_OBSTACLE = 3,     /* timeout 300 ms */
    SAFETY_HB_IMU      = 4,     /* timeout 100 ms */
    SAFETY_HB_COUNT    = 5
} safety_hb_t;

IMPORT void safety_heartbeat(safety_hb_t task);
IMPORT uint32_t safety_heartbeat_count(safety_hb_t task);

/* ---------------------------------------------------------------------
 *  IF-08  IMU snapshot (written by ImuTask).
 * --------------------------------------------------------------------- */
typedef enum
{
    IMU_HUMP_FLAT       = 0,
    IMU_HUMP_CLIMBING   = 1,
    IMU_HUMP_CREST      = 2,
    IMU_HUMP_DESCENDING = 3
} imu_hump_state_t;

typedef enum
{
    IMU_MOTION_STATIONARY   = 0,
    IMU_MOTION_ACCELERATING = 1,
    IMU_MOTION_TURNING      = 2,
    IMU_MOTION_CLIMBING     = 3,
    IMU_MOTION_DESCENDING   = 4,
    IMU_MOTION_IMPACT       = 5,
    IMU_MOTION_CRUISING     = 6
} imu_motion_t;

typedef struct
{
    uint32_t t_ms;
    int16_t  pitch_cdeg;        /* pitch, 0.01 deg, + nose up             */
    int16_t  roll_cdeg;
    int16_t  heading_cdeg;      /* 0..35999                               */
    int16_t  turn_rate_cdeg_s;  /* + left                                 */
    uint16_t hump_max_mm;       /* highest hump peak of the run           */
    uint16_t last_hump_mm;      /* peak of the last completed hump        */
    uint8_t  hump_state;        /* imu_hump_state_t                       */
    uint8_t  motion;            /* imu_motion_t                           */
    uint8_t  ok;                /* 1 = IMU readings valid                 */
    uint8_t  reserved;
} imu_snap_t;

IMPORT void imu_get_snap(imu_snap_t * p_out);           /* any task       */
IMPORT void imu_put_snap(imu_snap_t const * p_in);      /* ImuTask only   */

#endif /* SUBSYSTEM4_H */
