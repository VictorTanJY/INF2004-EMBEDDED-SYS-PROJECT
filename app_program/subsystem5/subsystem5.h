/*
 *  subsystem5.h
 *  Subsystem 5 -- Obstacle scanning and avoidance (ObstacleTask) and the
 *  system state machine (MissionTask).  Owner: Buddy 5 (Ryan).
 *
 *  PUBLIC interface of subsystem 5.  Week 6 design review: Sections 3,
 *  4, 5 and 6.4.
 *
 *  STATUS: interface + object creation only (shared base).
 */

#ifndef SUBSYSTEM5_H
#define SUBSYSTEM5_H

#include <stdint.h>
#include "../app.h"

#define S5_PRI_MISSION          8           /* MissionTask, core 1 */
#define S5_PRI_OBSTACLE         9           /* ObstacleTask, core 1 */
#define S5_CORE                 APP_CORE1

#define S5_PIN_US_TRIG          0u          /* GP0 (free with CRASH_REPORT=0) */
#define S5_PIN_US_ECHO          1u          /* GP1, bank-0 IRQ on core 1 */
#define S5_PIN_SERVO            15u         /* PWM7 B, 50 Hz (DIV >= 38.2 needed) */

/* ---------------------------------------------------------------------
 *  System states (Week 6 Section 5).  11 states.
 * --------------------------------------------------------------------- */
typedef enum
{
    MISSION_ST_BOOT              = 0,
    MISSION_ST_IDLE              = 1,
    MISSION_ST_CALIBRATING       = 2,
    MISSION_ST_LINE_FOLLOW       = 3,
    MISSION_ST_EXECUTE_NAV       = 4,
    MISSION_ST_OBSTACLE_HANDLING = 5,
    MISSION_ST_AVOIDING          = 6,
    MISSION_ST_LINE_RECOVERY     = 7,
    MISSION_ST_LINE_SEARCH       = 8,
    MISSION_ST_MISSION_COMPLETE  = 9,
    MISSION_ST_FAULT             = 10,
    MISSION_ST_COUNT             = 11
} mission_state_t;

/* ---------------------------------------------------------------------
 *  IF-06  mbf_mission: every event to MissionTask (owned by S5).
 *  All senders use TMO_POL (CommsTask and the control tasks must never
 *  block).  MissionTask is the only reader.
 * --------------------------------------------------------------------- */
typedef enum
{
    MISSION_EV_NONE            = 0,
    MISSION_EV_BTN_START_STOP  = 1,     /* HmiTask: GP20 press (MissionTask decides start or stop) */
    MISSION_EV_BTN_CALIBRATE   = 2,     /* HmiTask: GP21 short press */
    MISSION_EV_BTN_RESET       = 3,     /* HmiTask: GP21 long press  */
    MISSION_EV_REMOTE_START    = 4,     /* CommsTask                 */
    MISSION_EV_REMOTE_STOP     = 5,
    MISSION_EV_REMOTE_RESET    = 6,
    MISSION_EV_REMOTE_SET      = 7,     /* a = param id, data = value */
    MISSION_EV_BARCODE         = 8,     /* data = character           */
    MISSION_EV_JUNCTION        = 9,
    MISSION_EV_LINE_LOST       = 10,
    MISSION_EV_LINE_FOUND      = 11,
    MISSION_EV_MOTION_DONE     = 12,    /* a = seq                    */
    MISSION_EV_MOTION_FAILED   = 13,    /* a = seq                    */
    MISSION_EV_OBSTACLE        = 14,    /* data = distance [mm]       */
    MISSION_EV_SLOW_DOWN       = 15,    /* data = distance [mm]       */
    MISSION_EV_PROFILE_DONE    = 16,    /* profile in the obstacle snapshot */
    MISSION_EV_CONTINUE        = 17,
    MISSION_EV_BYPASS_DONE     = 18,
    MISSION_EV_RECOVERY_DONE   = 19,
    MISSION_EV_RECOVERY_FAILED = 20,
    MISSION_EV_BLOCKED_TIMEOUT = 21,
    MISSION_EV_HUMP_START      = 22,
    MISSION_EV_HUMP_END        = 23,    /* data = peak [mm]           */
    MISSION_EV_MOTION_CLASS    = 24,    /* data = imu_motion_t        */
    MISSION_EV_FAULT           = 25,    /* data = SAFETY_FLT_* bits   */
    MISSION_EV_CAL_DONE        = 26,    /* a = source subsystem       */
    MISSION_EV_COUNT           = 27
} mission_ev_t;

typedef struct
{
    uint8_t  type;              /* mission_ev_t                          */
    uint8_t  src;               /* APP_SRC_*                             */
    uint16_t a;                 /* small argument (seq, param id, ...)   */
    int32_t  data;              /* see mission_ev_t                      */
} mission_evt_t;                /* 8 bytes */

#define S5_MBF_MISSION_DEPTH    16u     /* events (Week 6: 16 deep)      */

IMPORT ID g_mbf_mission;        /* mbf_mission in the design documents */

/* Post an event without waiting.  E_OK, or E_TMOUT when full (dropped). */
IMPORT ER mission_post(uint8_t type, uint8_t src, uint16_t a, int32_t data);

/* ---------------------------------------------------------------------
 *  IF-09  mbf_scan + flg_obst: requests from MissionTask to ObstacleTask.
 *  MissionTask sends the request, then sets OBST_FLG_REQ, because
 *  ObstacleTask blocks only on flg_obst.
 * --------------------------------------------------------------------- */
typedef enum
{
    SCAN_REQ_MONITOR      = 1,  /* servo at 90 deg, ping every 60 ms       */
    SCAN_REQ_PROFILE      = 2,  /* coarse + fine scan, then PROFILE_DONE   */
    SCAN_REQ_BYPASS_LEFT  = 3,
    SCAN_REQ_BYPASS_RIGHT = 4,
    SCAN_REQ_RECOVER      = 5,  /* seek and align with the original line   */
    SCAN_REQ_STOP         = 6
} scan_req_code_t;

typedef struct
{
    uint8_t type;               /* scan_req_code_t */
    uint8_t seq;
    int16_t param;
} scan_req_t;                   /* 4 bytes */

#define S5_MBF_SCAN_DEPTH       4u

#define OBST_FLG_RANGE          0x01u   /* range_cyc, every 60 ms */
#define OBST_FLG_ECHO           0x02u   /* echo_isr               */
#define OBST_FLG_REQ            0x04u   /* MissionTask request    */

IMPORT ID g_mbf_scan;           /* mbf_scan */
IMPORT ID g_flg_obst;           /* flg_obst */

/* ---------------------------------------------------------------------
 *  IF-10  Mission and obstacle snapshots.
 * --------------------------------------------------------------------- */
typedef struct
{
    uint32_t t_ms;
    uint8_t  state;             /* mission_state_t                       */
    uint8_t  auth;              /* motion_src_t granted to MotionTask    */
    char     pending_nav;       /* latched barcode command, 0 = none     */
    uint8_t  calibrated;        /* 1 = IR + IMU calibration valid        */
} mission_snap_t;

typedef struct
{
    uint32_t t_ms;
    uint16_t d_mm;              /* last median range; 0xFFFF = none      */
    uint16_t w_mm;              /* profiled width                        */
    uint16_t clr_l_mm;          /* left / right clearance                */
    uint16_t clr_r_mm;
    uint8_t  valid;             /* 1 = profile fields valid              */
    uint8_t  reserved[3];
} obst_snap_t;

#define OBST_RANGE_NONE         0xFFFFu

IMPORT void mission_get_snap(mission_snap_t * p_out);
IMPORT void mission_put_snap(mission_snap_t const * p_in);     /* MissionTask only */
IMPORT void obst_get_snap(obst_snap_t * p_out);
IMPORT void obst_put_snap(obst_snap_t const * p_in);           /* ObstacleTask only */

#endif /* SUBSYSTEM5_H */
