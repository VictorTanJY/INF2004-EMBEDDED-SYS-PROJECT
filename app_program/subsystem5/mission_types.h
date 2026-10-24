/*
 *  mission_types.h
 *  Subsystem 5 -- system states, mission events, scan requests and the
 *  mission / obstacle snapshots (IF-06, IF-09, IF-10).
 *  Owner: Buddy 5 (Ryan).  Pure C (no kernel).
 */

#ifndef MISSION_TYPES_H
#define MISSION_TYPES_H

#include <stdint.h>

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
 *  IF-06  mbf_mission events.  All senders use TMO_POL.
 * --------------------------------------------------------------------- */
typedef enum
{
    MISSION_EV_NONE            = 0,
    MISSION_EV_BTN_START_STOP  = 1,     /* HmiTask: GP20 press (start or stop by state) */
    MISSION_EV_BTN_CALIBRATE   = 2,     /* HmiTask: GP21 short press */
    MISSION_EV_BTN_RESET       = 3,     /* HmiTask: GP21 long press  */
    MISSION_EV_REMOTE_START    = 4,     /* CommsTask                 */
    MISSION_EV_REMOTE_STOP     = 5,
    MISSION_EV_REMOTE_RESET    = 6,
    MISSION_EV_REMOTE_SET      = 7,     /* a = param id, data = value */
    MISSION_EV_BARCODE         = 8,     /* data = decoded character (any valid Code 39) */
    MISSION_EV_JUNCTION        = 9,
    MISSION_EV_LINE_LOST       = 10,
    MISSION_EV_LINE_FOUND      = 11,
    MISSION_EV_MOTION_DONE     = 12,    /* a = seq                    */
    MISSION_EV_MOTION_FAILED   = 13,    /* a = seq                    */
    MISSION_EV_OBSTACLE        = 14,    /* data = distance [mm]       */
    MISSION_EV_SLOW_DOWN       = 15,    /* data = distance [mm]       */
    MISSION_EV_PROFILE_DONE    = 16,    /* a = obst_plan_t, data = width [mm] */
    MISSION_EV_CONTINUE        = 17,
    MISSION_EV_BYPASS_DONE     = 18,
    MISSION_EV_RECOVERY_DONE   = 19,
    MISSION_EV_RECOVERY_FAILED = 20,
    MISSION_EV_BLOCKED_TIMEOUT = 21,
    MISSION_EV_HUMP_START      = 22,
    MISSION_EV_HUMP_END        = 23,    /* data = peak [mm]           */
    MISSION_EV_MOTION_CLASS    = 24,    /* data = imu_motion_t (telemetry only) */
    MISSION_EV_FAULT           = 25,    /* data = SAFETY_FLT_* bits   */
    MISSION_EV_CAL_DONE        = 26,    /* a = source subsystem, data = 1 ok / 0 failed */
    MISSION_EV_REMOTE_MAP      = 27,    /* data = 4 packed chars, see MISSION_NAV_* */
    MISSION_EV_SAFETY_OK       = 28,    /* SafetyTask cleared its latch after RESET */
    MISSION_EV_COUNT           = 29
} mission_ev_t;

typedef struct
{
    uint8_t  type;              /* mission_ev_t                          */
    uint8_t  src;               /* APP_SRC_*                             */
    uint16_t a;                 /* small argument (seq, param id, ...)   */
    int32_t  data;              /* see mission_ev_t                      */
} mission_evt_t;                /* 8 bytes */

/* ---------------------------------------------------------------------
 *  Navigation commands (FR4).  The character -> movement mapping is NOT
 *  hard-coded: the instructor gives the four characters just before the
 *  demo and the laptop sends them over MQTT (course forum, 24 Oct):
 *     {"id":N,"cmd":"map","left":"A","right":"F","straight":"Z","uturn":"W"}
 *  MISSION_EV_REMOTE_MAP carries them packed: byte k = character for
 *  movement k (MISSION_NAV_LEFT = byte 0 ... MISSION_NAV_UTURN = byte 3).
 *  Until a map arrives, decoded barcodes are reported but never acted on.
 * --------------------------------------------------------------------- */
typedef enum
{
    MISSION_NAV_LEFT     = 0,
    MISSION_NAV_RIGHT    = 1,
    MISSION_NAV_STRAIGHT = 2,
    MISSION_NAV_UTURN    = 3,
    MISSION_NAV_COUNT    = 4,
    MISSION_NAV_NONE     = 0xFF
} mission_nav_t;

/* MissionTask's own fault causes (mission_snap_t.cause), besides SafetyTask's. */
typedef enum
{
    MISSION_CAUSE_NONE         = 0,
    MISSION_CAUSE_SAFETY       = 1,     /* see mission_snap_t.fault_bits */
    MISSION_CAUSE_SELFTEST     = 2,     /* BOOT: a subsystem's hardware did not start */
    MISSION_CAUSE_BLOCKED      = 3,     /* BLOCKED_TIMEOUT: no path for 10 s */
    MISSION_CAUSE_LINE_LOST    = 4,     /* LINE_SEARCH exhausted      */
    MISSION_CAUSE_MBF_OVERFLOW = 5,     /* an event to mbf_mission was dropped */
    MISSION_CAUSE_TIMEOUT      = 6      /* a state exceeded its supervision time */
} mission_cause_t;

/* mission_snap_t.calibrated */
#define MISSION_CAL_IR          0x01u
#define MISSION_CAL_IMU         0x02u
#define MISSION_CAL_ALL         (MISSION_CAL_IR | MISSION_CAL_IMU)

/* mission_snap_t.cal_phase: what CALIBRATING asks the sensor tasks to do */
typedef enum
{
    MISSION_CAL_PH_NONE   = 0,
    MISSION_CAL_PH_STILL  = 1,          /* stand still 1 s: IMU level offsets          */
    MISSION_CAL_PH_SPIN   = 2,          /* 360 deg spin: IR min/max, magnetometer      */
    MISSION_CAL_PH_FINISH = 3           /* compute and report MISSION_EV_CAL_DONE      */
} mission_cal_phase_t;

/* ---------------------------------------------------------------------
 *  IF-10  Mission snapshot (written by MissionTask).  Besides the state,
 *  it carries the run parameters every task reads (set over MQTT in IDLE).
 * --------------------------------------------------------------------- */
typedef struct
{
    uint32_t t_ms;
    uint32_t fault_bits;        /* SAFETY_FLT_* bits of the last FAULT    */
    uint16_t cruise_mm_s;       /* line-following speed (param 1)         */
    uint16_t speed_cap_mm_s;    /* current cap: slow-down / hump / nav    */
    uint16_t slow_mm;           /* obstacle slow-down range (param 2)     */
    uint16_t stop_mm;           /* obstacle stop range (param 3)          */
    uint16_t lost_mm;           /* line-lost length (param 4)             */
    uint16_t junction_mm;       /* junction length (param 5)              */
    uint16_t runs;              /* runs started since boot                */
    uint8_t  state;             /* mission_state_t                        */
    uint8_t  auth;              /* motion_src_t granted to MotionTask     */
    uint8_t  pending_nav;       /* mission_nav_t latched for the next junction */
    char     pending_char;      /* the barcode character that set it, 0 = none */
    char     nav_map[MISSION_NAV_COUNT];    /* 0 = not mapped yet          */
    uint8_t  calibrated;        /* MISSION_CAL_* bits                     */
    uint8_t  cal_phase;         /* mission_cal_phase_t                    */
    uint8_t  cause;             /* mission_cause_t of the last FAULT      */
    uint8_t  reserved;
} mission_snap_t;

/* ---------------------------------------------------------------------
 *  IF-09  mbf_scan requests (MissionTask -> ObstacleTask), announced by
 *  OBST_FLG_REQ in flg_obst.
 * --------------------------------------------------------------------- */
typedef enum
{
    SCAN_REQ_NONE         = 0,
    SCAN_REQ_MONITOR      = 1,  /* servo at 90 deg, ping every 60 ms       */
    SCAN_REQ_PROFILE      = 2,  /* coarse + fine scan + plan, then PROFILE_DONE */
    SCAN_REQ_BYPASS_LEFT  = 3,
    SCAN_REQ_BYPASS_RIGHT = 4,
    SCAN_REQ_RECOVER      = 5,  /* seek and align with the original line   */
    SCAN_REQ_STOP         = 6,  /* stop everything, servo to 90 deg        */
    SCAN_REQ_BENCH_SERVO  = 7   /* APP_BENCH: servo to param deg, keep ranging */
} scan_req_code_t;

typedef struct
{
    uint8_t type;               /* scan_req_code_t */
    uint8_t seq;
    int16_t param;
} scan_req_t;                   /* 4 bytes */

/* ---------------------------------------------------------------------
 *  Obstacle snapshot (written by ObstacleTask).  Lateral positions are
 *  relative to the robot's centre line, + left, in mm.
 * --------------------------------------------------------------------- */
typedef enum
{
    OBST_PLAN_NONE     = 0,
    OBST_PLAN_CONTINUE = 1,     /* nothing inside the robot's corridor    */
    OBST_PLAN_LEFT     = 2,
    OBST_PLAN_RIGHT    = 3,
    OBST_PLAN_BLOCKED  = 4
} obst_plan_t;

typedef enum
{
    OBST_ST_IDLE        = 0,
    OBST_ST_MONITOR     = 1,
    OBST_ST_COARSE_SCAN = 2,
    OBST_ST_FINE_SCAN   = 3,
    OBST_ST_PLAN        = 4,
    OBST_ST_REVERSE     = 5,
    OBST_ST_BLOCKED     = 6,
    OBST_ST_BYPASS      = 7,
    OBST_ST_SEEK_LINE   = 8,
    OBST_ST_ALIGN       = 9
} obst_state_t;

#define OBST_RANGE_NONE         0xFFFFu

typedef struct
{
    uint32_t t_ms;
    uint32_t pings;             /* pings sent since boot                  */
    uint16_t d_mm;              /* last median range ahead; NONE = no target */
    uint16_t w_mm;              /* profiled width (beam-corrected)        */
    uint16_t closest_mm;        /* closest profiled point                 */
    uint16_t clr_l_mm;          /* free space beside each edge            */
    uint16_t clr_r_mm;
    int16_t  left_lat_mm;       /* lateral position of the left edge      */
    int16_t  right_lat_mm;      /* ... and of the right edge              */
    uint16_t no_echo;           /* consecutive pings with no echo         */
    uint8_t  valid;             /* 1 = profile fields valid               */
    uint8_t  plan;              /* obst_plan_t                            */
    uint8_t  state;             /* obst_state_t                           */
    uint8_t  retries;           /* reverse-and-rescan attempts            */
} obst_snap_t;

#endif /* MISSION_TYPES_H */
