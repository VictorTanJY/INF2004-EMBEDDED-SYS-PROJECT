/** @file mission_types.h
 *
 *  @brief Subsystem 5 -- system states, mission events, scan requests and the
 *  mission / obstacle snapshots (IF-06, IF-09, IF-10).
 *  Owner: Buddy 5 (Ryan).  Pure C (no kernel).
 */

#ifndef MISSION_TYPES_H
#define MISSION_TYPES_H

#include <stdint.h>

/* ---------------------------------------------------------------------
 *  System states (Week 6 Section 5).  11 states.
 * --------------------------------------------------------------------- */
typedef uint8_t mission_types_state_t; /* one of the constants below */
#define MISSION_ST_BOOT              0u
#define MISSION_ST_IDLE              1u
#define MISSION_ST_CALIBRATING       2u
#define MISSION_ST_LINE_FOLLOW       3u
#define MISSION_ST_EXECUTE_NAV       4u
#define MISSION_ST_OBSTACLE_HANDLING 5u
#define MISSION_ST_AVOIDING          6u
#define MISSION_ST_LINE_RECOVERY     7u
#define MISSION_ST_LINE_SEARCH       8u
#define MISSION_ST_MISSION_COMPLETE  9u
#define MISSION_ST_FAULT             10u
#define MISSION_ST_COUNT             11u

/* ---------------------------------------------------------------------
 *  IF-06  mbf_mission events.  All senders use TMO_POL.
 * --------------------------------------------------------------------- */
typedef uint8_t mission_types_ev_t; /* one of the constants below */
#define MISSION_EV_NONE 0u
/* HmiTask: GP20 press (start or stop by state) */
#define MISSION_EV_BTN_START_STOP 1u
#define MISSION_EV_BTN_CALIBRATE  2u /* HmiTask: GP21 short press */
#define MISSION_EV_BTN_RESET      3u /* HmiTask: GP21 long press */
#define MISSION_EV_REMOTE_START   4u /* CommsTask */
#define MISSION_EV_REMOTE_STOP    5u
#define MISSION_EV_REMOTE_RESET   6u
#define MISSION_EV_REMOTE_SET     7u /* a = param id, data = value */
/* data = decoded character (any valid Code 39) */
#define MISSION_EV_BARCODE       8u
#define MISSION_EV_JUNCTION      9u
#define MISSION_EV_LINE_LOST     10u
#define MISSION_EV_LINE_FOUND    11u
#define MISSION_EV_MOTION_DONE   12u /* a = seq */
#define MISSION_EV_MOTION_FAILED 13u /* a = seq */
#define MISSION_EV_OBSTACLE      14u /* data = distance [mm] */
#define MISSION_EV_SLOW_DOWN     15u /* data = distance [mm] */
/* a = mission_types_obst_plan_t, data = width [mm] */
#define MISSION_EV_PROFILE_DONE    16u
#define MISSION_EV_CONTINUE        17u
#define MISSION_EV_BYPASS_DONE     18u
#define MISSION_EV_RECOVERY_DONE   19u
#define MISSION_EV_RECOVERY_FAILED 20u
#define MISSION_EV_BLOCKED_TIMEOUT 21u
#define MISSION_EV_HUMP_START      22u
#define MISSION_EV_HUMP_END        23u /* data = peak [mm] */
/* data = imu_types_motion_t (telemetry only) */
#define MISSION_EV_MOTION_CLASS 24u
#define MISSION_EV_FAULT        25u /* data = SAFETY_FLT_* bits */
/* a = source subsystem, data = 1 ok / 0 failed */
#define MISSION_EV_CAL_DONE 26u
/* data = 4 packed chars, see MISSION_NAV_* */
#define MISSION_EV_REMOTE_MAP 27u
/* SafetyTask cleared its latch after RESET */
#define MISSION_EV_SAFETY_OK 28u
#define MISSION_EV_COUNT     29u

typedef struct
{
    uint8_t  type;     /**< mission_types_ev_t                          */
    uint8_t  src;      /**< APP_SRC_*                             */
    uint16_t arg;      /**< small argument (seq, param id, ...)   */
    int32_t  data;     /**< see mission_types_ev_t                      */
} mission_types_evt_t; /* 8 bytes */

/* ---------------------------------------------------------------------
 *  Navigation commands (FR4).  The character -> movement mapping is NOT
 *  hard-coded: the instructor gives the four characters just before the
 *  demo and the laptop sends them over MQTT (course forum, 24 Oct):
 *     {"id":N,"cmd":"map","left":"A","right":"F","straight":"Z","uturn":"W"}
 *  MISSION_EV_REMOTE_MAP carries them packed: byte k = character for
 *  movement k (MISSION_NAV_LEFT = byte 0 ... MISSION_NAV_UTURN = byte 3).
 *  Until a map arrives, decoded barcodes are reported but never acted on.
 * --------------------------------------------------------------------- */
typedef uint8_t mission_types_nav_t; /* one of the constants below */
#define MISSION_NAV_LEFT     0u
#define MISSION_NAV_RIGHT    1u
#define MISSION_NAV_STRAIGHT 2u
#define MISSION_NAV_UTURN    3u
#define MISSION_NAV_COUNT    4u
#define MISSION_NAV_NONE     0xFFu

/** MissionTask's own fault causes (mission_types_snap_t.cause), besides
 * SafetyTask's.
 */
typedef uint8_t mission_types_cause_t; /* one of the constants below */
#define MISSION_CAUSE_NONE   0u
#define MISSION_CAUSE_SAFETY 1u /* see mission_types_snap_t.fault_bits */
/* BOOT: a subsystem's hardware did not start */
#define MISSION_CAUSE_SELFTEST  2u
#define MISSION_CAUSE_BLOCKED   3u /* BLOCKED_TIMEOUT: no path for 10 s */
#define MISSION_CAUSE_LINE_LOST 4u /* LINE_SEARCH exhausted */
/* an event to mbf_mission was dropped */
#define MISSION_CAUSE_MBF_OVERFLOW 5u
#define MISSION_CAUSE_TIMEOUT 6u /* a state exceeded its supervision time */

/* mission_types_snap_t.calibrated */
#define MISSION_CAL_IR  0x01u
#define MISSION_CAL_IMU 0x02u
#define MISSION_CAL_ALL (MISSION_CAL_IR | MISSION_CAL_IMU)

/** mission_types_snap_t.cal_phase: what CALIBRATING asks the sensor tasks to do
 */
typedef uint8_t mission_types_cal_phase_t; /* one of the constants below */
#define MISSION_CAL_PH_NONE   0u
#define MISSION_CAL_PH_STILL  1u /* stand still 1 s: IMU level offsets */
#define MISSION_CAL_PH_SPIN   2u /* 360 deg spin: IR min/max, magnetometer */
#define MISSION_CAL_PH_FINISH 3u /* compute and report MISSION_EV_CAL_DONE */

/* ---------------------------------------------------------------------
 *  IF-10  Mission snapshot (written by MissionTask).  Besides the state,
 *  it carries the run parameters every task reads (set over MQTT in IDLE).
 * --------------------------------------------------------------------- */
typedef struct
{
    uint32_t t_ms;
    uint32_t fault_bits;     /**< SAFETY_FLT_* bits of the last FAULT    */
    uint16_t cruise_mm_s;    /**< line-following speed (param 1)         */
    uint16_t speed_cap_mm_s; /**< current cap: slow-down / hump / nav    */
    uint16_t slow_mm;        /**< obstacle slow-down range (param 2)     */
    uint16_t stop_mm;        /**< obstacle stop range (param 3)          */
    uint16_t lost_mm;        /**< line-lost length (param 4)             */
    uint16_t junction_mm;    /**< junction length (param 5)              */
    uint16_t runs;           /**< runs started since boot                */
    uint8_t  state; /**< mission_types_state_t                        */
    uint8_t  auth;  /**< motion_types_src_t granted to MotionTask     */

    uint8_t
         pending_nav;  /**< mission_types_nav_t latched for the next junction */
    char pending_char; /**< the barcode character that set it, 0 = none */
    char nav_map[MISSION_NAV_COUNT]; /**< 0 = not mapped yet          */
    uint8_t calibrated; /**< MISSION_CAL_* bits                     */
    uint8_t cal_phase;  /**< mission_types_cal_phase_t                    */
    uint8_t cause;      /**< mission_types_cause_t of the last FAULT      */
    uint8_t reserved;
} mission_types_snap_t;

/* ---------------------------------------------------------------------
 *  IF-09  mbf_scan requests (MissionTask -> ObstacleTask), announced by
 *  OBST_FLG_REQ in flg_obst.
 * --------------------------------------------------------------------- */
typedef uint8_t mission_types_scan_code_t; /* one of the constants below */
#define SCAN_REQ_NONE    0u
#define SCAN_REQ_MONITOR 1u /* servo at 90 deg, ping every 60 ms */
#define SCAN_REQ_PROFILE 2u /* coarse + fine scan + plan, then PROFILE_DONE */
#define SCAN_REQ_BYPASS_LEFT  3u
#define SCAN_REQ_BYPASS_RIGHT 4u
#define SCAN_REQ_RECOVER      5u /* seek and align with the original line */
#define SCAN_REQ_STOP         6u /* stop everything, servo to 90 deg */

typedef struct
{
    uint8_t type; /**< mission_types_scan_code_t */
    uint8_t seq;
    int16_t param;
} mission_types_scan_req_t; /* 4 bytes */

/* ---------------------------------------------------------------------
 *  Obstacle snapshot (written by ObstacleTask).  Lateral positions are
 *  relative to the robot's centre line, + left, in mm.
 * --------------------------------------------------------------------- */
typedef uint8_t mission_types_obst_plan_t; /* one of the constants below */
#define OBST_PLAN_NONE     0u
#define OBST_PLAN_CONTINUE 1u /* nothing inside the robot's corridor */
#define OBST_PLAN_LEFT     2u
#define OBST_PLAN_RIGHT    3u
#define OBST_PLAN_BLOCKED  4u

typedef uint8_t mission_types_obst_state_t; /* one of the constants below */
#define OBST_ST_IDLE        0u
#define OBST_ST_MONITOR     1u
#define OBST_ST_COARSE_SCAN 2u
#define OBST_ST_FINE_SCAN   3u
#define OBST_ST_PLAN        4u
#define OBST_ST_REVERSE     5u
#define OBST_ST_BLOCKED     6u
#define OBST_ST_BYPASS      7u
#define OBST_ST_SEEK_LINE   8u
#define OBST_ST_ALIGN       9u
#define OBST_ST_EDGE_SCAN   10u /* 4 Nov: car turns to find each edge */
#define OBST_ST_SEEK_SWEEP  11u /* no line straight ahead: sweep +-45 deg */
#define OBST_ST_WIDTH_PROBE 12u /* 7 Nov: side-step past both edges */

#define OBST_RANGE_NONE 0xFFFFu

typedef struct
{
    uint32_t t_ms;
    uint32_t pings;      /**< pings sent since boot                  */
    uint16_t d_mm;       /**< last median range ahead; NONE = no target */
    uint16_t w_mm;       /**< profiled width (beam-corrected)        */
    uint16_t closest_mm; /**< closest profiled point                 */
    uint16_t clr_l_mm;   /**< free space beside each edge            */
    uint16_t clr_r_mm;
    int16_t  left_lat_mm;  /**< lateral position of the left edge      */
    int16_t  right_lat_mm; /**< ... and of the right edge              */
    uint16_t no_echo;      /**< consecutive pings with no echo         */
    uint8_t  valid;        /**< 1 = profile fields valid               */
    uint8_t  plan;  /**< mission_types_obst_plan_t                            */
    uint8_t  state; /**< mission_types_obst_state_t                           */
    uint8_t  retries; /**< reverse-and-rescan attempts            */
} mission_types_obst_snap_t;

#endif /* MISSION_TYPES_H */

/*** end of file ***/
