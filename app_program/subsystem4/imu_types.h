/** @file imu_types.h
 *
 *  @brief Subsystem 4 -- fault bits (IF-05) and IMU snapshot (IF-08).
 *  Owner: Buddy 4 (Purvez).  Pure C (no kernel).
 */

#ifndef IMU_TYPES_H
#define IMU_TYPES_H

#include <stdint.h>

/* ---------------------------------------------------------------------
 *  IF-05  flg_safety bits.
 *  Brake causes (SAFETY_BRAKE_MASK) wake SafetyTask, which takes them with
 *  TWF_ORW | TWF_BITCLR and ORs them into its latched-fault word.  Report-
 *  only bits never wake it; their setter clears them when the cause ends.
 *  SAFETY_REQ_RESET is not a fault: MissionTask sets it to ask SafetyTask
 *  to clear the latch after an operator reset (if every cause is gone).
 * --------------------------------------------------------------------- */
/* ImuTask: lifted while running (end of run, 14 Nov) */
#define SAFETY_FLT_LIFT      0x0001u
#define SAFETY_FLT_TOO_CLOSE 0x0002u /* ObstacleTask, d < 8 cm         */
#define SAFETY_FLT_STALL     0x0004u /* MotionTask                     */
/* ImuTask: > 30 deg (overturned) while running */
#define SAFETY_FLT_TILT 0x0008u
/* 0x0010 unused: the remote E-stop was removed (14 Nov) */
#define SAFETY_FLT_HEARTBEAT 0x0020u /* SafetyTask itself (latched only) */
#define SAFETY_FLT_ENC_FAULT 0x0040u /* report-only: MotionTask        */
#define SAFETY_FLT_IMU_FAULT 0x0080u /* report-only: ImuTask           */
#define SAFETY_FLT_NO_ECHO   0x0100u /* report-only: ObstacleTask      */
#define SAFETY_FLT_WIFI_LOSS 0x0200u /* report-only: CommsTask         */
#define SAFETY_FLT_SELFTEST  0x0400u /* MissionTask BOOT (latched only) */
#define SAFETY_REQ_RESET     0x8000u /* MissionTask -> SafetyTask      */

#define SAFETY_BRAKE_MASK                                      \
    (SAFETY_FLT_LIFT | SAFETY_FLT_TOO_CLOSE | SAFETY_FLT_STALL \
     | SAFETY_FLT_TILT)

/* ---------------------------------------------------------------------
 *  IF-07  Heartbeats: monitored tasks and their timeouts (Week 6 Sec. 3).
 * --------------------------------------------------------------------- */
typedef uint8_t imu_types_safety_hb_t; /* one of the constants below */
#define SAFETY_HB_MOTION   0u          /* timeout 50 ms */
#define SAFETY_HB_LINE     1u          /* timeout 50 ms */
#define SAFETY_HB_MISSION  2u          /* timeout 300 ms */
#define SAFETY_HB_OBSTACLE 3u          /* timeout 300 ms */
#define SAFETY_HB_IMU      4u          /* timeout 100 ms */
#define SAFETY_HB_COUNT    5u

/* ---------------------------------------------------------------------
 *  IF-08  IMU snapshot (written by ImuTask every 10 ms).
 * --------------------------------------------------------------------- */
typedef uint8_t imu_types_hump_state_t; /* one of the constants below */
#define IMU_HUMP_FLAT       0u
#define IMU_HUMP_CLIMBING   1u
#define IMU_HUMP_CREST      2u
#define IMU_HUMP_DESCENDING 3u

typedef uint8_t imu_types_motion_t; /* one of the constants below */
#define IMU_MOTION_STATIONARY   0u
#define IMU_MOTION_ACCELERATING 1u
#define IMU_MOTION_TURNING      2u
#define IMU_MOTION_CLIMBING     3u
#define IMU_MOTION_DESCENDING   4u
#define IMU_MOTION_IMPACT \
    5u /* not produced since 14 Nov (IMU collision detection removed) */
#define IMU_MOTION_CRUISING     6u
#define IMU_MOTION_DECELERATING 7u /* speed falling (forward or reverse) */

/* imu_types_snap_t.cal */
#define IMU_CAL_LEVEL 0x01u /* level offsets stored           */
#define IMU_CAL_MAG   0x02u /* hard-iron offsets stored       */

typedef struct
{
    uint32_t t_ms;
    int16_t  pitch_cdeg;       /**< pitch, 0.01 deg, + nose up             */
    int16_t  roll_cdeg;        /**< + = right side up (checked on robot)   */
    uint16_t heading_cdeg;     /**< magnetometer, 0..35999                 */
    int16_t  turn_rate_cdeg_s; /**< encoder yaw rate, + left               */
    int16_t  accel_mg;         /**< |a| - 1 g, mg                          */
    uint16_t hump_max_mm;      /**< highest hump peak of the run           */
    uint16_t last_hump_mm;     /**< peak of the last completed hump        */
    uint16_t humps;            /**< completed humps this run               */
    uint16_t i2c_errors;       /**< since boot                             */
    uint8_t  hump_state;  /**< imu_types_hump_state_t                       */
    uint8_t  motion;      /**< imu_types_motion_t                           */
    uint8_t  b_ok;        /**< 1 = readings valid (no IMU_FAULT)      */
    uint8_t  cal;         /**< IMU_CAL_* bits                         */
    uint8_t  pitch_valid; /**< 1 = fresh, validated pitch (hump mask) */
    uint8_t  tilt;        /**< 1 = beyond 30 deg now (reset refused)  */
    uint8_t  reserved[2];
} imu_types_snap_t;

#endif /* IMU_TYPES_H */

/*** end of file ***/
