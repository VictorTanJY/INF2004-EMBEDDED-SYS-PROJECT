/*
 *  mission_core.h
 *  Subsystem 5 -- the system statechart (Week 6 Section 5) as a pure
 *  state machine.  Owner: Buddy 5 (Ryan).  Host-tested.
 *
 *  MissionTask feeds it every mbf_mission event and a supervision tick
 *  (at least every 100 ms) and carries out the actions it returns:
 *  motion commands (mbf_motion), scan requests (mbf_scan), the safety
 *  reset request, telemetry records.  It decides motion authority.
 */

#ifndef MISSION_CORE_H
#define MISSION_CORE_H

#include <stdint.h>
#include <stdbool.h>
#include "mission_types.h"
#include "../subsystem2/motion_types.h"

typedef enum
{
    MISSION_ACT_MOTION    = 1,      /* cmd, seq, p0, p1 (src = MISSION)       */
    MISSION_ACT_SCAN      = 2,      /* cmd = scan_req_code_t, p0 = param      */
    MISSION_ACT_SAFETY_RESET = 3,   /* safety_request_reset()                 */
    MISSION_ACT_STATE     = 4,      /* telemetry: p0 = new state              */
    MISSION_ACT_LOG       = 5,      /* telemetry: cmd = log code, p0/p1 data  */
    MISSION_ACT_NEW_RUN   = 6       /* a run starts (hump maximum resets)     */
} mission_act_kind_t;

/* MISSION_ACT_LOG codes */
#define MISSION_LOG_NOT_CALIBRATED  61u
#define MISSION_LOG_CAL_RESULT      62u     /* p0 = MISSION_CAL_* bits achieved */
#define MISSION_LOG_NAV_LATCHED     63u     /* p0 = mission_nav_t, p1 = char    */
#define MISSION_LOG_NAV_UNMAPPED    64u     /* p1 = char (valid, not in the map) */
#define MISSION_LOG_MAP_SET         65u     /* p0/p1: packed map low/high half  */
#define MISSION_LOG_MAP_BAD         66u
#define MISSION_LOG_SUMMARY         67u     /* run summary (task adds the hump max) */
#define MISSION_LOG_PARAM_SET       68u     /* p0 = param id, p1 = value        */
#define MISSION_LOG_MBF_DROP        69u

typedef struct
{
    uint8_t kind;                   /* mission_act_kind_t */
    uint8_t cmd;
    uint8_t seq;
    int16_t p0;
    int16_t p1;
} mission_act_t;

#define MISSION_ACT_MAX     12u

typedef struct
{
    mission_act_t act[MISSION_ACT_MAX];
    uint8_t       n;
} mission_out_t;

/* What the tick needs to know about the rest of the system. */
typedef struct
{
    bool     b_motion_hw_ok;
    bool     b_line_hw_ok;
    bool     b_imu_ok;
    bool     b_timer_checked;
    bool     b_timer_ok;
    bool     b_on_line;             /* line snapshot: a line sensor on black */
    uint32_t mbf_drops;             /* mission_post_drops()                   */
} mission_inputs_t;

typedef struct
{
    mission_snap_t snap;            /* public state (state, params, map, ...) */
    uint32_t entry_ms;              /* time the current state was entered     */
    uint32_t phase_ms;              /* time the current phase / step started  */
    uint8_t  motion_seq;            /* last MOVE / TURN sequence number sent  */
    uint8_t  wait_seq;              /* step waiting for this MOTION_DONE      */
    bool     b_waiting;
    uint8_t  step;                  /* EXECUTE_NAV / LINE_SEARCH step         */
    uint8_t  nav_exec;              /* movement being executed (EXECUTE_NAV); a
                                       barcode read meanwhile latches afresh  */
    uint8_t  cal_done;              /* MISSION_CAL_* answers received         */
    uint8_t  cal_ok;                /* ... and which were successful          */
    bool     b_hump;
    bool     b_slow;
    uint32_t last_drops;
} mission_core_t;

void mission_core_init(mission_core_t * p_mc, uint32_t now_ms);
void mission_core_event(mission_core_t * p_mc, mission_evt_t const * p_ev, uint32_t now_ms,
                        mission_out_t * p_out);
void mission_core_tick(mission_core_t * p_mc, mission_inputs_t const * p_in, uint32_t now_ms,
                       mission_out_t * p_out);

/* Helpers shared with CommsTask's validation and the tests. */
bool mission_map_char_ok(char c);
bool mission_map_unpack(int32_t packed, char map[MISSION_NAV_COUNT]);
int32_t mission_map_pack(char const map[MISSION_NAV_COUNT]);
uint8_t mission_nav_for(mission_snap_t const * p_snap, char c);

#endif /* MISSION_CORE_H */
