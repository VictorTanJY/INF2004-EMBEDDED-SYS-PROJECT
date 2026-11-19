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

typedef uint8_t mission_core_act_kind_t; /* one of the constants below */
#define MISSION_ACT_MOTION 1u            /* cmd, seq, p0, p1 (src = MISSION) */
#define MISSION_ACT_SCAN   2u /* cmd = mission_types_scan_code_t, p0 = param */
#define MISSION_ACT_SAFETY_RESET 3u /* subsystem4_request_reset() */
#define MISSION_ACT_STATE        4u /* telemetry: p0 = new state */
#define MISSION_ACT_LOG          5u /* telemetry: cmd = log code, p0/p1 data */
#define MISSION_ACT_NEW_RUN      6u /* a run starts (hump maximum resets) */

/* MISSION_ACT_LOG codes */
#define MISSION_LOG_NOT_CALIBRATED 61u
#define MISSION_LOG_CAL_RESULT     62u /* p0 = MISSION_CAL_* bits achieved */
#define MISSION_LOG_NAV_LATCHED    63u /* p0 = mission_types_nav_t, p1 = char */
#define MISSION_LOG_NAV_UNMAPPED   64u /* p1 = char (valid, not in the map) */
#define MISSION_LOG_MAP_SET        65u /* p0/p1: packed map low/high half  */
#define MISSION_LOG_MAP_BAD        66u
#define MISSION_LOG_SUMMARY   67u /* run summary (task adds the hump max) */
#define MISSION_LOG_PARAM_SET 68u /* p0 = param id, p1 = value        */
#define MISSION_LOG_MBF_DROP  69u

typedef struct
{
    uint8_t kind; /* mission_core_act_kind_t */
    uint8_t cmd;
    uint8_t seq;
    int16_t arg0;
    int16_t arg1;
} mission_core_act_t;

#define MISSION_ACT_MAX 12u

typedef struct
{
    mission_core_act_t act[MISSION_ACT_MAX];
    uint8_t            n_acts;
} mission_core_out_t;

/* What the tick needs to know about the rest of the system. */
typedef struct
{
    bool     b_motion_hw_ok;
    bool     b_line_hw_ok;
    bool     b_imu_ok;
    bool     b_timer_checked;
    bool     b_timer_ok;
    bool     b_on_line; /* line snapshot: a line sensor on black */
    uint32_t mbf_drops; /* subsystem5_mission_drops()                   */
} mission_core_inputs_t;

typedef struct
{
    mission_types_snap_t snap;     /* public state (state, params, map, ...) */
    uint32_t             entry_ms; /* time the current state was entered     */
    uint32_t             phase_ms; /* time the current phase / step started  */
    uint8_t motion_seq;            /* last MOVE / TURN sequence number sent  */
    uint8_t wait_seq;              /* step waiting for this MOTION_DONE      */
    bool    b_waiting;
    uint8_t step;      /* EXECUTE_NAV / LINE_SEARCH step         */
    uint8_t nav_exec;  /* movement being executed (EXECUTE_NAV); a
                          barcode read meanwhile latches afresh  */
    uint8_t  cal_done; /* MISSION_CAL_* answers received         */
    uint8_t  cal_ok;   /* ... and which were successful          */
    bool     b_hump;
    bool     b_slow;
    uint32_t last_drops;
} mission_core_t;

void mission_core_init(mission_core_t * p_mc, uint32_t now_ms);
void mission_core_event(mission_core_t * p_mc, mission_types_evt_t const * p_ev,
                        uint32_t now_ms, mission_core_out_t * p_out);
void mission_core_tick(mission_core_t *              p_mc,
                       mission_core_inputs_t const * p_in, uint32_t now_ms,
                       mission_core_out_t * p_out);

/* Helpers shared with CommsTask's validation and the tests. */
bool    mission_core_map_char_ok(char chr);
bool    mission_core_map_unpack(int32_t packed, char map[MISSION_NAV_COUNT]);
int32_t mission_core_map_pack(char const map[MISSION_NAV_COUNT]);
uint8_t mission_core_nav_for(mission_types_snap_t const * p_snap, char chr);

/* ---------------------------------------------------------------------
 *  Change Requirement 1 (minimum-bit data): mbf_mission events, mbf_scan
 *  requests and the mission snapshot as stored between tasks.
 *
 *  Event word: type 5 | src 3 | a 7 | data 17 (offset-binary, +-65535);
 *  REMOTE_MAP: type 5 | src 3 | four 6-bit Code 39 symbols (left, right,
 *  straight, U-turn).  Scan request: type 3 | seq 5 | param 8 (0..255).
 * --------------------------------------------------------------------- */
#define MISSION_EVT_A_BITS    7u
#define MISSION_EVT_DATA_BITS 17u
uint32_t mission_core_evt_pack(mission_types_evt_t const * p_ev);
void     mission_core_evt_unpack(uint32_t word, mission_types_evt_t * p_ev);
uint16_t mission_core_scan_req_pack(mission_types_scan_req_t const * p_req);
void     mission_core_scan_req_unpack(uint16_t                   word,
                                      mission_types_scan_req_t * p_req);

#define MISSION_SNAP_BITS  165u /* sum of the layout widths      */
#define MISSION_SNAP_WORDS 6u   /* 24 B instead of 36 B          */
void     mission_core_snap_pack(mission_types_snap_t const * p_snap,
                                uint32_t *                   p_words);
void     mission_core_snap_unpack(uint32_t const *       p_words,
                                  mission_types_snap_t * p_snap);
uint32_t mission_core_snap_layout_bits(void);

#endif /* MISSION_CORE_H */

/*** end of file ***/
