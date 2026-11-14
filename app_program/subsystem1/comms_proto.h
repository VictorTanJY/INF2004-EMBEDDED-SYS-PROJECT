/*
 *  comms_proto.h
 *  Subsystem 1 -- MQTT message formats (Week 6 Section 6.5).
 *  Owner: Buddy 1 (Victor).
 *
 *  Pure C: no kernel, lwIP or SDK dependency, so the same file is built
 *  into the firmware and into the host unit tests
 *  (testing/subsystem1/host/).  Nothing here touches shared state.
 *
 *  Change Requirement 1 (minimum-bit data): every MQTT payload is a binary
 *  frame of bit fields, each only as wide as its valid range, written LSB
 *  first with app_pack.h (explicit masks and shifts, never a C struct).
 *  The laptop decodes them with dashboard/picocar_codec.py.  Field order
 *  and widths are listed below; "off" = offset-binary (signed), "sat" =
 *  saturates at its largest value, "wrap" = low bits of a counter.
 *
 *  Every frame starts with one byte: type (5 bits) + version (3 bits).
 *
 *  telemetry (type 1, QoS 0, 5 Hz), 210 bits = 27 bytes:
 *    seq 8 wrap | t 24 (10 ms units, wrap) | state 4 | v_l 10 off |
 *    v_r 10 off (mm/s) | enc_l 16 wrap | enc_r 16 wrap | dist 16 wrap (mm) |
 *    line_err 11 off | lost 1 | line_bc 10 sat (0..1000) | barcode 6 |
 *    nav 6 (Code 39 symbol: 0 none, 1..43) | pitch 11 off | roll 12 off
 *    (0.1 deg) | hump_max 8 sat (mm) | d 9 (cm, 511 none) | w 10 (mm,
 *    1023 none) | clr_l 7 | clr_r 7 (cm, 127 none)
 *  event (type 2, QoS 1), 54 bits + data = 7..11 bytes:
 *    seq 8 wrap | t 24 | kind 2 (kind - 1) | src 3 | code 7 | dlen 2 |
 *    data 0 / 8 / 16 / 32 bits (two's complement; dlen 0 means data 0)
 *  status (type 3, QoS 1, retained, 1 Hz), 41 or 65 bits = 6 or 9 bytes:
 *    uptime 17 sat (s) | rssi 7 sat (-dBm) | drops 8 sat | has_map 1 |
 *    [left 6 | right 6 | straight 6 | uturn 6]
 *    The Last Will is the header byte alone: 1 byte = offline.
 *  ack (type 4, QoS 1), 21 bits = 3 bytes:
 *    has_id 1 | id 8 | err 4 (0 = accepted, else comms_err_t)
 *  cmd (type 5, in), 19 / 39 / 43 bits = 3 / 5 / 6 bytes:
 *    id 8 | cmd 3 | SET: param 4 (index) value 16 off | MAP: 4 x 6
 */

#ifndef COMMS_PROTO_H
#define COMMS_PROTO_H

#include <stdint.h>
#include <stdbool.h>
#include "../app_config.h"

/* ---------------------------------------------------------------------
 *  Telemetry record posted by any task to mbf_telem (IF-12).  8 bytes:
 *  word 0 = kind-1 (2 bits) | src (3) | code (7) | sender time in 10 ms
 *  units (20 bits, wraps every 2.9 h); word 1 = data (all 32 bits: LOG
 *  records carry packed values).  CommsTask rebuilds the full time from
 *  its own clock, as a record is never older than a few seconds.
 * --------------------------------------------------------------------- */
typedef enum
{
    COMMS_KIND_EVENT = 1,       /* code = mission_ev_t, data = its data   */
    COMMS_KIND_FAULT = 2,       /* code = 0, data = SAFETY_FLT_* bits     */
    COMMS_KIND_STATE = 3,       /* code = new mission_state_t             */
    COMMS_KIND_LOG   = 4        /* code, data: free use by the sender     */
} comms_kind_t;

#define COMMS_REC_KIND_SHIFT    0u
#define COMMS_REC_KIND_BITS     2u
#define COMMS_REC_SRC_SHIFT     2u
#define COMMS_REC_SRC_BITS      3u
#define COMMS_REC_CODE_SHIFT    5u
#define COMMS_REC_CODE_BITS     7u
#define COMMS_REC_T_SHIFT       12u
#define COMMS_REC_T_BITS        20u

typedef struct
{
    uint32_t hdr;               /* kind, src, code, time: see above       */
    int32_t  data;
} comms_rec_t;

/* Unpacked view of a record (task-local, never shared). */
typedef struct
{
    uint8_t  kind;              /* comms_kind_t                           */
    uint8_t  src;               /* APP_SRC_* (subsystem number)           */
    uint16_t code;
    int32_t  data;
    uint32_t t_ms;
} comms_event_t;

void comms_rec_pack(comms_rec_t * p_rec, uint8_t kind, uint8_t src, uint16_t code,
                    int32_t data, uint32_t now_ms);
void comms_rec_unpack(comms_rec_t const * p_rec, uint32_t now_ms, comms_event_t * p_ev);

/* ---------------------------------------------------------------------
 *  Frames
 * --------------------------------------------------------------------- */
#define COMMS_FRAME_VERSION     1u
#define COMMS_FRAME_MAX         32u     /* largest frame: telemetry, 27 B */

typedef enum
{
    COMMS_FT_TELEMETRY = 1,
    COMMS_FT_EVENT     = 2,
    COMMS_FT_STATUS    = 3,
    COMMS_FT_ACK       = 4,
    COMMS_FT_CMD       = 5
} comms_frame_type_t;

/* Last Will on the status topic: the status header byte alone (0x23). */
#define COMMS_WILL_MSG          "\x23"

/* Code 39 data symbols, index 1..43 (0 = none). */
#define COMMS_SYMBOL_COUNT      43u
uint8_t comms_symbol_index(char c);     /* 0 if c is not a data symbol */
char    comms_symbol_char(uint8_t index); /* '\0' for 0 or out of range */

/* ---------------------------------------------------------------------
 *  Telemetry (built by CommsTask from the status snapshots).
 * --------------------------------------------------------------------- */
#define COMMS_D_NONE    0xFFFFu     /* no obstacle range */

typedef struct
{
    uint32_t seq;
    uint32_t t_ms;
    uint8_t  state;             /* mission_state_t                        */
    bool     b_line_lost;
    char     barcode;           /* 0 = none yet                           */
    char     nav;               /* latched barcode command char, 0 = none */
    bool     b_obst_valid;      /* profile fields valid                   */
    int16_t  v_l_mm_s;
    int16_t  v_r_mm_s;
    int32_t  enc_l;
    int32_t  enc_r;
    int32_t  dist_mm;
    int16_t  line_err;
    uint16_t line_bc;           /* barcode sensor, 0..1000 (1000 = black) */
    int16_t  pitch_cdeg;
    int16_t  roll_cdeg;         /* + = right side up (one-wheel humps)    */
    uint16_t hump_max_mm;
    uint16_t obst_d_mm;         /* COMMS_D_NONE = nothing in range        */
    uint16_t obst_w_mm;
    uint16_t clr_l_mm;
    uint16_t clr_r_mm;
} comms_telem_t;

/* Each builder returns the frame length in bytes, 0 if cap is too small. */
uint16_t comms_build_telemetry(comms_telem_t const * p_t, uint8_t * p_buf, uint16_t cap);
uint16_t comms_build_event(comms_event_t const * p_ev, uint32_t seq, uint8_t * p_buf, uint16_t cap);
/* p_map: the 4 mapped characters (left, right, straight, U-turn) or NULL. */
uint16_t comms_build_status(uint32_t uptime_s, int32_t rssi, uint32_t drops,
                            char const * p_map, uint8_t * p_buf, uint16_t cap);

/* Field widths the other subsystems' enums must fit (checked at build). */
#define COMMS_STATE_BITS        4u      /* mission_state_t                */
#define COMMS_SRC_BITS          3u      /* APP_SRC_*                      */

/* ---------------------------------------------------------------------
 *  Remote commands.
 * --------------------------------------------------------------------- */
typedef enum
{
    COMMS_CMD_NONE  = 0,
    COMMS_CMD_START = 1,
    COMMS_CMD_STOP  = 2,        /* 3 unused: the remote E-stop was removed (14 Nov) */
    COMMS_CMD_RESET = 4,
    COMMS_CMD_SET   = 5,
    COMMS_CMD_MAP   = 6         /* value = 4 packed chars: left, right, straight, U-turn */
} comms_cmd_code_t;

typedef enum
{
    COMMS_ERR_NONE      = 0,
    COMMS_ERR_BAD_FRAME = 1,    /* wrong header, or wrong length for the command */
    COMMS_ERR_NO_ID     = 2,    /* frame too short to hold an id           */
    COMMS_ERR_BAD_CMD   = 3,    /* unknown command code                    */
    COMMS_ERR_BAD_PARAM = 4,    /* set: parameter index not whitelisted    */
    COMMS_ERR_RANGE     = 5,    /* set: value out of the parameter's range */
    COMMS_ERR_BUSY      = 6,    /* start/set outside IDLE (FR14)           */
    COMMS_ERR_FULL      = 7,    /* mbf_mission full                        */
    COMMS_ERR_REUSED    = 8,    /* id seen recently with a different command */
    COMMS_ERR_NOT_CAL   = 9,    /* start before a valid calibration        */
    COMMS_ERR_BAD_MAP   = 10,   /* map: a movement missing, not one Code 39
                                   character, '*', or a character used twice */
    COMMS_ERR_COUNT     = 11
} comms_err_t;

#define COMMS_ID_BITS           8u      /* command id, wraps (see dedup)  */
#define COMMS_CMD_BITS          3u
#define COMMS_PARAM_BITS        4u      /* index into the whitelist       */
#define COMMS_VALUE_BITS        16u     /* set value, offset-binary       */
#define COMMS_ERR_BITS          4u

/* Unpacked command (task-local). */
typedef struct
{
    uint8_t  id;
    bool     b_has_id;
    uint8_t  cmd;               /* comms_cmd_code_t                       */
    uint8_t  param_id;          /* set only: comms_param_t.id             */
    int32_t  value;             /* set: value; map: 4 packed chars        */
} comms_cmd_t;

/* Parse one binary command frame (len is authoritative). */
comms_err_t comms_parse_cmd(uint8_t const * p_data, uint16_t len, comms_cmd_t * p_cmd);

/* State rule (FR14): START, SET and MAP only in IDLE.  b_idle = mission IDLE. */
comms_err_t comms_check_cmd(comms_cmd_t const * p_cmd, bool b_idle);

/* START also needs a valid calibration (Week 6 Section 5). */
comms_err_t comms_check_ready(comms_cmd_t const * p_cmd, bool b_calibrated);

/* A character the map may use: any Code 39 data character (not '*'). */
bool        comms_map_char_ok(char c);

uint16_t comms_build_ack(comms_cmd_t const * p_cmd, comms_err_t err,
                         uint8_t * p_buf, uint16_t cap);

/* Whitelisted parameters for "set".  PROPOSED: the owning buddy confirms
   each range before Week 10 (TEAM.md IF-11). */
typedef struct
{
    char const * p_name;
    uint8_t      id;
    int32_t      min;
    int32_t      max;
} comms_param_t;

#if APP_BENCH
#define COMMS_PARAM_COUNT       9u      /* + bench_move/turn/speed/servo (ids 101-104) */
#else
#define COMMS_PARAM_COUNT       5u
#endif
comms_param_t const * comms_param_by_index(uint8_t index);  /* frame index */
comms_param_t const * comms_param_by_id(uint8_t id);

/* ---------------------------------------------------------------------
 *  Duplicate suppression (Tribunal 1 Version C gap 9).
 *
 *  QoS 1 is at-least-once, so the broker may deliver a command again after
 *  a reconnect, possibly after other commands.  The last COMMS_DEDUP_LEN
 *  accepted commands are kept for COMMS_DEDUP_WINDOW_MS:
 *    - same id, same command  -> acknowledged again, not executed again;
 *    - same id, other command -> rejected with COMMS_ERR_REUSED;
 *    - STOP is always executed (repeating it is safe).
 *  The cache is RAM only and empty after a reboot.  Ids are 8 bits and
 *  wrap; the laptop counts them up, so an id comes back only after 256
 *  further commands, far outside the 60 s window.
 * --------------------------------------------------------------------- */
#define COMMS_DEDUP_LEN         8u
#define COMMS_DEDUP_WINDOW_MS   60000u

typedef struct
{
    uint32_t t_ms;
    int32_t  value;
    uint8_t  id;
    uint8_t  cmd;
    uint8_t  param_id;
} comms_dedup_entry_t;

typedef struct
{
    comms_dedup_entry_t e[COMMS_DEDUP_LEN];
    uint8_t             next;       /* slot to overwrite next          */
    uint8_t             count;
} comms_dedup_t;

typedef enum
{
    COMMS_DEDUP_NEW      = 0,       /* not seen: execute               */
    COMMS_DEDUP_REPEAT   = 1,       /* same command again: ack only    */
    COMMS_DEDUP_CONFLICT = 2        /* id reused for another command   */
} comms_dedup_result_t;

void                 comms_dedup_init(comms_dedup_t * p_dd);
bool                 comms_cmd_repeat_safe(uint8_t cmd);
comms_dedup_result_t comms_dedup_check(comms_dedup_t const * p_dd,
                                       comms_cmd_t const * p_cmd, uint32_t now_ms);
void                 comms_dedup_add(comms_dedup_t * p_dd,
                                     comms_cmd_t const * p_cmd, uint32_t now_ms);

/* ---------------------------------------------------------------------
 *  Offline event ring: the last COMMS_RING_LEN events, re-published in
 *  order after a reconnection (Week 6 Section 6.5 BACKOFF).
 * --------------------------------------------------------------------- */
#define COMMS_RING_LEN          16u

typedef struct
{
    comms_rec_t rec[COMMS_RING_LEN];    /* packed, 8 B each              */
    uint8_t     seq[COMMS_RING_LEN];    /* event seq (8 bits on the wire) */
    uint8_t     head;           /* index of the oldest entry */
    uint8_t     count;
} comms_ring_t;

void comms_ring_init(comms_ring_t * p_ring);
bool comms_ring_push(comms_ring_t * p_ring, comms_rec_t const * p_rec, uint8_t seq); /* true = oldest overwritten */
bool comms_ring_peek(comms_ring_t const * p_ring, comms_rec_t * p_rec, uint8_t * p_seq);
void comms_ring_pop(comms_ring_t * p_ring);

#endif /* COMMS_PROTO_H */
