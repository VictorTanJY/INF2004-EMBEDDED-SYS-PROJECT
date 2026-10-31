/*
 *  comms_proto.h
 *  Subsystem 1 -- MQTT message formats (Week 6 Section 6.5).
 *  Owner: Buddy 1 (Victor).
 *
 *  Pure C: no kernel, lwIP or SDK dependency, so the same file is built
 *  into the firmware and into the host unit tests
 *  (testing/subsystem1/host/).  Nothing here touches shared state.
 *
 *  Payloads (base topic COMMS_TOPIC_BASE, e.g. "picocar/is23"):
 *    telemetry  {seq,t_ms,state,v_mm_s:[l,r],enc:[l,r],dist_mm,
 *                line:{err,lost},barcode,pitch_deg,hump_max_mm,
 *                obstacle:{d_cm,w_cm,w_mm,clr_l_cm,clr_r_cm},nav}  QoS 0, 5 Hz
 *    event      {seq,t_ms,type,data,src}                      QoS 1
 *    status     {online,uptime_s,rssi,drops,map}       QoS 1, retained, 1 Hz
 *    cmd (in)   {id,cmd:"start"|"stop"|"estop"|"reset"|"set",param,value}
 *               {id,cmd:"map",left:"A",right:"F",straight:"Z",uturn:"W"}
 *               (barcode character -> movement, sent by the laptop just
 *               before the demo; instructor, course forum 24 Oct)
 *    ack        {id,ok,err}                                   QoS 1
 */

#ifndef COMMS_PROTO_H
#define COMMS_PROTO_H

#include <stdint.h>
#include <stdbool.h>
#include "../app_config.h"

/* ---------------------------------------------------------------------
 *  Telemetry record posted by any task to mbf_telem (IF-12).  12 bytes.
 * --------------------------------------------------------------------- */
typedef enum
{
    COMMS_KIND_EVENT = 1,       /* code = mission_ev_t, data = its data   */
    COMMS_KIND_FAULT = 2,       /* code = 0, data = SAFETY_FLT_* bits     */
    COMMS_KIND_STATE = 3,       /* code = new mission_state_t             */
    COMMS_KIND_LOG   = 4        /* code, data: free use by the sender     */
} comms_kind_t;

typedef struct
{
    uint8_t  kind;              /* comms_kind_t                           */
    uint8_t  src;               /* APP_SRC_* (subsystem number)           */
    uint16_t code;
    int32_t  data;
    uint32_t t_ms;              /* sender's app_time_ms()                 */
} comms_rec_t;

/* ---------------------------------------------------------------------
 *  Bounded JSON writer.  Never writes past cap; on overflow the result
 *  is discarded (jw_finish returns 0).
 * --------------------------------------------------------------------- */
typedef struct
{
    char    *p_buf;
    uint16_t cap;
    uint16_t len;
    bool     b_overflow;
} jw_t;

void     jw_init(jw_t * p_jw, char * p_buf, uint16_t cap);
void     jw_raw(jw_t * p_jw, char const * p_text);
void     jw_int(jw_t * p_jw, int32_t value);
void     jw_uint(jw_t * p_jw, uint32_t value);
void     jw_fixed1(jw_t * p_jw, int32_t tenths);   /* -12 -> "-1.2" */
void     jw_str(jw_t * p_jw, char const * p_text); /* quoted, escaped */
uint16_t jw_finish(jw_t * p_jw);                   /* length, 0 on overflow */

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
    int16_t  pitch_cdeg;
    uint16_t hump_max_mm;
    uint16_t obst_d_mm;         /* COMMS_D_NONE = nothing in range        */
    uint16_t obst_w_mm;
    uint16_t clr_l_mm;
    uint16_t clr_r_mm;
} comms_telem_t;

uint16_t comms_build_telemetry(comms_telem_t const * p_t, char * p_buf, uint16_t cap);
uint16_t comms_build_event(comms_rec_t const * p_rec, uint32_t seq, char * p_buf, uint16_t cap);
/* p_map: the 4 mapped characters (left, right, straight, U-turn) or NULL. */
uint16_t comms_build_status(bool b_online, uint32_t uptime_s, int32_t rssi,
                            uint32_t drops, char const * p_map, char * p_buf, uint16_t cap);

/* Names used in the JSON.  Index = mission_state_t / mission_ev_t. */
#define COMMS_STATE_NAME_COUNT  11u
#define COMMS_EVENT_NAME_COUNT  29u
char const * comms_state_name(uint8_t state);
char const * comms_event_name(uint16_t event);

/* ---------------------------------------------------------------------
 *  Remote commands.
 * --------------------------------------------------------------------- */
typedef enum
{
    COMMS_CMD_NONE  = 0,
    COMMS_CMD_START = 1,
    COMMS_CMD_STOP  = 2,
    COMMS_CMD_ESTOP = 3,
    COMMS_CMD_RESET = 4,
    COMMS_CMD_SET   = 5,
    COMMS_CMD_MAP   = 6         /* value = 4 packed chars: left, right, straight, U-turn */
} comms_cmd_code_t;

typedef enum
{
    COMMS_ERR_NONE      = 0,
    COMMS_ERR_BAD_JSON  = 1,    /* not a flat JSON object                  */
    COMMS_ERR_NO_ID     = 2,    /* "id" missing or not 0..2^31-1           */
    COMMS_ERR_BAD_CMD   = 3,    /* "cmd" missing or unknown                */
    COMMS_ERR_BAD_PARAM = 4,    /* set: "param" missing or not whitelisted */
    COMMS_ERR_RANGE     = 5,    /* set: "value" missing or out of range    */
    COMMS_ERR_BUSY      = 6,    /* start/set outside IDLE (FR14)           */
    COMMS_ERR_FULL      = 7,    /* mbf_mission full                        */
    COMMS_ERR_REUSED    = 8,    /* id seen recently with a different command */
    COMMS_ERR_NOT_CAL   = 9,    /* start before a valid calibration        */
    COMMS_ERR_BAD_MAP   = 10,   /* map: a movement missing, not one Code 39
                                   character, '*', or a character used twice */
    COMMS_ERR_COUNT     = 11
} comms_err_t;

typedef struct
{
    uint32_t id;
    bool     b_has_id;
    uint8_t  cmd;               /* comms_cmd_code_t                       */
    uint8_t  param_id;          /* set only                               */
    int32_t  value;             /* set only                               */
} comms_cmd_t;

/* Parse one command payload (not NUL-terminated; len is authoritative). */
comms_err_t comms_parse_cmd(uint8_t const * p_data, uint16_t len, comms_cmd_t * p_cmd);

/* State rule (FR14): START, SET and MAP only in IDLE.  b_idle = mission IDLE. */
comms_err_t comms_check_cmd(comms_cmd_t const * p_cmd, bool b_idle);

/* START also needs a valid calibration (Week 6 Section 5). */
comms_err_t comms_check_ready(comms_cmd_t const * p_cmd, bool b_calibrated);

/* A character the map may use: any Code 39 data character (not '*'). */
bool        comms_map_char_ok(char c);

uint16_t comms_build_ack(comms_cmd_t const * p_cmd, comms_err_t err,
                         char * p_buf, uint16_t cap);
char const * comms_err_name(comms_err_t err);

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
comms_param_t const * comms_param_by_name(char const * p_name, uint16_t name_len);
comms_param_t const * comms_param_by_id(uint8_t id);

/* ---------------------------------------------------------------------
 *  Duplicate suppression (Tribunal 1 Version C gap 9).
 *
 *  QoS 1 is at-least-once, so the broker may deliver a command again after
 *  a reconnect, possibly after other commands.  The last COMMS_DEDUP_LEN
 *  accepted commands are kept for COMMS_DEDUP_WINDOW_MS:
 *    - same id, same command  -> acknowledged again, not executed again;
 *    - same id, other command -> rejected with COMMS_ERR_REUSED;
 *    - STOP and ESTOP are always executed (repeating them is safe).
 *  The cache is RAM only and empty after a reboot, so the laptop must not
 *  reuse an id within one robot session (e.g. start ids from the time).
 * --------------------------------------------------------------------- */
#define COMMS_DEDUP_LEN         8u
#define COMMS_DEDUP_WINDOW_MS   60000u

typedef struct
{
    uint32_t id;
    uint32_t t_ms;
    int32_t  value;
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
    comms_rec_t rec[COMMS_RING_LEN];
    uint32_t    seq[COMMS_RING_LEN];
    uint8_t     head;           /* index of the oldest entry */
    uint8_t     count;
} comms_ring_t;

void comms_ring_init(comms_ring_t * p_ring);
bool comms_ring_push(comms_ring_t * p_ring, comms_rec_t const * p_rec, uint32_t seq); /* true = oldest overwritten */
bool comms_ring_peek(comms_ring_t const * p_ring, comms_rec_t * p_rec, uint32_t * p_seq);
void comms_ring_pop(comms_ring_t * p_ring);

#endif /* COMMS_PROTO_H */
