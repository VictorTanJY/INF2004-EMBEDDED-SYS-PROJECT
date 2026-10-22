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
 *                obstacle:{d_cm,w_cm,clr_l_cm,clr_r_cm}}       QoS 0, 5 Hz
 *    event      {seq,t_ms,type,data,src}                      QoS 1
 *    status     {online,uptime_s,rssi,drops}           QoS 1, retained, 1 Hz
 *    cmd (in)   {id,cmd:"start"|"stop"|"estop"|"reset"|"set",param,value}
 *    ack        {id,ok,err}                                   QoS 1
 */

#ifndef COMMS_PROTO_H
#define COMMS_PROTO_H

#include <stdint.h>
#include <stdbool.h>

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
uint16_t comms_build_status(bool b_online, uint32_t uptime_s, int32_t rssi,
                            uint32_t drops, char * p_buf, uint16_t cap);

/* Names used in the JSON.  Index = mission_state_t / mission_ev_t. */
#define COMMS_STATE_NAME_COUNT  11u
#define COMMS_EVENT_NAME_COUNT  27u
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
    COMMS_CMD_SET   = 5
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
    COMMS_ERR_COUNT     = 8
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

/* State rule (FR14): START and SET only in IDLE.  b_idle = mission IDLE. */
comms_err_t comms_check_cmd(comms_cmd_t const * p_cmd, bool b_idle);

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

#define COMMS_PARAM_COUNT       5u
comms_param_t const * comms_param_by_name(char const * p_name, uint16_t name_len);
comms_param_t const * comms_param_by_id(uint8_t id);

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
