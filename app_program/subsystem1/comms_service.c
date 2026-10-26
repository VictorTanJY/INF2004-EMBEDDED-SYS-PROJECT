/*
 *  comms_service.c
 *  Subsystem 1 -- CommsTask application layer (Week 6 Section 6.5).
 *  Owner: Buddy 1 (Victor).
 *
 *  Everything in this file runs on the template's WiFi service task
 *  cyw43_utk_task (core 0, priority 4), called through the hooks declared
 *  in lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.h.  It is the only task
 *  allowed to call lwIP/MQTT, and this file reaches MQTT only through
 *  mqtt_utk_publish().  It never blocks: every kernel call uses TMO_POL,
 *  and snapshots are copied under their per-snapshot locks.
 *
 *  Every poll (~20 ms):
 *    1. re-publish the offline event ring, oldest first (QoS 1);
 *    2. drain up to COMMS_DRAIN_MAX records from mbf_telem and publish
 *       them as events, or keep them in the ring while offline;
 *    3. every 200 ms, sample the snapshots and publish telemetry (QoS 0;
 *       dropped while offline, as designed);
 *    4. every 1 s, publish the retained status message (the heartbeat).
 *  Received commands are validated, executed and acknowledged.
 *
 *  Built only with NET=lwip MQTT=1; otherwise the records in mbf_telem
 *  simply accumulate and further posts are counted as drops.
 */

#include "subsystem1.h"

#if TM_MQTT

#include "mqtt_utk.h"
#include "cyw43_utk.h"
#include "../subsystem2/subsystem2.h"
#include "../subsystem3/subsystem3.h"
#include "../subsystem4/subsystem4.h"
#include "../subsystem5/subsystem5.h"

/* The JSON name tables must match subsystem 5's enums. */
typedef char s1_check_state_names[(COMMS_STATE_NAME_COUNT == (uint32_t)MISSION_ST_COUNT) ? 1 : -1];
typedef char s1_check_event_names[(COMMS_EVENT_NAME_COUNT == (uint32_t)MISSION_EV_COUNT) ? 1 : -1];
typedef char s1_check_rec_size[(sizeof(comms_rec_t) == 12u) ? 1 : -1];

#define COMMS_DRAIN_MAX         8u      /* records per poll                     */
#define COMMS_TX_BUF_SIZE       384u    /* largest payload (telemetry ~330 B)   */

static T_MQTT_UTK_CFG const k_mqtt_cfg =
{
    COMMS_TOPIC_CMD,  1u,               /* subscribe: commands, QoS 1           */
    COMMS_TOPIC_STATUS,                 /* Last Will on the retained status     */
    "{\"online\":false}",
    1u,                                 /* will QoS 1                           */
    1u                                  /* will retained                        */
};

/* All state is zero-initialised and touched only by the WiFi task. */
static comms_ring_t g_ring;
static uint32_t     g_telem_seq;
static uint32_t     g_event_seq;
static uint32_t     g_next_telem_ms;
static uint32_t     g_next_status_ms;
static bool         g_b_online;
static bool         g_b_flag_synced;    /* WIFI_LOSS bit set once at start-up   */
static comms_dedup_t g_dedup;           /* recently accepted commands (gap 9)   */
static uint32_t     g_ring_lost;        /* events overwritten while offline     */
static uint32_t     g_pub_errors;       /* mqtt_publish() refused (ERR_MEM ...) */
static char         g_tx[COMMS_TX_BUF_SIZE];

static ID comms_telem_id(void)
{
    return *(ID volatile *)&g_mbf_telem;
}

static bool topic_eq(char const * p_a, char const * p_b)
{
    while ((*p_a != '\0') && (*p_a == *p_b))
    {
        p_a++;
        p_b++;
    }
    return (*p_a == *p_b);
}

static void comms_set_wifi_loss(bool b_lost)
{
    if (g_flg_safety > 0)
    {
        if (b_lost)
        {
            (void)tk_set_flg(g_flg_safety, SAFETY_FLT_WIFI_LOSS);
        }
        else
        {
            (void)tk_clr_flg(g_flg_safety, ~(UINT)SAFETY_FLT_WIFI_LOSS);
        }
        g_b_flag_synced = true;
    }
}

static bool comms_publish(char const * p_topic, uint16_t len, uint8_t qos, uint8_t retain)
{
    if (len == 0u)
    {
        g_pub_errors++;                 /* did not fit COMMS_TX_BUF_SIZE        */
        return true;                    /* nothing to retry                     */
    }
    if (mqtt_utk_publish(p_topic, (uint8_t const *)g_tx, len, qos, retain) != 0)
    {
        g_pub_errors++;
        return false;
    }
    return true;
}

static bool comms_publish_event(comms_rec_t const * p_rec, uint32_t seq)
{
    uint16_t len = comms_build_event(p_rec, seq, g_tx, (uint16_t)sizeof(g_tx));

    return comms_publish(COMMS_TOPIC_EVENT, len, 1u, 0u);
}

static void comms_publish_status(uint32_t now_ms)
{
    uint32_t       drops = comms_telem_drops() + g_ring_lost + g_pub_errors
                         + mqtt_utk_pub_failures() + mqtt_utk_rx_dropped();
    mission_snap_t ms;
    uint16_t       len;

    mission_get_snap(&ms);
    len = comms_build_status(true, now_ms / 1000u, cyw43_utk_get_rssi(), drops,
                             ms.nav_map, g_tx, (uint16_t)sizeof(g_tx));

    (void)comms_publish(COMMS_TOPIC_STATUS, len, 1u, 1u);
}

static void comms_publish_telemetry(uint32_t now_ms)
{
    comms_telem_t  t;
    motion_odom_t  odom;
    line_snap_t    line;
    imu_snap_t     imu;
    mission_snap_t mission;
    obst_snap_t    obst;
    uint16_t       len;

    /* Each copy holds its own lock for a few microseconds only. */
    motion_get_odom(&odom);
    line_get_snap(&line);
    imu_get_snap(&imu);
    mission_get_snap(&mission);
    obst_get_snap(&obst);

    g_telem_seq++;
    t.seq          = g_telem_seq;
    t.t_ms         = now_ms;
    t.state        = mission.state;
    t.v_l_mm_s     = odom.v_l_mm_s;
    t.v_r_mm_s     = odom.v_r_mm_s;
    t.enc_l        = odom.enc_l;
    t.enc_r        = odom.enc_r;
    t.dist_mm      = odom.dist_mm;
    t.line_err     = line.err;
    t.b_line_lost  = ((line.flags & LINE_FLAG_LOST) != 0u);
    t.barcode      = line.last_barcode;
    t.nav          = mission.pending_char;
    t.pitch_cdeg   = imu.pitch_cdeg;
    t.hump_max_mm  = imu.hump_max_mm;
    t.obst_d_mm    = obst.d_mm;
    t.b_obst_valid = (obst.valid != 0u);
    t.obst_w_mm    = obst.w_mm;
    t.clr_l_mm     = obst.clr_l_mm;
    t.clr_r_mm     = obst.clr_r_mm;

    len = comms_build_telemetry(&t, g_tx, (uint16_t)sizeof(g_tx));
    (void)comms_publish(COMMS_TOPIC_TELEMETRY, len, 0u, 0u);
}

/* Due check that tolerates the 32-bit millisecond wrap. */
static bool comms_due(uint32_t now_ms, uint32_t * p_next_ms, uint32_t period_ms)
{
    if ((int32_t)(now_ms - *p_next_ms) < 0)
    {
        return false;
    }
    *p_next_ms += period_ms;
    if ((int32_t)(now_ms - *p_next_ms) >= 0)
    {
        *p_next_ms = now_ms + period_ms;    /* fell behind: do not burst */
    }
    return true;
}

/* ---------------------------------------------------------------------
 *  mqtt_utk hooks
 * --------------------------------------------------------------------- */
const T_MQTT_UTK_CFG *mqtt_utk_app_config(void)
{
    return &k_mqtt_cfg;
}

void mqtt_utk_app_connected(void)
{
    g_b_online       = true;
    g_next_status_ms = (uint32_t)app_time_ms();    /* status "online" at once */
    g_next_telem_ms  = g_next_status_ms;           /* never compare against 0 */
    comms_set_wifi_loss(false);
}

void mqtt_utk_app_disconnected(void)
{
    g_b_online = false;
    comms_set_wifi_loss(true);
}

void mqtt_utk_app_poll(void)
{
    ID          mbf = comms_telem_id();
    uint32_t    now_ms;
    comms_rec_t rec;
    uint32_t    seq;
    uint32_t    n;

    if (mbf <= 0)
    {
        return;                         /* subsystem1_init() has not run yet */
    }
    now_ms = (uint32_t)app_time_ms();

    if (!g_b_flag_synced)
    {
        comms_set_wifi_loss(!g_b_online);
    }

    /* 1. Offline ring first, so events stay in order. */
    while (g_b_online && comms_ring_peek(&g_ring, &rec, &seq))
    {
        if (!comms_publish_event(&rec, seq))
        {
            break;                      /* lwIP full: retry on the next poll */
        }
        comms_ring_pop(&g_ring);
    }

    /* 2. New records. */
    for (n = 0u; n < COMMS_DRAIN_MAX; n++)
    {
        if (tk_rcv_mbf(mbf, &rec, TMO_POL) != (INT)sizeof(rec))
        {
            break;
        }
        g_event_seq++;
        if (g_b_online && (g_ring.count == 0u) && comms_publish_event(&rec, g_event_seq))
        {
            continue;
        }
        if (comms_ring_push(&g_ring, &rec, g_event_seq))
        {
            g_ring_lost++;
        }
    }

    /* 3. Telemetry, 5 Hz. */
    if (comms_due(now_ms, &g_next_telem_ms, COMMS_TELEM_PERIOD_MS) && g_b_online)
    {
        comms_publish_telemetry(now_ms);
    }

    /* 4. Status heartbeat, 1 Hz, retained. */
    if (comms_due(now_ms, &g_next_status_ms, COMMS_STATUS_PERIOD_MS) && g_b_online)
    {
        comms_publish_status(now_ms);
    }
}

static comms_err_t comms_execute(comms_cmd_t const * p_cmd)
{
    ER          er;
    mission_ev_t ev;

    switch (p_cmd->cmd)
    {
        case COMMS_CMD_ESTOP:
            /* Straight to SafetyTask (NFR8: E-stop received -> brake <= 20 ms). */
            if (g_flg_safety <= 0)
            {
                return COMMS_ERR_FULL;
            }
            (void)tk_set_flg(g_flg_safety, SAFETY_FLT_ESTOP);
            (void)comms_telem_post((uint8_t)COMMS_KIND_FAULT, (uint8_t)APP_SRC_COMMS,
                                   0u, (int32_t)SAFETY_FLT_ESTOP);
            return COMMS_ERR_NONE;

        case COMMS_CMD_START:
            ev = MISSION_EV_REMOTE_START;
            break;

        case COMMS_CMD_STOP:
            ev = MISSION_EV_REMOTE_STOP;
            break;

        case COMMS_CMD_RESET:
            ev = MISSION_EV_REMOTE_RESET;
            break;

        case COMMS_CMD_SET:
            ev = MISSION_EV_REMOTE_SET;
            break;

        case COMMS_CMD_MAP:
            ev = MISSION_EV_REMOTE_MAP;     /* MissionTask stores the map */
            break;

        default:
            return COMMS_ERR_BAD_CMD;
    }

    if (g_mbf_mission <= 0)
    {
        return COMMS_ERR_FULL;
    }
    er = mission_post((uint8_t)ev, (uint8_t)APP_SRC_COMMS,
                      (uint16_t)p_cmd->param_id, p_cmd->value);
    if (er != E_OK)
    {
        return COMMS_ERR_FULL;
    }
    (void)comms_telem_post((uint8_t)COMMS_KIND_EVENT, (uint8_t)APP_SRC_COMMS,
                           (uint16_t)ev, p_cmd->value);
    return COMMS_ERR_NONE;
}

void mqtt_utk_app_rx(const char *topic, const uint8_t *data, uint16_t len)
{
    comms_cmd_t    cmd;
    comms_err_t    err;
    mission_snap_t mission;
    uint16_t       ack_len;

    if ((comms_telem_id() <= 0) || !topic_eq(topic, COMMS_TOPIC_CMD))
    {
        return;
    }

    err = comms_parse_cmd(data, len, &cmd);
    if (err == COMMS_ERR_NONE)
    {
        uint32_t             now_ms = (uint32_t)app_time_ms();
        comms_dedup_result_t dup    = comms_cmd_repeat_safe(cmd.cmd)
                                      ? COMMS_DEDUP_NEW
                                      : comms_dedup_check(&g_dedup, &cmd, now_ms);

        if (dup == COMMS_DEDUP_REPEAT)
        {
            /* QoS 1 redelivery: acknowledge again, never execute twice. */
        }
        else if (dup == COMMS_DEDUP_CONFLICT)
        {
            err = COMMS_ERR_REUSED;
        }
        else
        {
            mission_get_snap(&mission);
            err = comms_check_cmd(&cmd, mission.state == (uint8_t)MISSION_ST_IDLE);
            if (err == COMMS_ERR_NONE)
            {
                err = comms_check_ready(&cmd, (mission.calibrated & MISSION_CAL_ALL) == MISSION_CAL_ALL);
            }
            if (err == COMMS_ERR_NONE)
            {
                err = comms_execute(&cmd);
            }
            if ((err == COMMS_ERR_NONE) && !comms_cmd_repeat_safe(cmd.cmd))
            {
                comms_dedup_add(&g_dedup, &cmd, now_ms);
            }
        }
    }

    ack_len = comms_build_ack(&cmd, err, g_tx, (uint16_t)sizeof(g_tx));
    (void)comms_publish(COMMS_TOPIC_ACK, ack_len, 1u, 0u);
}

#else  /* !TM_MQTT */

/* MQTT is not built: nothing drains mbf_telem. */
typedef int s1_comms_service_unused;

#endif /* TM_MQTT */
