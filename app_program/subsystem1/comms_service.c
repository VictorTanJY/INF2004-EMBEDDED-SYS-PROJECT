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

/* The frame fields must hold subsystem 5's enums (Change Requirement 1). */
typedef uint8_t s1_check_states_t
    [((uint32_t)MISSION_ST_COUNT <= (1uL << COMMS_STATE_BITS)) ? 1 : -1];
typedef uint8_t s1_check_events_t
    [((uint32_t)MISSION_EV_COUNT <= (1uL << COMMS_REC_CODE_BITS)) ? 1 : -1];
typedef uint8_t
    s1_check_src_t[(APP_SRC_OBST_MISSION < (1uL << COMMS_SRC_BITS)) ? 1 : -1];
typedef uint8_t s1_check_rec_size_t[(8u == sizeof(comms_rec_t)) ? 1 : -1];

#define COMMS_DRAIN_MAX   8u /* records per poll                     */
#define COMMS_TX_BUF_SIZE COMMS_FRAME_MAX /* largest frame: 27 B      */

static mqtt_utk_cfg_t const g_mqtt_cfg = {
    COMMS_TOPIC_CMD,
    1u,                 /* subscribe: commands, QoS 1           */
    COMMS_TOPIC_STATUS, /* Last Will on the retained status     */
    COMMS_WILL_MSG,     /* header byte alone = offline          */
    1u,                 /* will QoS 1                           */
    1u                  /* will retained                        */
};

/* All state is zero-initialised and touched only by the WiFi task. */
static comms_ring_t  g_ring;
static uint8_t       g_telem_seq; /* 8 bits on the wire, wraps            */
static uint8_t       g_event_seq;
static uint32_t      g_next_telem_ms;
static uint32_t      g_next_status_ms;
static bool          gb_online;
static bool          gb_flag_synced; /* WIFI_LOSS bit set once at start-up   */
static comms_dedup_t g_dedup;        /* recently accepted commands (gap 9)   */
static uint32_t      g_ring_lost;    /* events overwritten while offline     */
static uint32_t      g_pub_errors;   /* mqtt_publish() refused (ERR_MEM ...) */
static uint8_t       g_tx[COMMS_TX_BUF_SIZE];

static ID comms_telem_id (void)
{
    return *(ID volatile *)&g_mbf_telem;
}

static bool topic_eq (char const * p_a, char const * p_b)
{
    while ((*p_a != '\0') && (*p_a == *p_b))
    {
        p_a++;
        p_b++;
    }
    return (*p_a == *p_b);
}

static void comms_set_wifi_loss (bool b_lost)
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
        gb_flag_synced = true;
    }
}

static bool comms_publish (char const * p_topic, uint16_t len, uint8_t qos,
                           uint8_t retain)
{
    if (0u == len)
    {
        g_pub_errors++; /* did not fit COMMS_TX_BUF_SIZE        */
        return true;    /* nothing to retry                     */
    }
    if (mqtt_utk_publish(p_topic, g_tx, len, qos, retain) != 0)
    {
        g_pub_errors++;
        return false;
    }
    return true;
}

static bool comms_publish_event (comms_rec_t const * p_rec, uint8_t seq)
{
    comms_event_t event;
    uint16_t      len;

    comms_rec_unpack(p_rec, (uint32_t)app_time_ms(), &event);
    len = comms_build_event(&event, seq, g_tx, (uint16_t)sizeof(g_tx));
    return comms_publish(COMMS_TOPIC_EVENT, len, 1u, 0u);
}

static void comms_publish_status (uint32_t now_ms)
{
    uint32_t       drops = comms_telem_drops() + g_ring_lost + g_pub_errors
                           + mqtt_utk_pub_failures() + mqtt_utk_rx_dropped();
    mission_snap_t mission;
    uint16_t       len;

    mission_get_snap(&mission);
    len = comms_build_status(now_ms / 1000u, cyw43_utk_get_rssi(), drops,
                             mission.nav_map, g_tx, (uint16_t)sizeof(g_tx));

    (void)comms_publish(COMMS_TOPIC_STATUS, len, 1u, 1u);
}

static void comms_publish_telemetry (uint32_t now_ms)
{
    comms_telem_t  telem;
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
    telem.seq          = g_telem_seq;
    telem.t_ms         = now_ms;
    telem.state        = mission.state;
    telem.v_l_mm_s     = odom.v_l_mm_s;
    telem.v_r_mm_s     = odom.v_r_mm_s;
    telem.enc_l        = odom.enc_l;
    telem.enc_r        = odom.enc_r;
    telem.dist_mm      = odom.dist_mm;
    telem.line_err     = line.err;
    telem.line_bc      = line.raw_bc;
    telem.b_line_lost  = ((line.flags & LINE_FLAG_LOST) != 0u);
    telem.barcode      = line.last_barcode;
    telem.nav          = mission.pending_char;
    telem.pitch_cdeg   = imu.pitch_cdeg;
    telem.roll_cdeg    = imu.roll_cdeg;
    telem.hump_max_mm  = imu.hump_max_mm;
    telem.obst_d_mm    = obst.d_mm;
    telem.b_obst_valid = (obst.valid != 0u);
    telem.obst_w_mm    = obst.w_mm;
    telem.clr_l_mm     = obst.clr_l_mm;
    telem.clr_r_mm     = obst.clr_r_mm;

    len = comms_build_telemetry(&telem, g_tx, (uint16_t)sizeof(g_tx));
    (void)comms_publish(COMMS_TOPIC_TELEMETRY, len, 0u, 0u);
}

/* Due check that tolerates the 32-bit millisecond wrap. */
static bool comms_due (uint32_t now_ms, uint32_t * p_next_ms,
                       uint32_t period_ms)
{
    if ((int32_t)(now_ms - *p_next_ms) < 0)
    {
        return false;
    }
    *p_next_ms += period_ms;
    if ((int32_t)(now_ms - *p_next_ms) >= 0)
    {
        *p_next_ms = now_ms + period_ms; /* fell behind: do not burst */
    }
    return true;
}

/* ---------------------------------------------------------------------
 *  mqtt_utk hooks
 * --------------------------------------------------------------------- */
mqtt_utk_cfg_t const * mqtt_utk_app_config (void)
{
    return &g_mqtt_cfg;
}

void mqtt_utk_app_connected (void)
{
    uint32_t const now_ms = (uint32_t)app_time_ms();

    gb_online        = true;
    g_next_telem_ms  = now_ms;
    g_next_status_ms = now_ms;
    comms_set_wifi_loss(false);
}

void mqtt_utk_app_disconnected (void)
{
    gb_online = false;
    comms_set_wifi_loss(true);
}

void mqtt_utk_app_poll (void)
{
    ID          mbf = comms_telem_id();
    uint32_t    now_ms;
    comms_rec_t rec;
    uint8_t     seq;
    uint32_t    count;
    bool        b_sent;

    if (mbf <= 0)
    {
        return; /* subsystem1_init() has not run yet */
    }
    now_ms = (uint32_t)app_time_ms();

    if (!gb_flag_synced)
    {
        comms_set_wifi_loss(!gb_online);
    }

    /* 1. Offline ring first, so events stay in order. */
    while (gb_online && comms_ring_peek(&g_ring, &rec, &seq))
    {
        if (!comms_publish_event(&rec, seq))
        {
            break; /* lwIP full: retry on the next poll */
        }
        comms_ring_pop(&g_ring);
    }

    /* 2. New records. */
    for (count = 0u; count < COMMS_DRAIN_MAX; count++)
    {
        if (tk_rcv_mbf(mbf, &rec, TMO_POL) != (INT)sizeof(rec))
        {
            break;
        }
        g_event_seq++;
        /* Sent at once when online and nothing older is waiting; kept in the
           offline ring otherwise. */
        b_sent = gb_online && (0u == g_ring.count)
                 && comms_publish_event(&rec, g_event_seq);
        if (!b_sent && comms_ring_push(&g_ring, &rec, g_event_seq))
        {
            g_ring_lost++;
        }
    }

    /* 3. Telemetry, 5 Hz. */
    if (comms_due(now_ms, &g_next_telem_ms, COMMS_TELEM_PERIOD_MS) && gb_online)
    {
        comms_publish_telemetry(now_ms);
    }

    /* 4. Status heartbeat, 1 Hz, retained. */
    if (comms_due(now_ms, &g_next_status_ms, COMMS_STATUS_PERIOD_MS)
        && gb_online)
    {
        comms_publish_status(now_ms);
    }
}

static comms_err_t comms_execute (comms_cmd_t const * p_cmd)
{
    ER           err;
    mission_ev_t event;

    switch (p_cmd->cmd)
    {
        case COMMS_CMD_START:
            event = MISSION_EV_REMOTE_START;
        break;

        case COMMS_CMD_STOP:
            event = MISSION_EV_REMOTE_STOP;
        break;

        case COMMS_CMD_RESET:
            event = MISSION_EV_REMOTE_RESET;
        break;

        case COMMS_CMD_SET:
            event = MISSION_EV_REMOTE_SET;
        break;

        case COMMS_CMD_MAP:
            event = MISSION_EV_REMOTE_MAP; /* MissionTask stores the map */
        break;

        default:
            return COMMS_ERR_BAD_CMD;
    }

    if (g_mbf_mission <= 0)
    {
        return COMMS_ERR_FULL;
    }
    err = mission_post((uint8_t)event, (uint8_t)APP_SRC_COMMS,
                       (uint16_t)p_cmd->param_id, p_cmd->value);
    if (err != E_OK)
    {
        return COMMS_ERR_FULL;
    }
    (void)comms_telem_post((uint8_t)COMMS_KIND_EVENT, (uint8_t)APP_SRC_COMMS,
                           (uint16_t)event, p_cmd->value);
    return COMMS_ERR_NONE;
}

void mqtt_utk_app_rx (char const * p_topic, uint8_t const * p_data,
                      uint16_t len)
{
    comms_cmd_t    cmd;
    comms_err_t    err;
    mission_snap_t mission;
    uint16_t       ack_len;

    if ((comms_telem_id() <= 0) || !topic_eq(p_topic, COMMS_TOPIC_CMD))
    {
        return;
    }

    err = comms_parse_cmd(p_data, len, &cmd);
    if (COMMS_ERR_NONE == err)
    {
        uint32_t             now_ms = (uint32_t)app_time_ms();
        comms_dedup_result_t dup =
            comms_cmd_repeat_safe(cmd.cmd)
                ? COMMS_DEDUP_NEW
                : comms_dedup_check(&g_dedup, &cmd, now_ms);

        if (COMMS_DEDUP_REPEAT == dup)
        {
            /* QoS 1 redelivery: acknowledge again, never execute twice. */
        }
        else if (COMMS_DEDUP_CONFLICT == dup)
        {
            err = COMMS_ERR_REUSED;
        }
        else
        {
            mission_get_snap(&mission);
            err = comms_check_cmd(&cmd,
                                  (uint8_t)MISSION_ST_IDLE == mission.state);
            if (COMMS_ERR_NONE == err)
            {
                err = comms_check_ready(
                    &cmd,
                    MISSION_CAL_ALL == (mission.calibrated & MISSION_CAL_ALL));
            }
            if (COMMS_ERR_NONE == err)
            {
                err = comms_execute(&cmd);
            }
            if ((COMMS_ERR_NONE == err) && !comms_cmd_repeat_safe(cmd.cmd))
            {
                comms_dedup_add(&g_dedup, &cmd, now_ms);
            }
        }
    }

    ack_len = comms_build_ack(&cmd, err, g_tx, (uint16_t)sizeof(g_tx));
    (void)comms_publish(COMMS_TOPIC_ACK, ack_len, 1u, 0u);
}

#else /* !TM_MQTT */

/* MQTT is not built: nothing drains mbf_telem. */
typedef int32_t s1_comms_service_unused_t;

#endif /* TM_MQTT */

/*** end of file ***/
