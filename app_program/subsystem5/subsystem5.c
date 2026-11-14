/*
 *  subsystem5.c
 *  Subsystem 5 -- Obstacle and mission: kernel objects and snapshots.
 *  Owner: Buddy 5 (Ryan).  Tasks: mission_task.c, obstacle_task.c.
 *  Logic: mission_core.c, obstacle_core.c.
 */

#include "subsystem5.h"
#include "mission_core.h"
#include "obstacle_core.h"

EXPORT ID g_mbf_mission;
EXPORT ID g_mbf_scan;
EXPORT ID g_flg_obst;

/* Snapshots are stored packed (Change Requirement 1).  The writer packs
   and the readers unpack in their own task; only whole records are copied
   under the lock, so nobody read-modify-writes the shared words. */
static app_lock_t       g_mission_lock;
static uint32_t         g_mission_snap[MISSION_SNAP_WORDS];
static app_lock_t       g_obst_lock;
static uint32_t         g_obst_snap[OBST_SNAP_WORDS];
static UW               g_drops;
static ID               g_cyc_range;

/* range_cyc: every 60 ms, the HC-SR04 minimum cycle. */
static void range_cyc_handler(void * p_exinf)
{
    (void)p_exinf;
    (void)tk_set_flg(g_flg_obst, OBST_FLG_RANGE);
}

EXPORT ER subsystem5_init(void)
{
    T_CMBF  cmbf = {0};
    T_CFLG  cflg = {0};
    T_CCYC  ccyc = {0};

    app_lock_init(&g_mission_lock);
    app_lock_init(&g_obst_lock);
    {
        mission_snap_t ms = {0};
        obst_snap_t    os = {0};

        ms.state       = (uint8_t)MISSION_ST_BOOT;
        ms.pending_nav = (uint8_t)MISSION_NAV_NONE;
        os.d_mm        = OBST_RANGE_NONE;
        mission_snap_pack(&ms, g_mission_snap);
        obst_snap_pack(&os, g_obst_snap);
    }

    /* Messages are one packed word each (CR1); the kernel adds a 4-byte
       header and rounds the message up to 4 bytes (messagebuf.h). */
    cmbf.mbfatr  = TA_TFIFO;
    cmbf.bufsz   = S5_MBF_MISSION_DEPTH * (sizeof(INT) + sizeof(uint32_t));
    cmbf.maxmsz  = sizeof(uint32_t);
    g_mbf_mission = tk_cre_mbf(&cmbf);
    if (g_mbf_mission < E_OK)
    {
        return g_mbf_mission;
    }

    cmbf.bufsz  = S5_MBF_SCAN_DEPTH * (sizeof(INT) + sizeof(INT));   /* 2 B rounded to 4 */
    cmbf.maxmsz = sizeof(uint16_t);
    g_mbf_scan  = tk_cre_mbf(&cmbf);
    if (g_mbf_scan < E_OK)
    {
        return g_mbf_scan;
    }

    cflg.flgatr  = TA_TFIFO | TA_WSGL;  /* only ObstacleTask waits */
    cflg.iflgptn = 0;
    g_flg_obst   = tk_cre_flg(&cflg);
    if (g_flg_obst < E_OK)
    {
        return g_flg_obst;
    }

    ccyc.cycatr = TA_HLNG;
    ccyc.cychdr = (FP)range_cyc_handler;
    ccyc.cyctim = S5_RANGE_PERIOD_MS;
    ccyc.cycphs = S5_RANGE_PERIOD_MS;
    g_cyc_range = tk_cre_cyc(&ccyc);
    return (g_cyc_range < E_OK) ? g_cyc_range : E_OK;
}

EXPORT ER subsystem5_start(void)
{
    ID tid;

    tid = app_start_task((FP)mission_task, S5_PRI_MISSION, S5_CORE, "mission");
    if (tid < E_OK)
    {
        return tid;
    }
    tid = app_start_task((FP)obstacle_task, S5_PRI_OBSTACLE, S5_CORE, "obstacle");
    if (tid < E_OK)
    {
        return tid;
    }
    return tk_sta_cyc(g_cyc_range);
}

EXPORT ER mission_post(uint8_t type, uint8_t src, uint16_t a, int32_t data)
{
    mission_evt_t evt;
    uint32_t      word;
    ER            er;

    evt.type = type;
    evt.src  = src;
    evt.a    = a;
    evt.data = data;
    word = mission_evt_pack(&evt);
    er = tk_snd_mbf(g_mbf_mission, &word, sizeof(word), TMO_POL);
    if (er != E_OK)
    {
        app_atomic_inc(&g_drops);       /* senders on both cores */
    }
    return er;
}

EXPORT uint32_t mission_post_drops(void)
{
    return g_drops;
}

EXPORT ER scan_request(uint8_t type, uint8_t seq, int16_t param)
{
    scan_req_t req;
    uint16_t   word;
    ER         er;

    req.type  = type;
    req.seq   = seq;
    req.param = param;
    word = scan_req_pack(&req);
    er = tk_snd_mbf(g_mbf_scan, &word, sizeof(word), TMO_POL);
    (void)tk_set_flg(g_flg_obst, OBST_FLG_REQ);
    return er;
}

EXPORT void mission_get_snap(mission_snap_t * p_out)
{
    uint32_t w[MISSION_SNAP_WORDS];
    uint32_t i;
    UINT     sts = app_lock(&g_mission_lock);

    for (i = 0u; i < MISSION_SNAP_WORDS; i++)
    {
        w[i] = g_mission_snap[i];
    }
    app_unlock(&g_mission_lock, sts);
    mission_snap_unpack(w, p_out);
}

EXPORT void mission_put_snap(mission_snap_t const * p_in)
{
    uint32_t w[MISSION_SNAP_WORDS];
    UINT     sts;
    uint32_t i;

    mission_snap_pack(p_in, w);
    sts = app_lock(&g_mission_lock);
    for (i = 0u; i < MISSION_SNAP_WORDS; i++)
    {
        g_mission_snap[i] = w[i];
    }
    app_unlock(&g_mission_lock, sts);
}

EXPORT void obst_get_snap(obst_snap_t * p_out)
{
    uint32_t w[OBST_SNAP_WORDS];
    uint32_t i;
    UINT     sts = app_lock(&g_obst_lock);

    for (i = 0u; i < OBST_SNAP_WORDS; i++)
    {
        w[i] = g_obst_snap[i];
    }
    app_unlock(&g_obst_lock, sts);
    obst_snap_unpack(w, p_out);
}

EXPORT void obst_put_snap(obst_snap_t const * p_in)
{
    uint32_t w[OBST_SNAP_WORDS];
    UINT     sts;
    uint32_t i;

    obst_snap_pack(p_in, w);
    sts = app_lock(&g_obst_lock);
    for (i = 0u; i < OBST_SNAP_WORDS; i++)
    {
        g_obst_snap[i] = w[i];
    }
    app_unlock(&g_obst_lock, sts);
}
