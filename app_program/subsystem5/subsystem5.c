/*
 *  subsystem5.c
 *  Subsystem 5 -- Obstacle and mission: kernel objects and snapshots.
 *  Owner: Buddy 5 (Ryan).  Tasks: mission_task.c, obstacle_task.c.
 *  Logic: mission_core.c, obstacle_core.c.
 */

#include "subsystem5.h"

EXPORT ID g_mbf_mission;
EXPORT ID g_mbf_scan;
EXPORT ID g_flg_obst;

static app_lock_t       g_mission_lock;
static mission_snap_t   g_mission_snap;
static app_lock_t       g_obst_lock;
static obst_snap_t      g_obst_snap;
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
    g_mission_snap.state       = (uint8_t)MISSION_ST_BOOT;
    g_mission_snap.pending_nav = (uint8_t)MISSION_NAV_NONE;
    g_obst_snap.d_mm           = OBST_RANGE_NONE;

    cmbf.mbfatr  = TA_TFIFO;
    cmbf.bufsz   = S5_MBF_MISSION_DEPTH * (sizeof(INT) + sizeof(mission_evt_t));
    cmbf.maxmsz  = sizeof(mission_evt_t);
    g_mbf_mission = tk_cre_mbf(&cmbf);
    if (g_mbf_mission < E_OK)
    {
        return g_mbf_mission;
    }

    cmbf.bufsz  = S5_MBF_SCAN_DEPTH * (sizeof(INT) + sizeof(scan_req_t));
    cmbf.maxmsz = sizeof(scan_req_t);
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
    ER            er;

    evt.type = type;
    evt.src  = src;
    evt.a    = a;
    evt.data = data;
    er = tk_snd_mbf(g_mbf_mission, &evt, sizeof(evt), TMO_POL);
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
    ER         er;

    req.type  = type;
    req.seq   = seq;
    req.param = param;
    er = tk_snd_mbf(g_mbf_scan, &req, sizeof(req), TMO_POL);
    (void)tk_set_flg(g_flg_obst, OBST_FLG_REQ);
    return er;
}

EXPORT void mission_get_snap(mission_snap_t * p_out)
{
    UINT sts = app_lock(&g_mission_lock);

    *p_out = g_mission_snap;
    app_unlock(&g_mission_lock, sts);
}

EXPORT void mission_put_snap(mission_snap_t const * p_in)
{
    UINT sts = app_lock(&g_mission_lock);

    g_mission_snap = *p_in;
    app_unlock(&g_mission_lock, sts);
}

EXPORT void obst_get_snap(obst_snap_t * p_out)
{
    UINT sts = app_lock(&g_obst_lock);

    *p_out = g_obst_snap;
    app_unlock(&g_obst_lock, sts);
}

EXPORT void obst_put_snap(obst_snap_t const * p_in)
{
    UINT sts = app_lock(&g_obst_lock);

    g_obst_snap = *p_in;
    app_unlock(&g_obst_lock, sts);
}
