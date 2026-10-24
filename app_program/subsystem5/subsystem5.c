/*
 *  subsystem5.c
 *  Subsystem 5 -- Obstacle and mission.  Owner: Buddy 5 (Ryan).
 *
 *  STATUS: shared-base stub.  Creates mbf_mission, mbf_scan and
 *  flg_obst, and provides the mission and obstacle snapshots.
 *  TODO (owner): MissionTask (statechart, motion authority, rule for
 *  commands already queued in mbf_motion when authority changes),
 *  ObstacleTask, range_cyc, echo_isr (32-bit TIMERAWL difference, clamp
 *  >= 65535 us to "no echo"), drv_ultrasonic, drv_servo.
 */

#include "subsystem5.h"

EXPORT ID g_mbf_mission;
EXPORT ID g_mbf_scan;
EXPORT ID g_flg_obst;

static app_lock_t       g_mission_lock;
static mission_snap_t   g_mission_snap;
static app_lock_t       g_obst_lock;
static obst_snap_t      g_obst_snap;

EXPORT ER subsystem5_init(void)
{
    T_CMBF  cmbf = {0};
    T_CFLG  cflg = {0};

    app_lock_init(&g_mission_lock);
    app_lock_init(&g_obst_lock);
    g_mission_snap.state = (uint8_t)MISSION_ST_BOOT;
    g_obst_snap.d_mm     = OBST_RANGE_NONE;

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

    return (g_flg_obst < E_OK) ? g_flg_obst : E_OK;
}

EXPORT ER subsystem5_start(void)
{
    /* TODO (owner): start MissionTask (S5_PRI_MISSION) and ObstacleTask
       (S5_PRI_OBSTACLE), both on S5_CORE. */
    return E_OK;
}

EXPORT ER mission_post(uint8_t type, uint8_t src, uint16_t a, int32_t data)
{
    mission_evt_t evt;

    evt.type = type;
    evt.src  = src;
    evt.a    = a;
    evt.data = data;
    return tk_snd_mbf(g_mbf_mission, &evt, sizeof(evt), TMO_POL);
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
