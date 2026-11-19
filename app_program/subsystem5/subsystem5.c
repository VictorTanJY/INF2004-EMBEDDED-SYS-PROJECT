/** @file subsystem5.c
 *
 *  @brief Subsystem 5 -- Obstacle and mission: kernel objects and snapshots.
 *  Owner: Buddy 5 (Ryan).  Tasks: mission_task.c,
 *  obstacle_task.c. Logic: mission_core.c, obstacle_core.c.
 */

#include "subsystem5.h"
#include "mission_core.h"
#include "obstacle_core.h"

static ID g_mbf_mission;
static ID g_mbf_scan;
static ID g_flg_obst;

/** Snapshots are stored packed (Change Requirement 1).  The writer packs
   and the readers unpack in their own task; only whole records are copied
   under the lock, so nobody read-modify-writes the shared words. */
static app_lock_t g_mission_lock;
static uint32_t   g_mission_snap[MISSION_SNAP_WORDS];
static app_lock_t g_obst_lock;
static uint32_t   g_obst_snap[OBST_SNAP_WORDS];
static uint32_t   g_drops;
static ID         g_cyc_range;

static void range_cyc_handler(void * p_exinf);

EXPORT ER subsystem5_init (void)
{
    T_CMBF cmbf = {0};
    T_CFLG cflg = {0};
    T_CCYC ccyc = {0};

    app_lock_init(&g_mission_lock);
    app_lock_init(&g_obst_lock);
    {
        mission_types_snap_t      mission = {0};
        mission_types_obst_snap_t obst    = {0};

        mission.state       = MISSION_ST_BOOT;
        mission.pending_nav = MISSION_NAV_NONE;
        obst.d_mm           = OBST_RANGE_NONE;
        mission_core_snap_pack(&mission, g_mission_snap);
        obstacle_core_snap_pack(&obst, g_obst_snap);
    }

    /* Messages are one packed word each (CR1); the kernel adds a 4-byte
       header and rounds the message up to 4 bytes (messagebuf.h). */
    cmbf.mbfatr   = TA_TFIFO;
    cmbf.bufsz    = S5_MBF_MISSION_DEPTH * (sizeof(INT) + sizeof(uint32_t));
    cmbf.maxmsz   = sizeof(uint32_t);
    g_mbf_mission = tk_cre_mbf(&cmbf);
    if (g_mbf_mission < E_OK)
    {
        return g_mbf_mission;
    }

    cmbf.bufsz =
        S5_MBF_SCAN_DEPTH * (sizeof(INT) + sizeof(INT)); /* 2 B rounded to 4 */
    cmbf.maxmsz = sizeof(uint16_t);
    g_mbf_scan  = tk_cre_mbf(&cmbf);
    if (g_mbf_scan < E_OK)
    {
        return g_mbf_scan;
    }

    cflg.flgatr  = TA_TFIFO | TA_WSGL; /* only ObstacleTask waits */
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

EXPORT ER subsystem5_start (void)
{
    ID tid;

    tid = app_main_start_task((FP)mission_task_thread, S5_PRI_MISSION, S5_CORE,
                              "mission");
    if (tid < E_OK)
    {
        return tid;
    }
    tid = app_main_start_task((FP)obstacle_task_thread, S5_PRI_OBSTACLE,
                              S5_CORE, "obstacle");
    if (tid < E_OK)
    {
        return tid;
    }
    return tk_sta_cyc(g_cyc_range);
}

EXPORT ER subsystem5_mission_post (uint8_t type, uint8_t src, uint16_t arg,
                                   int32_t data)
{
    mission_types_evt_t evt;
    uint32_t            word;
    ER                  err;

    evt.type = type;
    evt.src  = src;
    evt.arg  = arg;
    evt.data = data;
    word     = mission_core_evt_pack(&evt);
    err      = tk_snd_mbf(g_mbf_mission, &word, sizeof(word), TMO_POL);
    if (err != E_OK)
    {
        app_atomic_inc(&g_drops); /* senders on both cores */
    }
    return err;
}

EXPORT uint32_t subsystem5_mission_drops (void)
{
    return g_drops;
}

EXPORT ER subsystem5_scan_request (uint8_t type, uint8_t seq, int16_t param)
{
    mission_types_scan_req_t req;
    uint16_t                 word;
    ER                       err;

    req.type  = type;
    req.seq   = seq;
    req.param = param;
    word      = mission_core_scan_req_pack(&req);
    err       = tk_snd_mbf(g_mbf_scan, &word, sizeof(word), TMO_POL);
    (void)tk_set_flg(g_flg_obst, OBST_FLG_REQ);
    return err;
}

EXPORT void subsystem5_get_mission_snap (mission_types_snap_t * p_out)
{
    uint32_t words[MISSION_SNAP_WORDS];
    uint32_t idx;
    UINT     sts = app_lock(&g_mission_lock);

    for (idx = 0u; idx < MISSION_SNAP_WORDS; idx++)
    {
        words[idx] = g_mission_snap[idx];
    }
    app_unlock(&g_mission_lock, sts);
    mission_core_snap_unpack(words, p_out);
}

EXPORT void subsystem5_put_mission_snap (mission_types_snap_t const * p_in)
{
    uint32_t words[MISSION_SNAP_WORDS];
    UINT     sts;
    uint32_t idx;

    mission_core_snap_pack(p_in, words);
    sts = app_lock(&g_mission_lock);
    for (idx = 0u; idx < MISSION_SNAP_WORDS; idx++)
    {
        g_mission_snap[idx] = words[idx];
    }
    app_unlock(&g_mission_lock, sts);
}

EXPORT void subsystem5_get_obst_snap (mission_types_obst_snap_t * p_out)
{
    uint32_t words[OBST_SNAP_WORDS];
    uint32_t idx;
    UINT     sts = app_lock(&g_obst_lock);

    for (idx = 0u; idx < OBST_SNAP_WORDS; idx++)
    {
        words[idx] = g_obst_snap[idx];
    }
    app_unlock(&g_obst_lock, sts);
    obstacle_core_snap_unpack(words, p_out);
}

EXPORT void subsystem5_put_obst_snap (mission_types_obst_snap_t const * p_in)
{
    uint32_t words[OBST_SNAP_WORDS];
    UINT     sts;
    uint32_t idx;

    obstacle_core_snap_pack(p_in, words);
    sts = app_lock(&g_obst_lock);
    for (idx = 0u; idx < OBST_SNAP_WORDS; idx++)
    {
        g_obst_snap[idx] = words[idx];
    }
    app_unlock(&g_obst_lock, sts);
}

EXPORT ID subsystem5_mbf_mission (void)
{
    return g_mbf_mission;
}

EXPORT ID subsystem5_mbf_scan (void)
{
    return g_mbf_scan;
}

EXPORT ID subsystem5_flg_obst (void)
{
    return g_flg_obst;
}

/** range_cyc: every 60 ms, the HC-SR04 minimum cycle. */
static void range_cyc_handler (void * p_exinf)
{
    (void)p_exinf;
    (void)tk_set_flg(g_flg_obst, OBST_FLG_RANGE);
}

/*** end of file ***/
