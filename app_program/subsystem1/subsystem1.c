/*
 *  subsystem1.c
 *  Subsystem 1 -- Comms and HMI: creates mbf_telem and starts HmiTask.
 *  Owner: Buddy 1 (Victor).
 *
 *  CommsTask is not created here: it is the template's WiFi service task
 *  (cyw43_utk_task), started by the kernel before usermain().  Its MQTT
 *  client (lib/liblwip/.../mqtt_utk.c) calls the hooks in
 *  comms_service.c.  Because that task may already be running, the
 *  service does nothing until g_mbf_telem holds a valid ID.
 */

#include "subsystem1.h"

EXPORT ID g_mbf_telem;

static UW g_telem_drops;

EXPORT ER subsystem1_init(void)
{
    T_CMBF  cmbf = {0};
    ID      id;

    cmbf.mbfatr = TA_TFIFO;
    cmbf.bufsz  = S1_MBF_TELEM_BYTES;
    cmbf.maxmsz = sizeof(comms_rec_t);
    id = tk_cre_mbf(&cmbf);
    if (id < E_OK)
    {
        return id;
    }
    /* The WiFi task may already be polling (it starts before usermain).
       comms_service.c keeps only zero-initialised static state and does
       nothing until it sees a valid ID here, so publishing the ID is the
       only hand-over needed: one aligned word store. */
    g_mbf_telem = id;
    return E_OK;
}

EXPORT ER subsystem1_start(void)
{
    ID tid = app_start_task(hmi_task, S1_PRI_HMI, S1_CORE_HMI, "hmi_task");

    return (tid < E_OK) ? tid : E_OK;
}

EXPORT bool comms_telem_post(uint8_t kind, uint8_t src, uint16_t code, int32_t data)
{
    comms_rec_t rec;

    if (g_mbf_telem <= 0)
    {
        app_atomic_inc(&g_telem_drops);
        return false;
    }
    rec.kind = kind;
    rec.src  = src;
    rec.code = code;
    rec.data = data;
    rec.t_ms = (uint32_t)app_time_ms();
    if (tk_snd_mbf(g_mbf_telem, &rec, sizeof(rec), TMO_POL) != E_OK)
    {
        app_atomic_inc(&g_telem_drops);
        return false;
    }
    return true;
}

EXPORT uint32_t comms_telem_drops(void)
{
    return (uint32_t)g_telem_drops;
}
