/** @file subsystem1.c
 *
 *  @brief Subsystem 1 -- Comms and HMI: creates mbf_telem and starts HmiTask.
 *  Owner: Buddy 1 (Victor).
 *
 *  CommsTask is not created here: it is the template's WiFi service task
 *  (cyw43_utk_task), started by the kernel before usermain().  Its MQTT
 *  client (lib/liblwip/.../mqtt_utk.c) calls the hooks in
 *  comms_service.c.  Because that task may already be running, the
 *  service does nothing until subsystem1_mbf_telem() returns a valid ID.
 */

#include "subsystem1.h"

static ID g_mbf_telem;

static uint32_t g_telem_drops;

EXPORT ER subsystem1_init (void)
{
    T_CMBF cmbf = {0};
    ID     obj_id;
    ER     err = E_OK;

    cmbf.mbfatr = TA_TFIFO;
    cmbf.bufsz  = S1_MBF_TELEM_BYTES;
    cmbf.maxmsz = sizeof(comms_proto_rec_t);
    obj_id      = tk_cre_mbf(&cmbf);
    if (obj_id < E_OK)
    {
        err = obj_id;
    }
    else
    {
        /* The WiFi task may already be polling (it starts before usermain).
           comms_service.c keeps only zero-initialised static state and does
           nothing until it sees a valid ID here, so publishing the ID is the
           only hand-over needed: one aligned word store. */
        g_mbf_telem = obj_id;
    }
    return err;
}

EXPORT ER subsystem1_start (void)
{
    ID tid = app_main_start_task(task_hmi_thread, S1_PRI_HMI, S1_CORE_HMI,
                                 "hmi_task");

    return (tid < E_OK) ? tid : E_OK;
}

EXPORT bool subsystem1_telem_post (uint8_t kind, uint8_t src, uint16_t code,
                                   int32_t data)
{
    comms_proto_rec_t rec;
    bool              b_sent = false;

    if (g_mbf_telem > 0)
    {
        comms_proto_rec_pack(&rec, kind, src, code, data, app_time_ms());
        b_sent = (E_OK == tk_snd_mbf(g_mbf_telem, &rec, sizeof(rec), TMO_POL));
    }
    if (!b_sent)
    {
        app_atomic_inc(&g_telem_drops);
    }
    return b_sent;
}

EXPORT uint32_t subsystem1_telem_drops (void)
{
    return g_telem_drops;
}

EXPORT ID subsystem1_mbf_telem (void)
{
    return g_mbf_telem;
}

/*** end of file ***/
