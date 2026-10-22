/*
 *  subsystem3.c
 *  Subsystem 3 -- Line following and barcode.  Owner: Buddy 3.
 *
 *  STATUS: shared-base stub (line snapshot only).
 *  TODO (owner): drv_adc (round-robin ADC0-ADC2, 3 kHz total, CLKDIV =
 *  48 MHz / 3 kHz - 1 = 15999; DMA channel from dma_claim_unused_channel,
 *  because the radio already claims two), adc_dma_isr registered with
 *  tk_def_int on core 1, LineBarcodeTask, calibration, Code 39 decoder.
 *  (DEVCNF_USE_ADC is set to 0 in config/config_device.h.)
 */

#include "subsystem3.h"

static app_lock_t   g_line_lock;
static line_snap_t  g_line_snap;

EXPORT ER subsystem3_init(void)
{
    app_lock_init(&g_line_lock);
    return E_OK;
}

EXPORT ER subsystem3_start(void)
{
    /* TODO (owner): app_start_task(line_task, S3_PRI_LINE, S3_CORE_LINE, "line"); */
    return E_OK;
}

EXPORT void line_get_snap(line_snap_t * p_out)
{
    UINT sts = app_lock(&g_line_lock);

    *p_out = g_line_snap;
    app_unlock(&g_line_lock, sts);
}

EXPORT void line_put_snap(line_snap_t const * p_in)
{
    UINT sts = app_lock(&g_line_lock);

    g_line_snap = *p_in;
    app_unlock(&g_line_lock, sts);
}
