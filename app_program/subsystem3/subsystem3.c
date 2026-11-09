/*
 *  subsystem3.c
 *  Subsystem 3 -- Line following and barcode: objects and the line
 *  snapshot.  Owner: Buddy 3.  The task is in line_task.c; the pure logic
 *  in line_track.c, line_barcode.c and code39.c; the driver in drv_adc.c.
 */

#include "subsystem3.h"

EXPORT ID g_tid_line;

static app_lock_t   g_line_lock;
static line_snap_t  g_line_snap;

EXPORT ER subsystem3_init(void)
{
    app_lock_init(&g_line_lock);
    return E_OK;
}

EXPORT ER subsystem3_start(void)
{
    g_tid_line = app_start_task((FP)line_task, S3_PRI_LINE, S3_CORE_LINE, "line");
    return (g_tid_line < E_OK) ? g_tid_line : E_OK;
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
