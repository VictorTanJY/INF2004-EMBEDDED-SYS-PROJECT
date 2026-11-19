/*
 *  subsystem3.c
 *  Subsystem 3 -- Line following and barcode: objects and the line
 *  snapshot.  Owner: Buddy 3.  The task is in line_task.c; the pure logic
 *  in line_track.c, line_barcode.c and code39.c; the driver in drv_adc.c.
 */

#include "subsystem3.h"
#include "line_track.h"

EXPORT ID g_tid_line;

/* Stored packed (Change Requirement 1); whole records copied under the
   lock, packing and unpacking done in the writer's / reader's own task. */
static app_lock_t g_line_lock;
static uint32_t   g_line_snap[LINE_SNAP_WORDS];

EXPORT ER subsystem3_init (void)
{
    app_lock_init(&g_line_lock);
    {
        line_snap_t const zero = {0};

        line_snap_pack(&zero,
                       g_line_snap); /* packed zero (offset-binary fields) */
    }
    return E_OK;
}

EXPORT ER subsystem3_start (void)
{
    g_tid_line =
        app_start_task((FP)line_task, S3_PRI_LINE, S3_CORE_LINE, "line");
    return (g_tid_line < E_OK) ? g_tid_line : E_OK;
}

EXPORT void line_get_snap (line_snap_t * p_out)
{
    uint32_t words[LINE_SNAP_WORDS];
    uint32_t idx;
    UINT     sts = app_lock(&g_line_lock);

    for (idx = 0u; idx < LINE_SNAP_WORDS; idx++)
    {
        words[idx] = g_line_snap[idx];
    }
    app_unlock(&g_line_lock, sts);
    line_snap_unpack(words, p_out);
}

EXPORT void line_put_snap (line_snap_t const * p_in)
{
    uint32_t words[LINE_SNAP_WORDS];
    uint32_t idx;
    UINT     sts;

    line_snap_pack(p_in, words);
    sts = app_lock(&g_line_lock);
    for (idx = 0u; idx < LINE_SNAP_WORDS; idx++)
    {
        g_line_snap[idx] = words[idx];
    }
    app_unlock(&g_line_lock, sts);
}

/*** end of file ***/
