/*
 *  subsystem3.h
 *  Subsystem 3 -- IR line following and Code 39 barcode decoding
 *  (LineBarcodeTask).  Owner: Buddy 3 (Ho Boon Hean).
 *
 *  PUBLIC interface of subsystem 3.  Week 6 design review: Sections 3,
 *  4 and 6.2.  Outputs: DRIVE -> mbf_motion (IF-01, with LINE
 *  authority); BARCODE / JUNCTION / LINE_LOST / LINE_FOUND -> mbf_mission
 *  (IF-06); the line snapshot (IF-04, format in line_types.h).
 */

#ifndef SUBSYSTEM3_H
#define SUBSYSTEM3_H

#include <stdint.h>
#include "../app.h"
#include "line_types.h"

#define S3_PRI_LINE             7           /* LineBarcodeTask, core 1 */
#define S3_CORE_LINE            APP_CORE1

#define S3_PIN_IR_LINE_L        26u         /* ADC0 */
#define S3_PIN_IR_LINE_R        27u         /* ADC1 */
#define S3_PIN_IR_BARCODE       28u         /* ADC2 (VBAT jumper open) */
#define S3_PIN_TIMING           7u          /* spare GP7, APP_TIMING_PINS=1 */

IMPORT void line_get_snap(line_snap_t * p_out);         /* any task       */
IMPORT void line_put_snap(line_snap_t const * p_in);    /* LineBarcodeTask only */

/* Private to subsystem 3 */
IMPORT void line_task(INT stacd, void * p_exinf);
IMPORT ID   g_tid_line;

#endif /* SUBSYSTEM3_H */
