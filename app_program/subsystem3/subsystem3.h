/*
 *  subsystem3.h
 *  Subsystem 3 -- IR line following and Code 39 barcode decoding
 *  (LineBarcodeTask).  Owner: Buddy 3 (Ho Boon Hean).
 *
 *  PUBLIC interface of subsystem 3.  Week 6 design review: Sections 3,
 *  4 and 6.2.  Outputs: DRIVE -> mbf_motion (IF-01, with LINE
 *  authority); BARCODE / JUNCTION / LINE_LOST / LINE_FOUND -> mbf_mission
 *  (IF-06); the line snapshot below (IF-04).
 *
 *  STATUS: interface + snapshot only (shared base).  Units marked
 *  PROPOSED must be confirmed by the owner.
 */

#ifndef SUBSYSTEM3_H
#define SUBSYSTEM3_H

#include <stdint.h>
#include "../app.h"

#define S3_PRI_LINE             7           /* LineBarcodeTask, core 1 */
#define S3_CORE_LINE            APP_CORE1

#define S3_PIN_IR_LINE_L        26u         /* ADC0 */
#define S3_PIN_IR_LINE_R        27u         /* ADC1 */
#define S3_PIN_IR_BARCODE       28u         /* ADC2 (VBAT jumper open) */

/* line_snap_t.flags */
#define LINE_FLAG_CALIBRATED    0x01u
#define LINE_FLAG_LOST          0x02u       /* both white >= 80 mm      */
#define LINE_FLAG_JUNCTION      0x04u       /* both black >= 20 mm, straight */
#define LINE_FLAG_HAS_AUTH      0x08u       /* currently holds LINE authority */

/* IF-04  Line snapshot (written by LineBarcodeTask). */
typedef struct
{
    uint32_t t_ms;              /* time of the update                     */
    int16_t  err;               /* (L - R)/(L + R) x 1000, -1000..1000 PROPOSED */
    uint16_t barcode_ok;        /* valid decodes since boot               */
    uint16_t barcode_err;       /* rejected reads since boot              */
    uint16_t raw_l;             /* normalised 0..1000 after calibration   */
    uint16_t raw_r;
    uint16_t raw_bc;
    uint8_t  flags;             /* LINE_FLAG_*                            */
    char     last_barcode;      /* last valid Code 39 character, 0 = none */
    uint8_t  reserved[2];
} line_snap_t;

IMPORT void line_get_snap(line_snap_t * p_out);         /* any task       */
IMPORT void line_put_snap(line_snap_t const * p_in);    /* LineBarcodeTask only */

#endif /* SUBSYSTEM3_H */
