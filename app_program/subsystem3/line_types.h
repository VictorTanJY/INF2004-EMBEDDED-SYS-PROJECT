/*
 *  line_types.h
 *  Subsystem 3 -- line snapshot format (IF-04).  Owner: Buddy 3 (Boon).
 *  Pure C (no kernel).  subsystem3.h adds the kernel side.
 */

#ifndef LINE_TYPES_H
#define LINE_TYPES_H

#include <stdint.h>

/* line_snap_t.flags */
#define LINE_FLAG_CALIBRATED    0x01u
#define LINE_FLAG_LOST          0x02u       /* both white >= lost_mm (80)       */
#define LINE_FLAG_JUNCTION      0x04u       /* both black >= junction_mm, straight */
#define LINE_FLAG_HAS_AUTH      0x08u       /* currently holds LINE authority   */
#define LINE_FLAG_ON_LINE       0x10u       /* at least one line sensor on black */
#define LINE_FLAG_HW_OK         0x20u       /* ADC + DMA running                */
#define LINE_FLAG_GAP           0x40u       /* bridging a short white gap       */

/* line_snap_t.bc_state */
typedef enum
{
    LINE_BC_WAIT_QUIET = 0,
    LINE_BC_ARMED      = 1,                 /* quiet zone seen, waiting for a bar */
    LINE_BC_CAPTURING  = 2,
    LINE_BC_COOLDOWN   = 3
} line_bc_state_t;

/* IF-04  Line snapshot (written by LineBarcodeTask every 10 ms). */
typedef struct
{
    uint32_t t_ms;              /* time of the update                       */
    uint32_t blocks;            /* 10 ms ADC blocks processed (NFR2 check)  */
    uint32_t missed_blocks;     /* blocks lost (sequence gap, NFR2)         */
    int16_t  err;               /* (L - R)/(L + R) x 1000, + = line to the left */
    int16_t  omega_mrad_s;      /* last steering command                    */
    uint16_t raw_l;             /* normalised 0..1000 (1000 = black)        */
    uint16_t raw_r;
    uint16_t raw_bc;
    uint16_t barcode_ok;        /* valid decodes since boot                 */
    uint16_t barcode_err;       /* rejected reads since boot                */
    uint16_t contrast_x10;      /* worst calibration contrast x 10 (>= 30)  */
    uint8_t  flags;             /* LINE_FLAG_*                              */
    uint8_t  bc_state;          /* line_bc_state_t                          */
    char     last_barcode;      /* last valid Code 39 character, 0 = none   */
    uint8_t  last_reject;       /* code39_result_t of the last reject       */
} line_snap_t;

#endif /* LINE_TYPES_H */
