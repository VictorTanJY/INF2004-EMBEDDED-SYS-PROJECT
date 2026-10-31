/*
 *  line_barcode.h
 *  Subsystem 3 -- region B of the Line & Barcode statechart (Week 6 6.2):
 *  WAIT_QUIET -> ARMED -> CAPTURING -> (VALID | REJECT) -> COOLDOWN.
 *  Owner: Buddy 3.  Pure C (host-tested).
 *
 *  Fed one barcode-sensor sample at a time (1 kHz) with the distance the
 *  robot moved since the previous sample, so every bar / space width is in
 *  micrometres of travel, independent of speed.
 */

#ifndef LINE_BARCODE_H
#define LINE_BARCODE_H

#include <stdint.h>
#include <stdbool.h>
#include "line_cfg.h"
#include "line_types.h"
#include "code39.h"

typedef enum
{
    BC_NONE   = 0,
    BC_VALID  = 1,              /* p_out->ch holds the character        */
    BC_REJECT = 2               /* p_out->reason holds the code39_result_t */
} bc_result_t;

typedef struct
{
    char     ch;
    uint8_t  reason;            /* code39_result_t */
    bool     b_reversed;
    uint16_t elements;
    uint32_t length_um;
    uint16_t peak_norm;         /* darkest bar seen (black-level learning) */
} bc_out_t;

typedef struct
{
    uint8_t  state;             /* line_bc_state_t */
    bool     b_black;           /* hysteresis level */
    uint32_t run_um;            /* current run      */
    uint32_t white_um;          /* WAIT_QUIET: white so far */
    uint32_t total_um;
    uint32_t max_elem_um;
    uint32_t cooldown_um;
    uint16_t n;
    uint16_t peak;
    uint32_t widths[BC_MAX_ELEMENTS + 1u];
} bc_capture_t;

void        bc_init(bc_capture_t * p_bc);

/* norm: 0..1000 (1000 = black).  step_um: distance since the last sample
   (absolute value).  Returns BC_VALID / BC_REJECT once per code. */
bc_result_t bc_sample(bc_capture_t * p_bc, uint16_t norm, uint32_t step_um, bc_out_t * p_out);

#endif /* LINE_BARCODE_H */
