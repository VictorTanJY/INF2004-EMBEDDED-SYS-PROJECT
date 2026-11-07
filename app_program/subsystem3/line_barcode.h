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
    uint16_t ext;               /* extreme of the current element (BC_DELTA) */
    uint32_t widths[BC_MAX_ELEMENTS + 1u];
    uint16_t prof[BC_PROF_BINS];    /* darkest norm per BC_PROF_BIN_UM   */
    uint16_t prof_n;
    uint32_t bin_um;                /* travel into the current bin       */
    uint8_t  pass;                  /* which reading decoded: 0 swing, 1 mid, 2 low */
    /* 7 Nov: white-paper levels over the last BC_QUIET_UM before a code
       (lightest, darkest: their difference is the noise a bar must beat) */
    uint16_t q_lo;
    uint16_t q_hi;
    uint16_t q_wlo;                 /* the window being measured            */
    uint16_t q_whi;
    /* 7 Nov: darkness record of the last BC_PRE_BINS bins while armed */
    uint16_t pre[BC_PRE_BINS];
    uint16_t pre_n;                 /* bins held (up to BC_PRE_BINS)        */
    uint16_t pre_i;                 /* next slot                            */
    uint16_t pre_cur;               /* darkest in the bin being filled      */
    uint32_t pre_um;                /* travel into it                       */
} bc_capture_t;

void        bc_init(bc_capture_t * p_bc);

/* From a darkness profile (bins of BC_PROF_BIN_UM), the elements against
   threshold thr (bars at or above thr + hyst, spaces below thr - hyst);
   runs under BC_PROF_MIN_BINS merge into their neighbours.  Returns the
   element count (widths in um), the trailing white run excluded. */
uint16_t    bc_prof_elements(uint16_t const * p_prof, uint16_t n_bins, uint16_t thr, uint16_t hyst,
                             uint32_t * p_widths, uint16_t max);
/* Light / dark levels of a profile (percentiles BC_PROF_WHITE_PCT / DARK_PCT). */
void        bc_prof_levels(uint16_t const * p_prof, uint16_t n_bins, uint16_t * p_white, uint16_t * p_dark);

/* norm: 0..1000 (1000 = black).  step_um: distance since the last sample
   (absolute value).  Returns BC_VALID / BC_REJECT once per code. */
bc_result_t bc_sample(bc_capture_t * p_bc, uint16_t norm, uint32_t step_um, bc_out_t * p_out);

#endif /* LINE_BARCODE_H */
