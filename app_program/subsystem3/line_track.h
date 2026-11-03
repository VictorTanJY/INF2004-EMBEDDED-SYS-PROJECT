/*
 *  line_track.h
 *  Subsystem 3 -- region A of the Line & Barcode statechart (Week 6 6.2):
 *  calibration, normalisation, steering error, junction / gap / line-lost
 *  detection.  Owner: Buddy 3.  Pure C (host-tested).
 *
 *  Called once per 10 ms block with the block's mean normalised readings
 *  and the distance travelled in that block.
 */

#ifndef LINE_TRACK_H
#define LINE_TRACK_H

#include <stdint.h>
#include <stdbool.h>
#include "line_cfg.h"

/* ---- Calibration (CALIBRATING: 360 deg spin, Week 6 Section 5) ------- */
typedef struct
{
    uint16_t min[LINE_CH_COUNT];
    uint16_t max[LINE_CH_COUNT];
    uint16_t white[LINE_CH_COUNT];      /* raw level of white */
    uint16_t black[LINE_CH_COUNT];      /* raw level of black */
    uint16_t contrast_x10;              /* worst line-sensor contrast */
    bool     b_valid;
} line_cal_t;

void     line_cal_start(line_cal_t * p_cal);
void     line_cal_sample(line_cal_t * p_cal, uint16_t const raw[LINE_CH_COUNT]);
/* Computes white / black; true if every sensor reached LINE_CONTRAST_MIN_X10.
   Keeps the previous calibration if it fails. */
bool     line_cal_finish(line_cal_t * p_cal);
/* Raw 12-bit -> 0..1000, 1000 = black. */
uint16_t line_cal_norm(line_cal_t const * p_cal, uint8_t ch, uint16_t raw);
/* After each barcode read: move the barcode channel's black level toward
   the darkest bar seen (its spin calibration only sees the line). */

/* ---- Tracking -------------------------------------------------------- */
typedef enum
{
    LINE_TRK_EV_NONE     = 0x00u,
    LINE_TRK_EV_JUNCTION = 0x01u,
    LINE_TRK_EV_LOST     = 0x02u,
    LINE_TRK_EV_FOUND    = 0x04u
} line_trk_ev_t;

typedef struct
{
    bool     b_black_l;         /* fully black (junction test)  */
    bool     b_black_r;
    bool     b_white_l;         /* fully white (gap / lost test) */
    bool     b_white_r;
    bool     b_lost;
    bool     b_junction;        /* inside a junction (both black, latched) */
    bool     b_have_err;
    int32_t  err;               /* -1000..1000 */
    int32_t  prev_err;
    int32_t  line_err;          /* last error while on the line       */
    int32_t  exit_err;          /* peak_err when both went white      */
    int32_t  peak_err;          /* strongest err of the last LINE_EXIT_PEAK_UM */
    uint32_t peak_um;           /* distance since peak_err was set    */
    int32_t  omega;             /* mrad/s */
    int32_t  v;                 /* mm/s   */
    uint32_t both_black_um;
    uint32_t both_white_um;
    uint32_t straight_um;       /* distance since |omega| last exceeded 0.5 rad/s */
    uint32_t since_junction_um;
} line_track_t;

typedef struct
{
    uint32_t junction_um;       /* FR2: 20 mm (param 5) */
    uint32_t lost_um;           /* FR2: 80 mm (param 4) */
    int32_t  cruise_mm_s;       /* speed cap already applied */
} line_trk_cfg_t;

void    line_track_init(line_track_t * p_t);

/* Returns LINE_TRK_EV_* bits; p_t->v / p_t->omega hold the DRIVE command. */
uint8_t line_track_step(line_track_t * p_t, line_trk_cfg_t const * p_cfg,
                        uint16_t norm_l, uint16_t norm_r, uint32_t step_um);

/* ---- Turn slip (calibration spin, 3 Nov) ---------------------------- */
typedef struct
{
    int16_t  h_ddeg[LINE_SLIP_MAX_BLOCKS];  /* encoder heading turned since the start */
    uint16_t raw_l[LINE_SLIP_MAX_BLOCKS];   /* block means, raw ADC                    */
    uint16_t raw_r[LINE_SLIP_MAX_BLOCKS];
    uint16_t n;
    int32_t  prev_cdeg;
    int32_t  acc_cdeg;
} line_slip_t;

void     line_slip_start(line_slip_t * p_s, int32_t heading_cdeg);
void     line_slip_sample(line_slip_t * p_s, int32_t heading_cdeg, uint16_t raw_l, uint16_t raw_r);
/* After line_cal_finish(): encoder deg per real deg x 1000, from where the
   line sensors cross the line again; 0 if not found or implausible. */
uint16_t line_slip_gain(line_slip_t const * p_s, line_cal_t const * p_cal);

#endif /* LINE_TRACK_H */
