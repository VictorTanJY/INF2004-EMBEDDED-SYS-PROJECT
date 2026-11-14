/*
 *  motion_rec.h
 *  Subsystem 2 -- speed-step recorder for the PID tuning report (NFR5).
 *  Owner: Buddy 2.
 *
 *  Used by MotionTask in APP_BENCH builds only.  When a MOVE starts,
 *  every 10 ms control step for MOTION_REC_N steps is kept: each wheel's
 *  set-point, measured speed and duty.  The telemetry runs at 5 Hz, too
 *  slow to see a 300 ms settle, so afterwards the samples are sent as log
 *  records, two per step of MotionTask:
 *
 *    LOG 84  header: data = sample count | period_ms << 16 (sent first)
 *    LOG 81  left:   data = i << 24 | v_l  (12 bits) << 12 | duty_l (12 bits)
 *    LOG 82  right:  data = i << 24 | v_r  (12 bits) << 12 | duty_r (12 bits)
 *    LOG 83  set:    data = i << 24 | sp_l (12 bits) << 12 | sp_r   (12 bits)
 *
 *  i = sample index (0 = the step the MOVE started); speeds in mm/s and
 *  duty in per mille, each a 12-bit two's complement field (-2048..2047,
 *  clamped).  The dashboard joins them back into a CSV.
 *
 *  Pure C, no kernel calls: the host tests link it directly.
 */
#ifndef MOTION_REC_H
#define MOTION_REC_H

#include <stdint.h>
#include <stdbool.h>

#define MOTION_REC_N            150u    /* 1.5 s of 10 ms steps          */
#define MOTION_REC_PERIOD_MS    10u
#define MOTION_REC_LOG_LEFT     81u
#define MOTION_REC_LOG_RIGHT    82u
#define MOTION_REC_LOG_SET      83u
#define MOTION_REC_LOG_HEADER   84u

typedef struct
{
    int16_t sp_l;
    int16_t sp_r;
    int16_t v_l;
    int16_t v_r;
    int16_t duty_l;
    int16_t duty_r;
} motion_rec_sample_t;

typedef enum
{
    MOTION_REC_IDLE      = 0,
    MOTION_REC_RECORDING = 1,
    MOTION_REC_SENDING   = 2
} motion_rec_state_t;

typedef struct
{
    motion_rec_sample_t s[MOTION_REC_N];
    uint16_t            n;          /* samples held                          */
    uint16_t            tx;         /* next record to send: 0 = header, then 3 per sample */
    uint8_t             state;      /* motion_rec_state_t                    */
} motion_rec_t;

void     motion_rec_init(motion_rec_t * p_rec);
/* A MOVE started: drop anything held or being sent and record afresh. */
void     motion_rec_start(motion_rec_t * p_rec);
/* One control step (only kept while recording). */
void     motion_rec_add(motion_rec_t * p_rec, motion_rec_sample_t const * p_s);
/* The next record to send, if any (does not move on). */
bool     motion_rec_peek(motion_rec_t const * p_rec, uint16_t * p_code, int32_t * p_data);
/* That record was posted: move on. */
void     motion_rec_sent(motion_rec_t * p_rec);
/* i << 24 | a << 12 | b, a and b clamped to 12-bit two's complement. */
int32_t  motion_rec_pack(uint8_t i, int32_t a, int32_t b);

#endif /* MOTION_REC_H */
