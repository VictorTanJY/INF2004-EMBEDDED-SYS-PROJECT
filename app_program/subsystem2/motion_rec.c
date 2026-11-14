/*
 *  motion_rec.c
 *  Subsystem 2 -- speed-step recorder.  See motion_rec.h.
 */

#include "motion_rec.h"

#define REC_FIELD_MIN   (-2048)
#define REC_FIELD_MAX   2047
#define REC_FIELD_MASK  0xFFFu

static uint32_t field12(int32_t v)
{
    int32_t const c = (v < REC_FIELD_MIN) ? REC_FIELD_MIN : ((v > REC_FIELD_MAX) ? REC_FIELD_MAX : v);

    return (uint32_t)c & REC_FIELD_MASK;
}

int32_t motion_rec_pack(uint8_t i, int32_t a, int32_t b)
{
    return (int32_t)(((uint32_t)i << 24) | (field12(a) << 12) | field12(b));
}

void motion_rec_init(motion_rec_t * p_rec)
{
    p_rec->n     = 0u;
    p_rec->tx    = 0u;
    p_rec->state = (uint8_t)MOTION_REC_IDLE;
}

void motion_rec_start(motion_rec_t * p_rec)
{
    p_rec->n     = 0u;
    p_rec->tx    = 0u;
    p_rec->state = (uint8_t)MOTION_REC_RECORDING;
}

void motion_rec_add(motion_rec_t * p_rec, motion_rec_sample_t const * p_s)
{
    if (p_rec->state != (uint8_t)MOTION_REC_RECORDING)
    {
        return;
    }
    p_rec->s[p_rec->n] = *p_s;
    p_rec->n++;
    if (p_rec->n >= MOTION_REC_N)
    {
        p_rec->state = (uint8_t)MOTION_REC_SENDING;
        p_rec->tx    = 0u;
    }
}

bool motion_rec_peek(motion_rec_t const * p_rec, uint16_t * p_code, int32_t * p_data)
{
    uint16_t i;

    if ((p_rec->state != (uint8_t)MOTION_REC_SENDING) || (p_rec->tx > (3u * p_rec->n)))
    {
        return false;
    }
    if (p_rec->tx == 0u)
    {
        *p_code = (uint16_t)MOTION_REC_LOG_HEADER;
        *p_data = (int32_t)((uint32_t)p_rec->n | ((uint32_t)MOTION_REC_PERIOD_MS << 16));
        return true;
    }
    i = (uint16_t)((p_rec->tx - 1u) / 3u);
    switch ((p_rec->tx - 1u) % 3u)
    {
        case 0u:
            *p_code = (uint16_t)MOTION_REC_LOG_LEFT;
            *p_data = motion_rec_pack((uint8_t)i, p_rec->s[i].v_l, p_rec->s[i].duty_l);
            break;
        case 1u:
            *p_code = (uint16_t)MOTION_REC_LOG_RIGHT;
            *p_data = motion_rec_pack((uint8_t)i, p_rec->s[i].v_r, p_rec->s[i].duty_r);
            break;
        default:
            *p_code = (uint16_t)MOTION_REC_LOG_SET;
            *p_data = motion_rec_pack((uint8_t)i, p_rec->s[i].sp_l, p_rec->s[i].sp_r);
            break;
    }
    return true;
}

void motion_rec_sent(motion_rec_t * p_rec)
{
    if (p_rec->state != (uint8_t)MOTION_REC_SENDING)
    {
        return;
    }
    p_rec->tx++;
    if (p_rec->tx > (3u * p_rec->n))
    {
        p_rec->state = (uint8_t)MOTION_REC_IDLE;
    }
}
