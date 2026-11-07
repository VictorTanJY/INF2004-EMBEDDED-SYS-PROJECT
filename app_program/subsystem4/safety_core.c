/*
 *  safety_core.c
 *  Subsystem 4 -- heartbeat supervision (pure).  See safety_core.h.
 */

#include "safety_core.h"

static uint32_t const k_timeout_ms[SAFETY_HB_COUNT] = { 50u, 50u, 300u, 300u, 100u };

uint32_t safety_hb_timeout_ms(safety_hb_t task)
{
    return ((uint32_t)task < (uint32_t)SAFETY_HB_COUNT) ? k_timeout_ms[task] : 0u;
}

void safety_hb_start(safety_hb_state_t * p_st, uint32_t const counts[SAFETY_HB_COUNT],
                     uint32_t now_ms)
{
    uint8_t k;

    for (k = 0u; k < (uint8_t)SAFETY_HB_COUNT; k++)
    {
        p_st->last_count[k]   = counts[k];
        p_st->last_seen_ms[k] = now_ms;
    }
    p_st->b_started = true;
}

uint32_t safety_hb_check(safety_hb_state_t * p_st, uint32_t const counts[SAFETY_HB_COUNT],
                         uint32_t now_ms)
{
    uint32_t missed = 0u;
    uint8_t  k;

    if (!p_st->b_started)
    {
        safety_hb_start(p_st, counts, now_ms);
        return 0u;
    }
    for (k = 0u; k < (uint8_t)SAFETY_HB_COUNT; k++)
    {
        if (counts[k] != p_st->last_count[k])
        {
            p_st->last_count[k]   = counts[k];
            p_st->last_seen_ms[k] = now_ms;
        }
        else if ((uint32_t)(now_ms - p_st->last_seen_ms[k]) > k_timeout_ms[k])
        {
            missed |= (1uL << k);
        }
        else
        {
            /* within its timeout */
        }
    }
    return missed;
}
