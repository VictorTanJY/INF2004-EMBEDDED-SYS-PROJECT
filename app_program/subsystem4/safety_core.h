/** @file safety_core.h
 *
 *  @brief Subsystem 4 -- SafetyTask heartbeat supervision as pure functions
 *  (Week 6 Section 3).  Owner: Buddy 4.  Host-tested.
 *
 *  Each monitored task increments its own counter on every pass; SafetyTask
 *  calls safety_core_hb_check() at least every 20 ms.  A task whose counter has
 *  not changed for longer than its timeout is reported as missed.
 *  Timeouts: Motion 50, LineBarcode 50, Mission 300, Obstacle 300, Imu 100 ms.
 */

#ifndef SAFETY_CORE_H
#define SAFETY_CORE_H

#include <stdint.h>
#include <stdbool.h>
#include "imu_types.h"

typedef struct
{
    uint32_t last_count[SAFETY_HB_COUNT];
    uint32_t last_seen_ms[SAFETY_HB_COUNT];
    bool     b_started;
} safety_core_hb_state_t;

/** Heartbeat timeout of a monitored task, ms. */
uint32_t safety_core_hb_timeout_ms(imu_types_safety_hb_t task);

/** Start (or restart, after an operator reset) the supervision now. */
void safety_core_hb_start(safety_core_hb_state_t * p_st,
                          uint32_t const           counts[SAFETY_HB_COUNT],
                          uint32_t                 now_ms);

/** Returns a bit mask (bit k = task k) of the tasks past their timeout. */
uint32_t safety_core_hb_check(safety_core_hb_state_t * p_st,
                              uint32_t const           counts[SAFETY_HB_COUNT],
                              uint32_t                 now_ms);

#endif /* SAFETY_CORE_H */

/*** end of file ***/
