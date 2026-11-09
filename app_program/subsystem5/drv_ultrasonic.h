/*
 *  drv_ultrasonic.h
 *  Subsystem 5 -- HC-SR04P: TRIG on GP0, ECHO on GP1 (Week 6 2.2).
 *  echo_isr timestamps both edges with the 1 us timer (32-bit difference,
 *  so the 71-minute wrap is harmless) and sets OBST_FLG_ECHO.
 *  Owner: Buddy 5.
 */

#ifndef DRV_ULTRASONIC_H
#define DRV_ULTRASONIC_H

#include <stdint.h>
#include <stdbool.h>
#include "../app.h"

IMPORT ER   drv_us_init(void);
/* Enable echo_isr on the CALLING core (ObstacleTask, core 1). */
IMPORT ER   drv_us_attach(void);
/* 10 us trigger pulse (busy wait). */
IMPORT void drv_us_trigger(void);
/* true once per completed echo; *p_width_us = high time. */
IMPORT bool drv_us_take(uint32_t * p_width_us);

#endif /* DRV_ULTRASONIC_H */
