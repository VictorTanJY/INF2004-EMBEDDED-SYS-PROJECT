/*
 *  drv_servo.h
 *  Subsystem 5 -- scan servo on GP15 = PWM slice 7 B, 50 Hz (Week 6 2.2).
 *  Register-level.  Slice 7 is separate from the motor slices 4/5, so its
 *  50 Hz timebase cannot disturb the 20 kHz motor PWM.  Owner: Buddy 5.
 */

#ifndef DRV_SERVO_H
#define DRV_SERVO_H

#include <stdint.h>
#include "../app.h"

IMPORT ER   drv_servo_init(void);
/* 0..180 deg, 90 = straight ahead, larger = further left (mission_cfg.h) */
IMPORT void drv_servo_set_deg(int32_t angle_deg);

#endif /* DRV_SERVO_H */
