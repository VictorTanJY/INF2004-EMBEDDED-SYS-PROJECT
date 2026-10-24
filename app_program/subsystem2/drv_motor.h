/*
 *  drv_motor.h
 *  Subsystem 2 -- Robo Pico motor driver on PWM slices 4 and 5 (GP8-GP11),
 *  20 kHz.  Register-level (A3: no SDK hardware_pwm in this port).
 *  Owner: Buddy 2.  Used by MotionTask (and motor_estop) only.
 */

#ifndef DRV_MOTOR_H
#define DRV_MOTOR_H

#include <stdint.h>
#include "../app.h"

/* Pins driven LOW (brake) as SIO outputs, PWM configured with duty 0 but
   NOT yet connected to the pins.  Safe to call while the E-stop latch is set. */
IMPORT ER   drv_motor_init(void);

/* Route GP8-GP11 to PWM (duty is whatever was last written, 0 after init).
   Call only with interrupts disabled and the E-stop latch clear. */
IMPORT void drv_motor_connect(void);

/* Per-mille duty, + = forward (MOTION_MOT_SIGN_* applied here).  One input
   of each motor is always 0: A = PWM / B = 0 forward, A = 0 / B = PWM
   reverse, 0 / 0 brake; both inputs high (coast) cannot be produced.
   Both halves of a slice's CC register are written in one 32-bit store. */
IMPORT void drv_motor_write(int16_t duty_l, int16_t duty_r);

#endif /* DRV_MOTOR_H */
