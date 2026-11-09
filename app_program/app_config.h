/*
 *  app_config.h
 *  Build switches for the whole application.  Pure C (also included by the
 *  host tests).  Change a value here, rebuild, flash.
 */

#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* 1: toggle spare pins for timing captures on a logic analyser:
   GP6 = every MotionTask cycle, GP7 = every LineBarcodeTask block
   (NFR1, NFR2, A2).  0 for normal builds. */
#ifndef APP_TIMING_PINS
#define APP_TIMING_PINS     0
#endif

/* 1: bench mode for bring-up (docs/SETUP_AND_TEST.md).  The MQTT "set"
   command then also accepts, in IDLE only:
     bench_move  -2000..2000 mm      MOVE at 200 mm/s
     bench_turn  -3600..3600 0.1 deg TURN at 90 deg/s
     bench_speed 50..450 mm/s        MOVE 1000 mm at that speed (step response)
     bench_servo 0..180 deg          servo to that angle and range continuously
   0 for normal and demo builds. */
#ifndef APP_BENCH
#define APP_BENCH           0
#endif

#endif /* APP_CONFIG_H */
