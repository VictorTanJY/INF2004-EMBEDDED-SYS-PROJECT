/*
 *  drv_servo.c
 *  Subsystem 5 -- servo PWM.  125 MHz / DIV 125 = 1 MHz count, TOP 19999
 *  -> 20 ms period; the compare value is the pulse width in microseconds.
 */

#include "drv_servo.h"
#include "mission_cfg.h"
#include "subsystem5.h"

#define SERVO_SLICE         7u
#define PWM_CH_CSR(s)       (PWM_BASE + (0x14u * (s)) + 0x00u)
#define PWM_CH_DIV(s)       (PWM_BASE + (0x14u * (s)) + 0x04u)
#define PWM_CH_CC(s)        (PWM_BASE + (0x14u * (s)) + 0x0Cu)
#define PWM_CH_TOP(s)       (PWM_BASE + (0x14u * (s)) + 0x10u)

EXPORT ER drv_servo_init(void)
{
    app_reset_release(RESETS_RESET_PWM);
    out_w(PWM_CH_CSR(SERVO_SLICE), 0u);
    out_w(PWM_CH_DIV(SERVO_SLICE), 125uL << 4);
    out_w(PWM_CH_TOP(SERVO_SLICE), 19999u);
    drv_servo_set_deg(OBST_SERVO_CENTRE_DEG);
    out_w(PWM_CH_CSR(SERVO_SLICE), PWM_CH_CSR_EN);
    clr_w(GPIO(S5_PIN_SERVO), GPIO_OD);
    out_w(GPIO_CTRL(S5_PIN_SERVO), GPIO_CTRL_FUNCSEL_PWM);
    return (in_w(PWM_CH_TOP(SERVO_SLICE)) == 19999u) ? E_OK : E_IO;
}

EXPORT void drv_servo_set_deg(int32_t angle_deg)
{
    int32_t a = angle_deg;
    int32_t us;

    if (a < 0)
    {
        a = 0;
    }
    if (a > 180)
    {
        a = 180;
    }
#if !OBST_SERVO_LEFT_IS_HIGH
    a = 180 - a;
#endif
    us = OBST_SERVO_MIN_US + ((a * (OBST_SERVO_MAX_US - OBST_SERVO_MIN_US)) / 180);
    out_w(PWM_CH_CC(SERVO_SLICE), (uint32_t)us << 16);      /* channel B; A (GP14) = 0 */
}
