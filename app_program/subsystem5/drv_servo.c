/*
 *  drv_servo.c
 *  Subsystem 5 -- servo PWM.  125 MHz / DIV 125 = 1 MHz count, TOP 19999
 *  -> 20 ms period; the compare value is the pulse width in microseconds.
 */

#include "drv_servo.h"
#include "mission_cfg.h"
#include "subsystem5.h"

#define SERVO_SLICE 7u
static inline uint32_t pwm_ch_csr (uint32_t slice)
{
    return PWM_BASE + (0x14u * slice) + 0x00u;
}

static inline uint32_t pwm_ch_div (uint32_t slice)
{
    return PWM_BASE + (0x14u * slice) + 0x04u;
}

static inline uint32_t pwm_ch_cc (uint32_t slice)
{
    return PWM_BASE + (0x14u * slice) + 0x0Cu;
}

static inline uint32_t pwm_ch_top (uint32_t slice)
{
    return PWM_BASE + (0x14u * slice) + 0x10u;
}

EXPORT ER drv_servo_init (void)
{
    app_reset_release(RESETS_RESET_PWM);
    out_w(pwm_ch_csr(SERVO_SLICE), 0u);
    out_w(pwm_ch_div(SERVO_SLICE), 125uL << 4);
    out_w(pwm_ch_top(SERVO_SLICE), 19999u);
    drv_servo_set_deg(OBST_SERVO_CENTRE_DEG);
    out_w(pwm_ch_csr(SERVO_SLICE), PWM_CH_CSR_EN);
    clr_w(GPIO(S5_PIN_SERVO), GPIO_OD);
    out_w(GPIO_CTRL(S5_PIN_SERVO), GPIO_CTRL_FUNCSEL_PWM);
    return (19999u == in_w(pwm_ch_top(SERVO_SLICE))) ? E_OK : E_IO;
}

EXPORT void drv_servo_set_deg (int32_t angle_deg)
{
    int32_t angle = angle_deg;
    int32_t pulse_us;

    if (angle < 0)
    {
        angle = 0;
    }
    if (angle > 180)
    {
        angle = 180;
    }
#if !OBST_SERVO_LEFT_IS_HIGH
    a = 180 - a;
#endif
    pulse_us = OBST_SERVO_MIN_US
               + ((angle * (OBST_SERVO_MAX_US - OBST_SERVO_MIN_US)) / 180);
    out_w(pwm_ch_cc(SERVO_SLICE),
          (uint32_t)pulse_us << 16); /* channel B; A (GP14) = 0 */
}

/*** end of file ***/
