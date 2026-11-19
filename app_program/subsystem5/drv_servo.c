/*
 *  drv_servo.c
 *  Subsystem 5 -- servo PWM.  125 MHz / DIV 125 = 1 MHz count, TOP 19999
 *  -> 20 ms period; the compare value is the pulse width in microseconds.
 */

#include "drv_servo.h"
#include "mission_cfg.h"
#include "subsystem5.h"

#define SERVO_SLICE 7u

static inline uint32_t pwm_ch_csr(uint32_t slice);
static inline uint32_t pwm_ch_div(uint32_t slice);
static inline uint32_t pwm_ch_cc(uint32_t slice);
static inline uint32_t pwm_ch_top(uint32_t slice);

EXPORT ER drv_servo_init (void)
{
    app_reset_release(APP_REGS_RESET_PWM);
    out_w(pwm_ch_csr(SERVO_SLICE), 0u);
    out_w(pwm_ch_div(SERVO_SLICE), 125uL << 4);
    out_w(pwm_ch_top(SERVO_SLICE), 19999u);
    drv_servo_set_deg(OBST_SERVO_CENTRE_DEG);
    out_w(pwm_ch_csr(SERVO_SLICE), APP_REGS_PWM_CSR_EN);
    clr_w(app_regs_pad(S5_PIN_SERVO), APP_REGS_PAD_OD);
    out_w(app_regs_gpio_ctrl(S5_PIN_SERVO), APP_REGS_FUNCSEL_PWM);
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
    angle = 180 - angle;
#endif
    pulse_us = OBST_SERVO_MIN_US
               + ((angle * (OBST_SERVO_MAX_US - OBST_SERVO_MIN_US)) / 180);
    out_w(pwm_ch_cc(SERVO_SLICE),
          (uint32_t)pulse_us << 16); /* channel B; A (GP14) = 0 */
}

static inline uint32_t pwm_ch_csr (uint32_t slice)
{
    return APP_REGS_PWM_BASE + (0x14u * slice) + 0x00u;
}

static inline uint32_t pwm_ch_div (uint32_t slice)
{
    return APP_REGS_PWM_BASE + (0x14u * slice) + 0x04u;
}

static inline uint32_t pwm_ch_cc (uint32_t slice)
{
    return APP_REGS_PWM_BASE + (0x14u * slice) + 0x0Cu;
}

static inline uint32_t pwm_ch_top (uint32_t slice)
{
    return APP_REGS_PWM_BASE + (0x14u * slice) + 0x10u;
}

/*** end of file ***/
