/*
 *  drv_motor.c
 *  Subsystem 2 -- motor PWM driver.  See drv_motor.h.
 *
 *  PWM: 125 MHz clk_sys, DIV 1.0, TOP 6249 -> 20 kHz (NFR1, Robo Pico
 *  driver maximum).  The PWM block is held in reset after boot
 *  (USE_PTMR = 0), so it is released here; the servo (S5, slice 7) also
 *  releases it, and app_reset_release() never re-asserts the reset.
 */

#include "drv_motor.h"
#include "motion_cfg.h"
#include "subsystem2.h"

#define PWM_CH_CSR(s)       (PWM_BASE + (0x14u * (s)) + 0x00u)
#define PWM_CH_DIV(s)       (PWM_BASE + (0x14u * (s)) + 0x04u)
#define PWM_CH_CTR(s)       (PWM_BASE + (0x14u * (s)) + 0x08u)
#define PWM_CH_CC(s)        (PWM_BASE + (0x14u * (s)) + 0x0Cu)
#define PWM_CH_TOP(s)       (PWM_BASE + (0x14u * (s)) + 0x10u)
#define PWM_DIV_INT_1       (1u << 4)

#define SLICE_L             4u              /* GP8 = 4A, GP9 = 4B   */
#define SLICE_R             5u              /* GP10 = 5A, GP11 = 5B */

#define MOTOR_PIN_MASK      ((1uL << S2_PIN_MOT_L_A) | (1uL << S2_PIN_MOT_L_B) | \
                             (1uL << S2_PIN_MOT_R_A) | (1uL << S2_PIN_MOT_R_B))

static uint32_t cc_for(int32_t duty, int32_t sign)
{
    int32_t  d  = duty * sign;
    uint32_t mag;

    if (d > MOTION_DUTY_MAX)
    {
        d = MOTION_DUTY_MAX;
    }
    if (d < -MOTION_DUTY_MAX)
    {
        d = -MOTION_DUTY_MAX;
    }
    mag = (uint32_t)((d < 0) ? -d : d);
    mag = (mag * (MOTION_PWM_TOP + 1u)) / (uint32_t)MOTION_DUTY_MAX;

    /* channel A in bits 15:0, channel B in bits 31:16 */
    return (d >= 0) ? mag : (mag << 16);
}

EXPORT ER drv_motor_init(void)
{
    uint32_t const pins[4] = { S2_PIN_MOT_L_A, S2_PIN_MOT_L_B, S2_PIN_MOT_R_A, S2_PIN_MOT_R_B };
    uint32_t const slices[2] = { SLICE_L, SLICE_R };
    UINT           intsts;
    uint32_t       i;

    /* 1. Brake first: GP8-GP11 SIO outputs, LOW (BOOT order, Section 5). */
    DI(intsts);
    out_w(GPIO_OUT_CLR, MOTOR_PIN_MASK);
    out_w(GPIO_OE_SET, MOTOR_PIN_MASK);
    for (i = 0u; i < 4u; i++)
    {
        clr_w(GPIO(pins[i]), GPIO_OD);
        out_w(GPIO_CTRL(pins[i]), GPIO_CTRL_FUNCSEL_SIO);
    }
    EI(intsts);

    /* 2. PWM block out of reset, slices 4 and 5 at 20 kHz, duty 0. */
    app_reset_release(RESETS_RESET_PWM);
    for (i = 0u; i < 2u; i++)
    {
        out_w(PWM_CH_CSR(slices[i]), 0u);
        out_w(PWM_CH_DIV(slices[i]), PWM_DIV_INT_1);
        out_w(PWM_CH_TOP(slices[i]), MOTION_PWM_TOP);
        out_w(PWM_CH_CC(slices[i]), 0u);
        out_w(PWM_CH_CTR(slices[i]), 0u);
        out_w(PWM_CH_CSR(slices[i]), PWM_CH_CSR_EN);
    }

    /* Read back: the block really left reset and took the settings. */
    return (in_w(PWM_CH_TOP(SLICE_L)) == MOTION_PWM_TOP) ? E_OK : E_IO;
}

EXPORT void drv_motor_connect(void)
{
    out_w(GPIO_CTRL(S2_PIN_MOT_L_A), GPIO_CTRL_FUNCSEL_PWM);
    out_w(GPIO_CTRL(S2_PIN_MOT_L_B), GPIO_CTRL_FUNCSEL_PWM);
    out_w(GPIO_CTRL(S2_PIN_MOT_R_A), GPIO_CTRL_FUNCSEL_PWM);
    out_w(GPIO_CTRL(S2_PIN_MOT_R_B), GPIO_CTRL_FUNCSEL_PWM);
}

EXPORT void drv_motor_write(int16_t duty_l, int16_t duty_r)
{
    out_w(PWM_CH_CC(SLICE_L), cc_for(duty_l, MOTION_MOT_SIGN_L));
    out_w(PWM_CH_CC(SLICE_R), cc_for(duty_r, MOTION_MOT_SIGN_R));
}
