/** @file drv_motor.c
 *
 *  @brief Subsystem 2 -- motor PWM driver.  See drv_motor.h.
 *
 *  PWM: 125 MHz clk_sys, DIV 1.0, TOP 6249 -> 20 kHz (NFR1, Robo Pico
 *  driver maximum).  The PWM block is held in reset after boot
 *  (USE_PTMR = 0), so it is released here; the servo (S5, slice 7) also
 *  releases it, and app_reset_release() never re-asserts the reset.
 */

#include "drv_motor.h"
#include "motion_cfg.h"
#include "subsystem2.h"

#define MOTOR_SLICE_COUNT 2u /* SLICE_L, SLICE_R */

#define MOTOR_PIN_COUNT 4u /* GP8..GP11 */

#define PWM_DIV_INT_1 (1u << 4)

#define SLICE_L 4u /* GP8 = 4A, GP9 = 4B   */
#define SLICE_R 5u /* GP10 = 5A, GP11 = 5B */

#define MOTOR_PIN_MASK                                 \
    ((1uL << S2_PIN_MOT_L_A) | (1uL << S2_PIN_MOT_L_B) \
     | (1uL << S2_PIN_MOT_R_A) | (1uL << S2_PIN_MOT_R_B))

static inline uint32_t pwm_ch_csr(uint32_t slice);
static inline uint32_t pwm_ch_div(uint32_t slice);
static inline uint32_t pwm_ch_ctr(uint32_t slice);
static inline uint32_t pwm_ch_cc(uint32_t slice);
static inline uint32_t pwm_ch_top(uint32_t slice);
static uint32_t        cc_for(int32_t duty, int32_t sign);

EXPORT ER drv_motor_init (void)
{
    uint32_t const pins[MOTOR_PIN_COUNT] = {S2_PIN_MOT_L_A, S2_PIN_MOT_L_B,
                                            S2_PIN_MOT_R_A, S2_PIN_MOT_R_B};
    uint32_t const slices[2]             = {SLICE_L, SLICE_R};
    UINT           intsts;
    uint32_t       idx;

    /* 1. Brake first: GP8-GP11 SIO outputs, LOW (BOOT order, Section 5). */
    DI(intsts);
    out_w(APP_REGS_GPIO_OUT_CLR, MOTOR_PIN_MASK);
    out_w(APP_REGS_GPIO_OE_SET, MOTOR_PIN_MASK);

    for (idx = 0u; idx < MOTOR_PIN_COUNT; idx++)
    {
        clr_w(app_regs_pad(pins[idx]), APP_REGS_PAD_OD);
        out_w(app_regs_gpio_ctrl(pins[idx]), APP_REGS_FUNCSEL_SIO);
    }

    EI(intsts);

    /* 2. PWM block out of reset, slices 4 and 5 at 20 kHz, duty 0. */
    app_reset_release(APP_REGS_RESET_PWM);

    for (idx = 0u; idx < MOTOR_SLICE_COUNT; idx++)
    {
        out_w(pwm_ch_csr(slices[idx]), 0u);
        out_w(pwm_ch_div(slices[idx]), PWM_DIV_INT_1);
        out_w(pwm_ch_top(slices[idx]), MOTION_PWM_TOP);
        out_w(pwm_ch_cc(slices[idx]), 0u);
        out_w(pwm_ch_ctr(slices[idx]), 0u);
        out_w(pwm_ch_csr(slices[idx]), APP_REGS_PWM_CSR_EN);
    }

    /* Read back: the block really left reset and took the settings. */
    return (MOTION_PWM_TOP == in_w(pwm_ch_top(SLICE_L))) ? E_OK : E_IO;
}

EXPORT void drv_motor_connect (void)
{
    out_w(app_regs_gpio_ctrl(S2_PIN_MOT_L_A), APP_REGS_FUNCSEL_PWM);
    out_w(app_regs_gpio_ctrl(S2_PIN_MOT_L_B), APP_REGS_FUNCSEL_PWM);
    out_w(app_regs_gpio_ctrl(S2_PIN_MOT_R_A), APP_REGS_FUNCSEL_PWM);
    out_w(app_regs_gpio_ctrl(S2_PIN_MOT_R_B), APP_REGS_FUNCSEL_PWM);
}

EXPORT void drv_motor_write (int16_t duty_l, int16_t duty_r)
{
    out_w(pwm_ch_cc(SLICE_L), cc_for(duty_l, MOTION_MOT_SIGN_L));
    out_w(pwm_ch_cc(SLICE_R), cc_for(duty_r, MOTION_MOT_SIGN_R));
}


/** Address of the control register of PWM slice. */
static inline uint32_t pwm_ch_csr (uint32_t slice)
{
    return APP_REGS_PWM_BASE + (0x14u * slice) + 0x00u;
}

/** Address of the clock divider register of PWM slice. */
static inline uint32_t pwm_ch_div (uint32_t slice)
{
    return APP_REGS_PWM_BASE + (0x14u * slice) + 0x04u;
}

/** Address of the counter register of PWM slice. */
static inline uint32_t pwm_ch_ctr (uint32_t slice)
{
    return APP_REGS_PWM_BASE + (0x14u * slice) + 0x08u;
}

/** Address of the compare register (channels A and B) of PWM slice. */
static inline uint32_t pwm_ch_cc (uint32_t slice)
{
    return APP_REGS_PWM_BASE + (0x14u * slice) + 0x0Cu;
}

/** Address of the wrap (TOP) register of PWM slice. */
static inline uint32_t pwm_ch_top (uint32_t slice)
{
    return APP_REGS_PWM_BASE + (0x14u * slice) + 0x10u;
}

/** Compare value of one motor input for a signed duty (per mille); sign
    selects the input that drives forward.  0 for the input that stays low. */
static uint32_t cc_for (int32_t duty, int32_t sign)
{
    int32_t  drive = duty * sign;
    uint32_t mag;

    if (drive > MOTION_DUTY_MAX)
    {
        drive = MOTION_DUTY_MAX;
    }

    if (drive < -MOTION_DUTY_MAX)
    {
        drive = -MOTION_DUTY_MAX;
    }

    /* Casts (uint32_t)((drive < 0) ? -drive : ...): drive is limited to
       +-MOTION_DUTY_MAX above; (uint32_t)MOTION_DUTY_MAX: a constant that
       fits. */
    mag = (uint32_t)((drive < 0) ? -drive : drive);
    mag = (mag * (MOTION_PWM_TOP + 1u)) / (uint32_t)MOTION_DUTY_MAX;

    /* channel A in bits 15:0, channel B in bits 31:16 */
    return (drive >= 0) ? mag : (mag << 16);
}

/*** end of file ***/
