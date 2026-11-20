/** @file drv_ultrasonic.c
 *
 *  @brief Subsystem 5 -- HC-SR04P driver.  See drv_ultrasonic.h.
 */

#include "drv_ultrasonic.h"
#include "subsystem5.h"
#include "../app_gpio_irq.h"

#define US_TRIG_PULSE_US 12u /* trigger pulse >= 10 us (datasheet) */

/* CONSOLE=uart and CRASH_REPORT=1 builds put UART0 on GP0/GP1 (Week 6
   2.2, A3): the HC-SR04 is unplugged then, and these pins must not be
   touched, or the console dies.  Every call below becomes a no-op. */
#if TM_CONSOLE_UART
#define US_PINS_FREE 0
#else
#define US_PINS_FREE 1
#endif

/* Echo result handed from echo_isr to ObstacleTask (Change Requirement 1):
   ONE word, pulse width in 16 bits (us, saturating: 65535 us is far beyond
   OBST_ECHO_MAX_US, so it reads as "no echo", never as a short range) plus
   a done bit.  The ISR writes the whole word with one store; the task
   reads and clears it with interrupts masked.  Both run on core 1, so the
   ISR cannot run in between. */
#define ECHO_WIDTH_BITS 16u
#define ECHO_DONE       (1uL << ECHO_WIDTH_BITS)

static uint32_t volatile g_echo;
static uint32_t volatile g_rise_us; /* ISR only, between the two edges */
static bool volatile gb_rise;

#if US_PINS_FREE
static void echo_isr(uint32_t pin, uint32_t events, uint32_t t_us);
#endif

EXPORT ER drv_ultrasonic_init (void)
{
    ER err = E_NOSPT;

#if US_PINS_FREE
    out_w(APP_REGS_GPIO_OUT_CLR, 1uL << S5_PIN_US_TRIG);
    out_w(APP_REGS_GPIO_OE_SET, 1uL << S5_PIN_US_TRIG);
    clr_w(app_regs_pad(S5_PIN_US_TRIG), APP_REGS_PAD_OD);
    out_w(app_regs_gpio_ctrl(S5_PIN_US_TRIG), APP_REGS_FUNCSEL_SIO);

    out_w(APP_REGS_GPIO_OE_CLR, 1uL << S5_PIN_US_ECHO);
    out_w(app_regs_pad(S5_PIN_US_ECHO),
          APP_REGS_PAD_IE | APP_REGS_PAD_PDE
              | APP_REGS_PAD_SHEMITT); /* pull-down: unplugged = low */
    out_w(app_regs_gpio_ctrl(S5_PIN_US_ECHO), APP_REGS_FUNCSEL_SIO);
    err = E_OK;
#endif
    return err;
}

EXPORT ER drv_ultrasonic_attach (void)
{
    ER err = E_NOSPT;

#if US_PINS_FREE
    err = app_gpio_irq_attach(
        S5_PIN_US_ECHO, APP_GPIO_EDGE_RISE | APP_GPIO_EDGE_FALL, echo_isr);
#endif
    return err;
}

EXPORT void drv_ultrasonic_trigger (void)
{
#if US_PINS_FREE
    uint32_t t_start;

    gb_rise = false;
    g_echo  = 0u;
    out_w(APP_REGS_GPIO_OUT_SET, 1uL << S5_PIN_US_TRIG);
    t_start = app_time_us();
    while ((app_time_us() - t_start) < US_TRIG_PULSE_US)
    {
        ; /* >= 10 us (datasheet) */
    }
    out_w(APP_REGS_GPIO_OUT_CLR, 1uL << S5_PIN_US_TRIG);
#endif /* else no pins: nothing to trigger */
}

EXPORT bool drv_ultrasonic_take (uint32_t * p_width_us)
{
    UINT     intsts;
    uint32_t echo;

    DI(intsts);
    echo   = g_echo;
    g_echo = 0u;
    EI(intsts);
    *p_width_us = echo & ((1uL << ECHO_WIDTH_BITS) - 1uL);
    return ((echo & ECHO_DONE) != 0u);
}

#if US_PINS_FREE

/** echo_isr: bank-0 callback on core 1. */
static void echo_isr (uint32_t pin, uint32_t events, uint32_t t_us)
{
    (void)pin;
    if ((events & APP_GPIO_EDGE_RISE) != 0u)
    {
        g_rise_us = t_us;
        gb_rise   = true;
    }
    if (((events & APP_GPIO_EDGE_FALL) != 0u) && gb_rise)
    {
        uint32_t width = t_us - g_rise_us;

        if (width > ((1uL << ECHO_WIDTH_BITS) - 1uL))
        {
            width = (1uL << ECHO_WIDTH_BITS) - 1uL;
        }
        g_echo  = ECHO_DONE | width;
        gb_rise = false;
        (void)tk_set_flg(subsystem5_flg_obst(), OBST_FLG_ECHO);
    }
}

#endif

/*** end of file ***/
