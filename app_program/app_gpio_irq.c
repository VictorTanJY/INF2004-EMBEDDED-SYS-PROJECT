/*
 *  app_gpio_irq.c
 *  Shared IO_IRQ_BANK0 handler.  See app_gpio_irq.h.
 */

#include "app_gpio_irq.h"

/* IO_BANK0 interrupt registers (RP2040 datasheet 2.19.6.1).  Each register
   covers 8 GPIOs, 4 bits per GPIO: LEVEL_LOW, LEVEL_HIGH, EDGE_LOW, EDGE_HIGH. */
#define IO_INTR(n)          (IO_BANK0_BASE + 0x0F0u + (4u * (n)))
#define IO_PROC0_INTE(n)    (IO_BANK0_BASE + 0x100u + (4u * (n)))
#define IO_PROC0_INTS(n)    (IO_BANK0_BASE + 0x120u + (4u * (n)))
#define IO_PROC1_INTE(n)    (IO_BANK0_BASE + 0x130u + (4u * (n)))
#define IO_PROC1_INTS(n)    (IO_BANK0_BASE + 0x150u + (4u * (n)))
#define IO_INT_REGS         4u
#define IO_EDGE_BITS        (APP_GPIO_EDGE_FALL | APP_GPIO_EDGE_RISE)

static app_gpio_cb_t     g_cb[GPIO_NUM];
static volatile uint32_t g_count;
static bool              g_b_defined;

static uint32_t this_core(void)
{
    return in_w(SIO_CPUID_ADDR);        /* 0 = core 0, 1 = core 1 */
}

static void gpio_bank0_isr(UINT intno)
{
    uint32_t const t_us = (uint32_t)app_time_us();
    uint32_t const core = this_core();
    uint32_t       reg;

    (void)intno;
    g_count++;
    for (reg = 0u; reg < IO_INT_REGS; reg++)
    {
        uint32_t ints = in_w((core == 0u) ? IO_PROC0_INTS(reg) : IO_PROC1_INTS(reg));

        while (ints != 0u)
        {
            uint32_t const bit    = (uint32_t)__builtin_ctz(ints);
            uint32_t const pin    = (reg * 8u) + (bit / 4u);
            uint32_t const shift  = (bit / 4u) * 4u;
            uint32_t const events = (ints >> shift) & 0xFu;

            /* Edge events are latched: write 1 to clear before the callback,
               so an edge during the callback is not lost. */
            out_w(IO_INTR(reg), (events & IO_EDGE_BITS) << shift);
            ints &= ~(0xFuL << shift);

            if ((pin < GPIO_NUM) && (g_cb[pin] != NULL))
            {
                g_cb[pin](pin, events, t_us);
            }
        }
    }
}

EXPORT ER app_gpio_irq_init(void)
{
    T_DINT dint;
    ER     er = E_OK;

    if (!g_b_defined)
    {
        dint.intatr = TA_HLNG;
        dint.inthdr = (FP)gpio_bank0_isr;
        er = tk_def_int(APP_GPIO_IRQ_NO, &dint);
        g_b_defined = (er == E_OK);
    }
    return er;
}

EXPORT ER app_gpio_irq_attach(uint32_t pin, uint32_t edges, app_gpio_cb_t cb)
{
    uint32_t const reg   = pin / 8u;
    uint32_t const shift = (pin % 8u) * 4u;
    UINT           intsts;

    if ((pin >= GPIO_NUM) || (cb == NULL) || ((edges & ~IO_EDGE_BITS) != 0u) || !g_b_defined)
    {
        return E_PAR;
    }

    DI(intsts);
    g_cb[pin] = cb;
    out_w(IO_INTR(reg), IO_EDGE_BITS << shift);             /* drop stale edges */
    if (this_core() == 0u)
    {
        set_w(IO_PROC0_INTE(reg), edges << shift);
    }
    else
    {
        set_w(IO_PROC1_INTE(reg), edges << shift);
    }
    EI(intsts);

    EnableInt(APP_GPIO_IRQ_NO, APP_GPIO_IRQ_LEVEL);         /* this core's NVIC */
    return E_OK;
}

EXPORT void app_gpio_irq_detach(uint32_t pin)
{
    uint32_t const reg   = pin / 8u;
    uint32_t const shift = (pin % 8u) * 4u;
    UINT           intsts;

    if (pin >= GPIO_NUM)
    {
        return;
    }
    DI(intsts);
    clr_w(IO_PROC0_INTE(reg), 0xFuL << shift);
    clr_w(IO_PROC1_INTE(reg), 0xFuL << shift);
    g_cb[pin] = NULL;
    EI(intsts);
}

EXPORT uint32_t app_gpio_irq_count(void)
{
    return g_count;
}
