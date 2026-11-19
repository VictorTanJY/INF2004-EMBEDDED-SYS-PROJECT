/** @file app_gpio_irq.c
 *
 *  @brief Shared IO_IRQ_BANK0 handler.  See app_gpio_irq.h.
 */

#include "app_gpio_irq.h"

#define IO_INT_REGS  4u
#define IO_EDGE_BITS (APP_GPIO_EDGE_FALL | APP_GPIO_EDGE_RISE)

static app_gpio_irq_cb_t g_cb[APP_REGS_GPIO_NUM];
static uint32_t volatile g_count;
static bool gb_defined;

static inline uint32_t io_intr(uint32_t gpio_bank);
static inline uint32_t io_proc0_inte(uint32_t gpio_bank);
static inline uint32_t io_proc0_ints(uint32_t gpio_bank);
static inline uint32_t io_proc1_inte(uint32_t gpio_bank);
static inline uint32_t io_proc1_ints(uint32_t gpio_bank);
static uint32_t        this_core(void);
static void            gpio_bank0_isr(UINT intno);

EXPORT ER app_gpio_irq_init (void)
{
    T_DINT dint;
    ER     err = E_OK;

    if (!gb_defined)
    {
        /* Cast (FP)gpio_bank0_isr: the kernel stores the entry point as FP; it
           is called with its own type. */
        dint.intatr = TA_HLNG;
        dint.inthdr = (FP)gpio_bank0_isr;
        err         = tk_def_int(APP_GPIO_IRQ_NO, &dint);
        gb_defined  = (E_OK == err);
    }
    return err;
}

EXPORT ER app_gpio_irq_attach (uint32_t pin, uint32_t edges,
                               app_gpio_irq_cb_t p_callback)
{
    uint32_t const reg   = pin / 8u;
    uint32_t const shift = (pin % 8u) * 4u;
    UINT           intsts;

    if ((pin >= APP_REGS_GPIO_NUM) || (NULL == p_callback)
        || ((edges & ~IO_EDGE_BITS) != 0u) || !gb_defined)
    {
        return E_PAR;
    }

    DI(intsts);
    g_cb[pin] = p_callback;
    out_w(io_intr(reg), IO_EDGE_BITS << shift); /* drop stale edges */
    if (0u == this_core())
    {
        set_w(io_proc0_inte(reg), edges << shift);
    }
    else
    {
        set_w(io_proc1_inte(reg), edges << shift);
    }
    EI(intsts);

    EnableInt(APP_GPIO_IRQ_NO, APP_GPIO_IRQ_LEVEL); /* this core's NVIC */
    return E_OK;
}

EXPORT void app_gpio_irq_detach (uint32_t pin)
{
    uint32_t const reg   = pin / 8u;
    uint32_t const shift = (pin % 8u) * 4u;
    UINT           intsts;

    if (pin >= APP_REGS_GPIO_NUM)
    {
        return;
    }
    DI(intsts);
    clr_w(io_proc0_inte(reg), 0xFuL << shift);
    clr_w(io_proc1_inte(reg), 0xFuL << shift);
    g_cb[pin] = NULL;
    EI(intsts);
}

EXPORT uint32_t app_gpio_irq_count (void)
{
    return g_count;
}

/** IO_BANK0 interrupt registers (RP2040 datasheet 2.19.6.1).  Each register
   covers 8 GPIOs, 4 bits per GPIO: LEVEL_LOW, LEVEL_HIGH, EDGE_LOW, EDGE_HIGH.
 */
static inline uint32_t io_intr (uint32_t gpio_bank)
{
    return APP_REGS_IO_BANK0_BASE + 0x0F0u + (4u * gpio_bank);
}

/** Address of core 0's interrupt-enable register for GPIO bank word gpio_bank.
 */
static inline uint32_t io_proc0_inte (uint32_t gpio_bank)
{
    return APP_REGS_IO_BANK0_BASE + 0x100u + (4u * gpio_bank);
}

/** Address of core 0's interrupt-status register for GPIO bank word gpio_bank.
 */
static inline uint32_t io_proc0_ints (uint32_t gpio_bank)
{
    return APP_REGS_IO_BANK0_BASE + 0x120u + (4u * gpio_bank);
}

/** Address of core 1's interrupt-enable register for GPIO bank word gpio_bank.
 */
static inline uint32_t io_proc1_inte (uint32_t gpio_bank)
{
    return APP_REGS_IO_BANK0_BASE + 0x130u + (4u * gpio_bank);
}

/** Address of core 1's interrupt-status register for GPIO bank word gpio_bank.
 */
static inline uint32_t io_proc1_ints (uint32_t gpio_bank)
{
    return APP_REGS_IO_BANK0_BASE + 0x150u + (4u * gpio_bank);
}

/** Number of the core running the caller: 0 or 1. */
static uint32_t this_core (void)
{
    return in_w(SIO_CPUID_ADDR); /* 0 = core 0, 1 = core 1 */
}

/** Bank-0 GPIO interrupt: timestamp it, acknowledge the edges and call the
    callbacks attached on this core. */
static void gpio_bank0_isr (UINT intno)
{
    uint32_t const t_us = app_time_us();
    uint32_t const core = this_core();
    uint32_t       reg;

    (void)intno;
    g_count++;
    for (reg = 0u; reg < IO_INT_REGS; reg++)
    {
        uint32_t ints =
            in_w((0u == core) ? io_proc0_ints(reg) : io_proc1_ints(reg));

        while (ints != 0u)
        {
            /* Cast (uint32_t)__builtin_ctz(ints): ints != 0 here, so 0..31. */
            uint32_t const bit    = (uint32_t)__builtin_ctz(ints);
            uint32_t const pin    = (reg * 8u) + (bit / 4u);
            uint32_t const shift  = (bit / 4u) * 4u;
            uint32_t const events = (ints >> shift) & 0xFu;

            /* Edge events are latched: write 1 to clear before the callback,
               so an edge during the callback is not lost. */
            out_w(io_intr(reg), (events & IO_EDGE_BITS) << shift);
            ints &= ~(0xFuL << shift);

            if ((pin < APP_REGS_GPIO_NUM) && (g_cb[pin] != NULL))
            {
                g_cb[pin](pin, events, t_us);
            }
        }
    }
}

/*** end of file ***/
