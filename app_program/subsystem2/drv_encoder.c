/** @file drv_encoder.c
 *
 *  @brief Subsystem 2 -- PIO0 quadrature encoder driver.  See drv_encoder.h.
 *
 *  Each state machine loops forever: it samples the two pins, jumps through
 *  the 16-entry table on (old state, new state), adds +1 / -1 / 0 to Y and
 *  pushes Y to its RX FIFO (no-block, so the FIFO always holds the latest
 *  counts).  The reader drains the FIFO and keeps the last value.
 */

#include "drv_encoder.h"
#include "subsystem2.h"

#define PIO0_BASE_ADDR 0x50200000u
#define PIO_CTRL       (PIO0_BASE_ADDR + 0x000u)
#define PIO_FSTAT      (PIO0_BASE_ADDR + 0x004u)
#define PIO_FLEVEL     (PIO0_BASE_ADDR + 0x00Cu)

#define QUAD_WRAP_BOTTOM 15u
#define QUAD_WRAP_TOP    23u
#define QUAD_PROG_LEN    24u

/* ~ 16 us at 125 MHz; the SM pushes every ~10 cycles */
#define READ_SPIN_MAX 2000u

/** pico-examples pio/quadrature_encoder/quadrature_encoder.pio, assembled. */
static uint16_t const g_quad_prog[QUAD_PROG_LEN] = {
    0x000f, 0x000e, 0x0015, 0x000f, /*  0- 3: jump table, old state 00 */
    0x0015, 0x000f, 0x000f, 0x000e, /*  4- 7: old state 01             */
    0x000e, 0x000f, 0x000f, 0x0015, /*  8-11: old state 10             */
    0x000f, 0x0015,                 /* 12-13: old state 11 (part)      */
    0x008f,                         /* 14: decrement: jmp y--, 15      */
    0xa0c2,                         /* 15: update: mov isr, y (wrap target) */
    0x8000,                         /* 16: push noblock                */
    0x60c2,                         /* 17: out isr, 2                  */
    0x4002,                         /* 18: in pins, 2                  */
    0xa0e6,                         /* 19: mov osr, isr                */
    0xa0a6,                         /* 20: mov pc, isr                 */
    0xa04a,                         /* 21: increment: mov y, ~y        */
    0x0097,                         /* 22: jmp y--, 23                 */
    0xa04a                          /* 23: mov y, ~y (wrap)            */
};

static uint32_t g_last[2]; /* returned if no fresh value arrives */

static inline uint32_t pio_rxf(uint32_t sm_no);
static inline uint32_t pio_instr_mem(uint32_t slot);
static inline uint32_t pio_sm_clkdiv(uint32_t sm_no);
static inline uint32_t pio_sm_execctrl(uint32_t sm_no);
static inline uint32_t pio_sm_shiftctrl(uint32_t sm_no);
static inline uint32_t pio_sm_instr(uint32_t sm_no);
static inline uint32_t pio_sm_pinctrl(uint32_t sm_no);
static inline uint32_t pio_fstat_rxempty(uint32_t sm_no);
static inline uint32_t pio_ctrl_sm_enable(uint32_t sm_no);
static inline uint32_t pio_ctrl_sm_restart(uint32_t sm_no);
static inline uint32_t pio_ctrl_clk_restart(uint32_t sm_no);
static void            pin_setup(uint32_t pin);
static void            sm_setup(uint32_t sm_idx, uint32_t pin_a);

EXPORT ER drv_encoder_init (void)
{
    uint32_t idx;
    ER       err = E_OK;

    /* With WIFI=cyw43 the kernel releases PIO0 at boot; otherwise do it. */
    app_reset_release(APP_REGS_RESET_PIO0);

    if ((in_w(PIO_CTRL) & (pio_ctrl_sm_enable(0u) | pio_ctrl_sm_enable(1u)))
        != 0u)
    {
        err = E_OBJ; /* someone else is using SM0/SM1 */
    }
    else
    {
        for (idx = 0u; idx < QUAD_PROG_LEN; idx++)
        {
            out_w(pio_instr_mem(idx), g_quad_prog[idx]);
        }
        pin_setup(S2_PIN_ENC_L_A);
        pin_setup(S2_PIN_ENC_L_A + 1u);
        pin_setup(S2_PIN_ENC_R_A);
        pin_setup(S2_PIN_ENC_R_A + 1u);
        sm_setup(DRV_ENC_LEFT, S2_PIN_ENC_L_A);
        sm_setup(DRV_ENC_RIGHT, S2_PIN_ENC_R_A);
        set_w(PIO_CTRL, pio_ctrl_sm_enable(DRV_ENC_LEFT)
                            | pio_ctrl_sm_enable(DRV_ENC_RIGHT));
    }
    return err;
}

EXPORT int32_t drv_encoder_read (uint32_t which)
{
    uint32_t const sm_idx  = (DRV_ENC_RIGHT == which) ? 1u : 0u;
    uint32_t       n_words = (in_w(PIO_FLEVEL) >> ((sm_idx * 8u) + 4u)) & 0xFu;
    uint32_t       value   = g_last[sm_idx];
    uint32_t       spin;

    /* Drain what is queued (possibly stale), then wait for one fresh push. */
    while (n_words > 0u)
    {
        value = in_w(pio_rxf(sm_idx));
        n_words--;
    }
    for (spin = 0u; spin < READ_SPIN_MAX; spin++)
    {
        if (0u == (pio_fstat_rxempty(sm_idx) & in_w(PIO_FSTAT)))
        {
            value = in_w(pio_rxf(sm_idx));
            break;
        }
    }
    /* Cast (int32_t)value: the PIO counter's 32-bit pattern read as a signed
       count. */
    g_last[sm_idx] = value;
    return (int32_t)value;
}

/** Address of PIO0's receive FIFO of state machine sm_no. */
static inline uint32_t pio_rxf (uint32_t sm_no)
{
    return PIO0_BASE_ADDR + 0x020u + (4u * sm_no);
}

/** Address of PIO0 instruction memory slot. */
static inline uint32_t pio_instr_mem (uint32_t slot)
{
    return PIO0_BASE_ADDR + 0x048u + (4u * slot);
}

/** Address of the clock divider register of PIO0 state machine sm_no. */
static inline uint32_t pio_sm_clkdiv (uint32_t sm_no)
{
    return PIO0_BASE_ADDR + 0x0C8u + (0x18u * sm_no);
}

/** Address of the execution control register of PIO0 state machine sm_no. */
static inline uint32_t pio_sm_execctrl (uint32_t sm_no)
{
    return PIO0_BASE_ADDR + 0x0CCu + (0x18u * sm_no);
}

/** Address of the shift control register of PIO0 state machine sm_no. */
static inline uint32_t pio_sm_shiftctrl (uint32_t sm_no)
{
    return PIO0_BASE_ADDR + 0x0D0u + (0x18u * sm_no);
}

/** Address of the immediate-instruction register of PIO0 state machine sm_no.
 */
static inline uint32_t pio_sm_instr (uint32_t sm_no)
{
    return PIO0_BASE_ADDR + 0x0D8u + (0x18u * sm_no);
}

/** Address of the pin control register of PIO0 state machine sm_no. */
static inline uint32_t pio_sm_pinctrl (uint32_t sm_no)
{
    return PIO0_BASE_ADDR + 0x0DCu + (0x18u * sm_no);
}

/** FSTAT bit: the receive FIFO of state machine sm_no is empty. */
static inline uint32_t pio_fstat_rxempty (uint32_t sm_no)
{
    return 1uL << (8u + sm_no);
}

/** CTRL bit that enables state machine sm_no. */
static inline uint32_t pio_ctrl_sm_enable (uint32_t sm_no)
{
    return 1uL << sm_no;
}

/** CTRL bit that restarts state machine sm_no. */
static inline uint32_t pio_ctrl_sm_restart (uint32_t sm_no)
{
    return 1uL << (4u + sm_no);
}

/** CTRL bit that restarts the clock divider of state machine sm_no. */
static inline uint32_t pio_ctrl_clk_restart (uint32_t sm_no)
{
    return 1uL << (8u + sm_no);
}

/** Make pin a Schmitt-triggered input with pull-up, connected to PIO0. */
static void pin_setup (uint32_t pin)
{
    out_w(app_regs_pad(pin), APP_REGS_PAD_IE | APP_REGS_PAD_PUE
                                 | APP_REGS_PAD_SHEMITT); /* input, pull-up */
    out_w(app_regs_gpio_ctrl(pin), APP_REGS_FUNCSEL_PIO0);
}

/** Load and start the quadrature program on PIO0 state machine sm_idx for the
    encoder whose channel A is pin_a. */
static void sm_setup (uint32_t sm_idx, uint32_t pin_a)
{
    out_w(pio_sm_clkdiv(sm_idx), 1uL << 16); /* full speed */
    out_w(pio_sm_execctrl(sm_idx),
          (QUAD_WRAP_TOP << 12) | (QUAD_WRAP_BOTTOM << 7));
    /* RX FIFO joined (8 deep), OUT shifts right, IN shifts left, no autopush */
    out_w(pio_sm_shiftctrl(sm_idx), (1uL << 31) | (1uL << 19));
    out_w(pio_sm_pinctrl(sm_idx), pin_a << 15); /* IN_BASE = A, B = A + 1 */
    set_w(PIO_CTRL, pio_ctrl_sm_restart(sm_idx) | pio_ctrl_clk_restart(sm_idx));
    out_w(pio_sm_instr(sm_idx), 0x0000u); /* jmp 0 */
}

/*** end of file ***/
