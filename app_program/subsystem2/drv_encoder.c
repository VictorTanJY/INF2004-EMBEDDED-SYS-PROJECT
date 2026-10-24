/*
 *  drv_encoder.c
 *  Subsystem 2 -- PIO0 quadrature encoder driver.  See drv_encoder.h.
 *
 *  Each state machine loops forever: it samples the two pins, jumps through
 *  the 16-entry table on (old state, new state), adds +1 / -1 / 0 to Y and
 *  pushes Y to its RX FIFO (no-block, so the FIFO always holds the latest
 *  counts).  The reader drains the FIFO and keeps the last value.
 */

#include "drv_encoder.h"
#include "subsystem2.h"

#define PIO0_BASE_ADDR          0x50200000u
#define PIO_CTRL                (PIO0_BASE_ADDR + 0x000u)
#define PIO_FSTAT               (PIO0_BASE_ADDR + 0x004u)
#define PIO_FLEVEL              (PIO0_BASE_ADDR + 0x00Cu)
#define PIO_RXF(sm)             (PIO0_BASE_ADDR + 0x020u + (4u * (sm)))
#define PIO_INSTR_MEM(i)        (PIO0_BASE_ADDR + 0x048u + (4u * (i)))
#define PIO_SM_CLKDIV(sm)       (PIO0_BASE_ADDR + 0x0C8u + (0x18u * (sm)))
#define PIO_SM_EXECCTRL(sm)     (PIO0_BASE_ADDR + 0x0CCu + (0x18u * (sm)))
#define PIO_SM_SHIFTCTRL(sm)    (PIO0_BASE_ADDR + 0x0D0u + (0x18u * (sm)))
#define PIO_SM_INSTR(sm)        (PIO0_BASE_ADDR + 0x0D8u + (0x18u * (sm)))
#define PIO_SM_PINCTRL(sm)      (PIO0_BASE_ADDR + 0x0DCu + (0x18u * (sm)))

#define PIO_FSTAT_RXEMPTY(sm)   (1uL << (8u + (sm)))
#define PIO_CTRL_SM_ENABLE(sm)  (1uL << (sm))
#define PIO_CTRL_SM_RESTART(sm) (1uL << (4u + (sm)))
#define PIO_CTRL_CLK_RESTART(sm) (1uL << (8u + (sm)))

#define QUAD_WRAP_BOTTOM        15u
#define QUAD_WRAP_TOP           23u
#define QUAD_PROG_LEN           24u
#define READ_SPIN_MAX           2000u   /* ~ 16 us at 125 MHz; the SM pushes every ~10 cycles */

/* pico-examples pio/quadrature_encoder/quadrature_encoder.pio, assembled. */
static uint16_t const k_quad_prog[QUAD_PROG_LEN] =
{
    0x000f, 0x000e, 0x0015, 0x000f,     /*  0- 3: jump table, old state 00 */
    0x0015, 0x000f, 0x000f, 0x000e,     /*  4- 7: old state 01             */
    0x000e, 0x000f, 0x000f, 0x0015,     /*  8-11: old state 10             */
    0x000f, 0x0015,                     /* 12-13: old state 11 (part)      */
    0x008f,                             /* 14: decrement: jmp y--, 15      */
    0xa0c2,                             /* 15: update: mov isr, y (wrap target) */
    0x8000,                             /* 16: push noblock                */
    0x60c2,                             /* 17: out isr, 2                  */
    0x4002,                             /* 18: in pins, 2                  */
    0xa0e6,                             /* 19: mov osr, isr                */
    0xa0a6,                             /* 20: mov pc, isr                 */
    0xa04a,                             /* 21: increment: mov y, ~y        */
    0x0097,                             /* 22: jmp y--, 23                 */
    0xa04a                              /* 23: mov y, ~y (wrap)            */
};

static uint32_t g_last[2];              /* returned if no fresh value arrives */

static void pin_setup(uint32_t pin)
{
    out_w(GPIO(pin), GPIO_IE | GPIO_PUE | GPIO_SHEMITT);   /* input, pull-up */
    out_w(GPIO_CTRL(pin), GPIO_CTRL_FUNCSEL_PIO0);
}

static void sm_setup(uint32_t sm, uint32_t pin_a)
{
    out_w(PIO_SM_CLKDIV(sm), 1uL << 16);                    /* full speed */
    out_w(PIO_SM_EXECCTRL(sm), (QUAD_WRAP_TOP << 12) | (QUAD_WRAP_BOTTOM << 7));
    /* RX FIFO joined (8 deep), OUT shifts right, IN shifts left, no autopush */
    out_w(PIO_SM_SHIFTCTRL(sm), (1uL << 31) | (1uL << 19));
    out_w(PIO_SM_PINCTRL(sm), pin_a << 15);                 /* IN_BASE = A, B = A + 1 */
    set_w(PIO_CTRL, PIO_CTRL_SM_RESTART(sm) | PIO_CTRL_CLK_RESTART(sm));
    out_w(PIO_SM_INSTR(sm), 0x0000u);                       /* jmp 0 */
}

EXPORT ER drv_encoder_init(void)
{
    uint32_t i;

    /* With WIFI=cyw43 the kernel releases PIO0 at boot; otherwise do it. */
    app_reset_release(RESETS_RESET_PIO0);

    if ((in_w(PIO_CTRL) & (PIO_CTRL_SM_ENABLE(0u) | PIO_CTRL_SM_ENABLE(1u))) != 0u)
    {
        return E_OBJ;                   /* someone else is using SM0/SM1 */
    }
    for (i = 0u; i < QUAD_PROG_LEN; i++)
    {
        out_w(PIO_INSTR_MEM(i), k_quad_prog[i]);
    }
    pin_setup(S2_PIN_ENC_L_A);
    pin_setup(S2_PIN_ENC_L_A + 1u);
    pin_setup(S2_PIN_ENC_R_A);
    pin_setup(S2_PIN_ENC_R_A + 1u);
    sm_setup(DRV_ENC_LEFT, S2_PIN_ENC_L_A);
    sm_setup(DRV_ENC_RIGHT, S2_PIN_ENC_R_A);
    set_w(PIO_CTRL, PIO_CTRL_SM_ENABLE(DRV_ENC_LEFT) | PIO_CTRL_SM_ENABLE(DRV_ENC_RIGHT));
    return E_OK;
}

EXPORT int32_t drv_encoder_read(uint32_t which)
{
    uint32_t const sm    = (which == DRV_ENC_RIGHT) ? 1u : 0u;
    uint32_t       n     = (in_w(PIO_FLEVEL) >> ((sm * 8u) + 4u)) & 0xFu;
    uint32_t       value = g_last[sm];
    uint32_t       spin;

    /* Drain what is queued (possibly stale), then wait for one fresh push. */
    while (n > 0u)
    {
        value = in_w(PIO_RXF(sm));
        n--;
    }
    for (spin = 0u; spin < READ_SPIN_MAX; spin++)
    {
        if ((in_w(PIO_FSTAT) & PIO_FSTAT_RXEMPTY(sm)) == 0u)
        {
            value = in_w(PIO_RXF(sm));
            break;
        }
    }
    g_last[sm] = value;
    return (int32_t)value;
}
