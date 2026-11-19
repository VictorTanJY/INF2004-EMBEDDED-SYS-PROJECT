/*
 *  drv_adc.c
 *  Subsystem 3 -- ADC + DMA driver.  See drv_adc.h.
 *
 *  ADC: 48 MHz / (1 + 15999) = 3 kHz, round robin over AIN0..AIN2, so each
 *  channel is sampled at 1 kHz (NFR2).  Each DMA channel fills a 30-sample
 *  buffer (10 ms) and chains to the other; its completion interrupt
 *  re-arms the finished channel's write address (TRANS_COUNT reloads by
 *  itself on the next trigger) and wakes LineBarcodeTask.
 */

#include "drv_adc.h"

#define ADC_BASE_ADDR 0x4004C000u
#define ADC_CS        (ADC_BASE_ADDR + 0x00u)
#define ADC_FCS       (ADC_BASE_ADDR + 0x08u)
#define ADC_FIFO      (ADC_BASE_ADDR + 0x0Cu)
#define ADC_DIV       (ADC_BASE_ADDR + 0x10u)

#define ADC_CS_EN         (1uL << 0)
#define ADC_CS_START_MANY (1uL << 3)
#define ADC_CS_READY      (1uL << 8)
#define ADC_CS_ERR_STICKY (1uL << 10)
#define ADC_CS_RROBIN_012 (0x7uL << 16)
#define ADC_FCS_EN        (1uL << 0)
#define ADC_FCS_DREQ_EN   (1uL << 3)
#define ADC_FCS_OVER      (1uL << 11)
#define ADC_FCS_UNDER     (1uL << 10)
#define ADC_FCS_THRESH_1  (1uL << 24)
static inline uint32_t adc_fcs_level (uint32_t fcs)
{
    return (fcs >> 16) & 0xFu;
}

#define ADC_DIV_3KHZ (15999uL << 8)

#define DMA_BASE_ADDR 0x50000000u
static inline uint32_t dma_read_addr (uint32_t chan)
{
    return DMA_BASE_ADDR + (0x40u * chan) + 0x00u;
}

static inline uint32_t dma_write_addr (uint32_t chan)
{
    return DMA_BASE_ADDR + (0x40u * chan) + 0x04u;
}

static inline uint32_t dma_trans_count (uint32_t chan)
{
    return DMA_BASE_ADDR + (0x40u * chan) + 0x08u;
}

static inline uint32_t dma_ctrl_trig (uint32_t chan)
{
    return DMA_BASE_ADDR + (0x40u * chan) + 0x0Cu;
}

static inline uint32_t dma_al1_ctrl (uint32_t chan)
{
    return DMA_BASE_ADDR + (0x40u * chan) + 0x10u;
}

#define DMA_INTE1      (DMA_BASE_ADDR + 0x414u)
#define DMA_INTS1      (DMA_BASE_ADDR + 0x41Cu)
#define DMA_CHAN_ABORT (DMA_BASE_ADDR + 0x444u)

#define DMA_CTRL_EN         (1uL << 0)
#define DMA_CTRL_SIZE_HALF  (1uL << 2)
#define DMA_CTRL_INCR_WRITE (1uL << 5)
static inline uint32_t dma_ctrl_chain (uint32_t chan)
{
    return (uint32_t)chan << 11;
}

#define DMA_CTRL_TREQ_ADC (36uL << 15) /* DREQ_ADC */

#define DMA_CH_A      10u
#define DMA_CH_B      11u
#define DMA_IRQ1_NO   12u /* DMA_IRQ_1 */
#define DMA_IRQ_LEVEL 2

static uint16_t g_buf[2][DRV_ADC_BLOCK_LEN];
static uint8_t volatile g_ready; /* buffer last completed */
static uint32_t volatile g_seq;  /* blocks completed      */
static uint32_t volatile g_errors;
static ID g_task;

static uint32_t ctrl_for (uint32_t chain_to)
{
    return DMA_CTRL_EN | DMA_CTRL_SIZE_HALF | DMA_CTRL_INCR_WRITE
           | dma_ctrl_chain(chain_to) | DMA_CTRL_TREQ_ADC;
}

/* adc_dma_isr: one 10 ms block is complete. */
static void adc_dma_isr (UINT intno)
{
    uint32_t const ints =
        in_w(DMA_INTS1) & ((1uL << DMA_CH_A) | (1uL << DMA_CH_B));

    (void)intno;
    out_w(DMA_INTS1, ints); /* write 1 to clear */
    if ((ints & (1uL << DMA_CH_A)) != 0u)
    {
        out_w(dma_write_addr(DMA_CH_A),
              (uint32_t)&g_buf[0][0]); /* re-arm, no trigger */
        g_ready = 0u;
        g_seq++;
    }
    if ((ints & (1uL << DMA_CH_B)) != 0u)
    {
        out_w(dma_write_addr(DMA_CH_B), (uint32_t)&g_buf[1][0]);
        g_ready = 1u;
        g_seq++;
    }
    if ((in_w(ADC_FCS) & (ADC_FCS_OVER | ADC_FCS_UNDER)) != 0u)
    {
        out_w(ADC_FCS, in_w(ADC_FCS) | ADC_FCS_OVER | ADC_FCS_UNDER); /* W1C */
        g_errors++;
    }
    if (g_task > 0)
    {
        (void)tk_wup_tsk(g_task);
    }
}

EXPORT ER drv_adc_init (void)
{
    uint32_t const pins[3] = {26u, 27u, 28u};
    uint32_t       spin;
    uint32_t       idx;

    app_reset_release(APP_REGS_RESET_ADC | APP_REGS_RESET_DMA);

    /* Analog pins: digital input and pulls off (RP2040 datasheet 4.9.1). */
    for (idx = 0u; idx < 3u; idx++)
    {
        out_w(app_regs_pad(pins[idx]), APP_REGS_PAD_OD);
        out_w(app_regs_gpio_ctrl(pins[idx]), APP_REGS_FUNCSEL_NULL);
    }

    out_w(ADC_CS, ADC_CS_EN);
    for (spin = 0u; (spin < 100000u) && (0u == (in_w(ADC_CS) & ADC_CS_READY));
         spin++)
    {
        ;
    }
    if (0u == (in_w(ADC_CS) & ADC_CS_READY))
    {
        return E_IO; /* clk_adc not running? */
    }
    out_w(ADC_DIV, ADC_DIV_3KHZ);
    out_w(ADC_FCS, ADC_FCS_EN | ADC_FCS_DREQ_EN | ADC_FCS_THRESH_1);
    while (adc_fcs_level(in_w(ADC_FCS)) != 0u)
    {
        (void)in_w(ADC_FIFO); /* drain */
    }

    /* Two chained channels, B configured without triggering. */
    out_w(DMA_CHAN_ABORT, (1uL << DMA_CH_A) | (1uL << DMA_CH_B));
    out_w(dma_read_addr(DMA_CH_A), ADC_FIFO);
    out_w(dma_write_addr(DMA_CH_A), (uint32_t)&g_buf[0][0]);
    out_w(dma_trans_count(DMA_CH_A), DRV_ADC_BLOCK_LEN);
    out_w(dma_read_addr(DMA_CH_B), ADC_FIFO);
    out_w(dma_write_addr(DMA_CH_B), (uint32_t)&g_buf[1][0]);
    out_w(dma_trans_count(DMA_CH_B), DRV_ADC_BLOCK_LEN);
    out_w(dma_al1_ctrl(DMA_CH_B), ctrl_for(DMA_CH_A));
    return E_OK;
}

EXPORT ER drv_adc_start (ID task)
{
    T_DINT dint;
    ER     err;

    g_task      = task;
    dint.intatr = TA_HLNG;
    dint.inthdr = (FP)adc_dma_isr;
    err         = tk_def_int(DMA_IRQ1_NO, &dint);
    if (err != E_OK)
    {
        return err;
    }
    out_w(DMA_INTS1, (1uL << DMA_CH_A) | (1uL << DMA_CH_B));
    set_w(DMA_INTE1, (1uL << DMA_CH_A) | (1uL << DMA_CH_B));
    EnableInt(DMA_IRQ1_NO, DMA_IRQ_LEVEL); /* this (owning) core */

    out_w(dma_ctrl_trig(DMA_CH_A), ctrl_for(DMA_CH_B)); /* arm A */
    /* AINSEL = 0 and round robin 0, 1, 2: the buffer always starts at L. */
    out_w(ADC_CS, ADC_CS_EN | ADC_CS_RROBIN_012 | ADC_CS_START_MANY);
    return E_OK;
}

EXPORT bool drv_adc_get_block (uint16_t   p_out[DRV_ADC_BLOCK_LEN],
                               uint32_t * p_seq)
{
    UINT     intsts;
    uint32_t idx;
    uint8_t  which;

    DI(intsts);
    which  = g_ready;
    *p_seq = g_seq;
    EI(intsts);
    if (0u == *p_seq)
    {
        return false;
    }
    /* The other buffer is being filled now; this one is stable for 10 ms. */
    for (idx = 0u; idx < DRV_ADC_BLOCK_LEN; idx++)
    {
        p_out[idx] = (uint16_t)(g_buf[which][idx] & 0x0FFFu);
    }
    return true;
}

EXPORT uint32_t drv_adc_errors (void)
{
    return g_errors + (((in_w(ADC_CS) & ADC_CS_ERR_STICKY) != 0u) ? 1u : 0u);
}

/*** end of file ***/
