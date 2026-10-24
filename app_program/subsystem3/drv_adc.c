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

#define ADC_BASE_ADDR       0x4004C000u
#define ADC_CS              (ADC_BASE_ADDR + 0x00u)
#define ADC_FCS             (ADC_BASE_ADDR + 0x08u)
#define ADC_FIFO            (ADC_BASE_ADDR + 0x0Cu)
#define ADC_DIV             (ADC_BASE_ADDR + 0x10u)

#define ADC_CS_EN           (1uL << 0)
#define ADC_CS_START_MANY   (1uL << 3)
#define ADC_CS_READY        (1uL << 8)
#define ADC_CS_ERR_STICKY   (1uL << 10)
#define ADC_CS_RROBIN_012   (0x7uL << 16)
#define ADC_FCS_EN          (1uL << 0)
#define ADC_FCS_DREQ_EN     (1uL << 3)
#define ADC_FCS_OVER        (1uL << 11)
#define ADC_FCS_UNDER       (1uL << 10)
#define ADC_FCS_THRESH_1    (1uL << 24)
#define ADC_FCS_LEVEL(f)    (((f) >> 16) & 0xFu)
#define ADC_DIV_3KHZ        (15999uL << 8)

#define DMA_BASE_ADDR       0x50000000u
#define DMA_READ_ADDR(c)    (DMA_BASE_ADDR + (0x40u * (c)) + 0x00u)
#define DMA_WRITE_ADDR(c)   (DMA_BASE_ADDR + (0x40u * (c)) + 0x04u)
#define DMA_TRANS_COUNT(c)  (DMA_BASE_ADDR + (0x40u * (c)) + 0x08u)
#define DMA_CTRL_TRIG(c)    (DMA_BASE_ADDR + (0x40u * (c)) + 0x0Cu)
#define DMA_AL1_CTRL(c)     (DMA_BASE_ADDR + (0x40u * (c)) + 0x10u)
#define DMA_INTE1           (DMA_BASE_ADDR + 0x414u)
#define DMA_INTS1           (DMA_BASE_ADDR + 0x41Cu)
#define DMA_CHAN_ABORT      (DMA_BASE_ADDR + 0x444u)

#define DMA_CTRL_EN         (1uL << 0)
#define DMA_CTRL_SIZE_HALF  (1uL << 2)
#define DMA_CTRL_INCR_WRITE (1uL << 5)
#define DMA_CTRL_CHAIN(c)   ((uint32_t)(c) << 11)
#define DMA_CTRL_TREQ_ADC   (36uL << 15)            /* DREQ_ADC */

#define DMA_CH_A            10u
#define DMA_CH_B            11u
#define DMA_IRQ1_NO         12u                     /* DMA_IRQ_1 */
#define DMA_IRQ_LEVEL       2

static uint16_t          g_buf[2][DRV_ADC_BLOCK_LEN];
static volatile uint8_t  g_ready;                   /* buffer last completed */
static volatile uint32_t g_seq;                     /* blocks completed      */
static volatile uint32_t g_errors;
static ID                g_task;

static uint32_t ctrl_for(uint32_t chain_to)
{
    return DMA_CTRL_EN | DMA_CTRL_SIZE_HALF | DMA_CTRL_INCR_WRITE |
           DMA_CTRL_CHAIN(chain_to) | DMA_CTRL_TREQ_ADC;
}

/* adc_dma_isr: one 10 ms block is complete. */
static void adc_dma_isr(UINT intno)
{
    uint32_t const ints = in_w(DMA_INTS1) & ((1uL << DMA_CH_A) | (1uL << DMA_CH_B));

    (void)intno;
    out_w(DMA_INTS1, ints);                         /* write 1 to clear */
    if ((ints & (1uL << DMA_CH_A)) != 0u)
    {
        out_w(DMA_WRITE_ADDR(DMA_CH_A), (uint32_t)&g_buf[0][0]);   /* re-arm, no trigger */
        g_ready = 0u;
        g_seq++;
    }
    if ((ints & (1uL << DMA_CH_B)) != 0u)
    {
        out_w(DMA_WRITE_ADDR(DMA_CH_B), (uint32_t)&g_buf[1][0]);
        g_ready = 1u;
        g_seq++;
    }
    if ((in_w(ADC_FCS) & (ADC_FCS_OVER | ADC_FCS_UNDER)) != 0u)
    {
        out_w(ADC_FCS, in_w(ADC_FCS) | ADC_FCS_OVER | ADC_FCS_UNDER);  /* W1C */
        g_errors++;
    }
    if (g_task > 0)
    {
        (void)tk_wup_tsk(g_task);
    }
}

EXPORT ER drv_adc_init(void)
{
    uint32_t const pins[3] = { 26u, 27u, 28u };
    uint32_t       spin;
    uint32_t       i;

    app_reset_release(RESETS_RESET_ADC | RESETS_RESET_DMA);

    /* Analog pins: digital input and pulls off (RP2040 datasheet 4.9.1). */
    for (i = 0u; i < 3u; i++)
    {
        out_w(GPIO(pins[i]), GPIO_OD);
        out_w(GPIO_CTRL(pins[i]), GPIO_CTRL_FUNCSEL_NULL);
    }

    out_w(ADC_CS, ADC_CS_EN);
    for (spin = 0u; (spin < 100000u) && ((in_w(ADC_CS) & ADC_CS_READY) == 0u); spin++)
    {
        ;
    }
    if ((in_w(ADC_CS) & ADC_CS_READY) == 0u)
    {
        return E_IO;                                /* clk_adc not running? */
    }
    out_w(ADC_DIV, ADC_DIV_3KHZ);
    out_w(ADC_FCS, ADC_FCS_EN | ADC_FCS_DREQ_EN | ADC_FCS_THRESH_1);
    while (ADC_FCS_LEVEL(in_w(ADC_FCS)) != 0u)
    {
        (void)in_w(ADC_FIFO);                       /* drain */
    }

    /* Two chained channels, B configured without triggering. */
    out_w(DMA_CHAN_ABORT, (1uL << DMA_CH_A) | (1uL << DMA_CH_B));
    out_w(DMA_READ_ADDR(DMA_CH_A), ADC_FIFO);
    out_w(DMA_WRITE_ADDR(DMA_CH_A), (uint32_t)&g_buf[0][0]);
    out_w(DMA_TRANS_COUNT(DMA_CH_A), DRV_ADC_BLOCK_LEN);
    out_w(DMA_READ_ADDR(DMA_CH_B), ADC_FIFO);
    out_w(DMA_WRITE_ADDR(DMA_CH_B), (uint32_t)&g_buf[1][0]);
    out_w(DMA_TRANS_COUNT(DMA_CH_B), DRV_ADC_BLOCK_LEN);
    out_w(DMA_AL1_CTRL(DMA_CH_B), ctrl_for(DMA_CH_A));
    return E_OK;
}

EXPORT ER drv_adc_start(ID task)
{
    T_DINT dint;
    ER     er;

    g_task      = task;
    dint.intatr = TA_HLNG;
    dint.inthdr = (FP)adc_dma_isr;
    er = tk_def_int(DMA_IRQ1_NO, &dint);
    if (er != E_OK)
    {
        return er;
    }
    out_w(DMA_INTS1, (1uL << DMA_CH_A) | (1uL << DMA_CH_B));
    set_w(DMA_INTE1, (1uL << DMA_CH_A) | (1uL << DMA_CH_B));
    EnableInt(DMA_IRQ1_NO, DMA_IRQ_LEVEL);          /* this (owning) core */

    out_w(DMA_CTRL_TRIG(DMA_CH_A), ctrl_for(DMA_CH_B));           /* arm A */
    /* AINSEL = 0 and round robin 0, 1, 2: the buffer always starts at L. */
    out_w(ADC_CS, ADC_CS_EN | ADC_CS_RROBIN_012 | ADC_CS_START_MANY);
    return E_OK;
}

EXPORT bool drv_adc_get_block(uint16_t p_out[DRV_ADC_BLOCK_LEN], uint32_t * p_seq)
{
    UINT     intsts;
    uint32_t i;
    uint8_t  which;

    DI(intsts);
    which  = g_ready;
    *p_seq = g_seq;
    EI(intsts);
    if (*p_seq == 0u)
    {
        return false;
    }
    /* The other buffer is being filled now; this one is stable for 10 ms. */
    for (i = 0u; i < DRV_ADC_BLOCK_LEN; i++)
    {
        p_out[i] = (uint16_t)(g_buf[which][i] & 0x0FFFu);
    }
    return true;
}

EXPORT uint32_t drv_adc_errors(void)
{
    return g_errors + (((in_w(ADC_CS) & ADC_CS_ERR_STICKY) != 0u) ? 1u : 0u);
}
