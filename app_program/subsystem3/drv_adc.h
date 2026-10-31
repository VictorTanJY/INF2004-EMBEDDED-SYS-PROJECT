/*
 *  drv_adc.h
 *  Subsystem 3 -- ADC0-ADC2 round-robin at 3 kHz (1 kHz per channel) into
 *  two 10 ms DMA buffers (ping-pong), adc_dma_isr on DMA_IRQ_1 (Week 6
 *  Section 3).  Register-level; the kernel ADC driver is off
 *  (DEVCNF_USE_ADC = 0).  Owner: Buddy 3.
 *
 *  DMA channels 10 and 11 are used.  The radio's SDK driver claims its two
 *  channels from the bottom (0, 1) with dma_claim_unused_channel(), so
 *  the top two never collide (TEAM.md section 3).
 */

#ifndef DRV_ADC_H
#define DRV_ADC_H

#include <stdint.h>
#include "../app.h"
#include "line_cfg.h"

#define DRV_ADC_BLOCK_LEN   (LINE_SAMPLES_PER_BLOCK * LINE_CH_COUNT)   /* 30 */

/* Release ADC / DMA from reset and configure them (not started). */
IMPORT ER drv_adc_init(void);

/* Register adc_dma_isr, enable it ON THE CALLING CORE (core 1), start the
   conversions.  task = the task the ISR wakes every 10 ms. */
IMPORT ER drv_adc_start(ID task);

/* Latest completed block (interleaved L, R, BC, L, R, BC, ...; 12-bit).
   *p_seq = block number; a gap means blocks were missed (NFR2).
   Copies under DI/EI so the ISR cannot switch buffers mid-copy.
   Returns false if no block has completed yet. */
IMPORT bool drv_adc_get_block(uint16_t p_out[DRV_ADC_BLOCK_LEN], uint32_t * p_seq);

/* ADC FIFO overflow / conversion errors seen (diagnostics). */
IMPORT uint32_t drv_adc_errors(void);

#endif /* DRV_ADC_H */
