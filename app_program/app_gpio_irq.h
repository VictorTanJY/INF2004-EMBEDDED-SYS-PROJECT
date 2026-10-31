/*
 *  app_gpio_irq.h
 *  One shared IO_IRQ_BANK0 handler for GPIO edge interrupts (Week 6 Section 3).
 *  Changed only by team agreement.
 *
 *  GP1 (US_ECHO, subsystem 5) and GP19 (IMU_INT1, subsystem 4) both raise
 *  IO_IRQ_BANK0, and the kernel keeps ONE handler per interrupt number for
 *  both cores.  So this module owns that interrupt and calls each pin's
 *  callback.  The Pico SDK irq_* API is not used: the port replaces it with
 *  shims (with CONSOLE=usb_cdc irq_add_shared_handler() has a single slot
 *  that the USB console uses).
 *
 *  Use:
 *    app_gpio_irq_init()    once, from usermain() before any task starts
 *                           (registers the handler with tk_def_int).
 *    app_gpio_irq_attach()  from the task that owns the pin, ON THE CORE
 *                           THAT MUST TAKE THE INTERRUPT: GPIO interrupt
 *                           enables and the NVIC are per core on the RP2040.
 *                           SafetyTask and ObstacleTask call it on core 1.
 *
 *  Callbacks run in interrupt context: keep them to a few microseconds and
 *  only call non-blocking kernel services (tk_set_flg, tk_wup_tsk).
 */

#ifndef APP_GPIO_IRQ_H
#define APP_GPIO_IRQ_H

#include <stdint.h>
#include <stdbool.h>
#include "app.h"

#define APP_GPIO_EDGE_FALL      0x4u    /* IO_BANK0 INTR EDGE_LOW  */
#define APP_GPIO_EDGE_RISE      0x8u    /* IO_BANK0 INTR EDGE_HIGH */

#define APP_GPIO_IRQ_NO         13u     /* IO_IRQ_BANK0             */
#define APP_GPIO_IRQ_LEVEL      1       /* INTPRI_MAX_EXTINT_PRI: echo timestamps */

/* events = APP_GPIO_EDGE_* bits that fired; t_us = TIMERAWL at handler entry */
typedef void (*app_gpio_cb_t)(uint32_t pin, uint32_t events, uint32_t t_us);

IMPORT ER   app_gpio_irq_init(void);
IMPORT ER   app_gpio_irq_attach(uint32_t pin, uint32_t edges, app_gpio_cb_t cb);
IMPORT void app_gpio_irq_detach(uint32_t pin);

/* Number of bank-0 interrupts taken since boot (diagnostics). */
IMPORT uint32_t app_gpio_irq_count(void);

#endif /* APP_GPIO_IRQ_H */
