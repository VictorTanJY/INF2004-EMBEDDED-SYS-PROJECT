/** @file app_regs.h
 *
 *  @brief RP2040 register addresses and bit values used by the drivers, as
 *  unsigned constants.
 *
 *  The template's sysdef.h writes many of these as signed int constants
 *  (e.g. GPIO_IE is (1<<6)), so using them with the unsigned register values
 *  would mix signed and unsigned data (Barr-C 5.3b, 5.3c).  These copies have
 *  the same values (RP2040 datasheet sections 2.14, 2.19, 4.5); the checks
 *  at the end stop the build if one ever differs from sysdef.h.
 *  Changed only by team agreement.
 */

#ifndef APP_REGS_H
#define APP_REGS_H

#include <stdint.h>
#include <tk/tkernel.h>

/* ---- Base addresses --------------------------------------------------- */
#define APP_REGS_IO_BANK0_BASE   0x40014000u /* GPIO control (CTRL)     */
#define APP_REGS_PADS_BANK0_BASE 0x4001C000u /* pad control             */
#define APP_REGS_PWM_BASE        0x40050000u
#define APP_REGS_GPIO_NUM        30u /* user GPIO pins GP0..GP29 */

/* ---- SIO GPIO registers (single-cycle IO block) ----------------------- */
#define APP_REGS_SIO_BASE     0xD0000000u
#define APP_REGS_GPIO_IN      (APP_REGS_SIO_BASE + 0x004u)
#define APP_REGS_GPIO_OUT_SET (APP_REGS_SIO_BASE + 0x014u)
#define APP_REGS_GPIO_OUT_CLR (APP_REGS_SIO_BASE + 0x018u)
#define APP_REGS_GPIO_OUT_XOR (APP_REGS_SIO_BASE + 0x01Cu)
#define APP_REGS_GPIO_OE_SET  (APP_REGS_SIO_BASE + 0x024u)
#define APP_REGS_GPIO_OE_CLR  (APP_REGS_SIO_BASE + 0x028u)

/* ---- GPIOx_CTRL.FUNCSEL ----------------------------------------------- */
#define APP_REGS_FUNCSEL_I2C  3u
#define APP_REGS_FUNCSEL_PWM  4u
#define APP_REGS_FUNCSEL_SIO  5u
#define APP_REGS_FUNCSEL_PIO0 6u
#define APP_REGS_FUNCSEL_NULL 31u /* pin disconnected (ADC inputs) */

/* ---- Pad control bits ------------------------------------------------- */
#define APP_REGS_PAD_OD        (1u << 7) /* output disable */
#define APP_REGS_PAD_IE        (1u << 6) /* input enable   */
#define APP_REGS_PAD_DRIVE_4MA (1u << 4)
#define APP_REGS_PAD_PUE       (1u << 3) /* pull-up        */
#define APP_REGS_PAD_PDE       (1u << 2) /* pull-down      */
#define APP_REGS_PAD_SHEMITT   (1u << 1) /* Schmitt trigger */

/* ---- PWM and RESETS bits ---------------------------------------------- */
#define APP_REGS_PWM_CSR_EN (1u << 0)
#define APP_REGS_RESET_ADC  0x00000001u
#define APP_REGS_RESET_DMA  0x00000004u
#define APP_REGS_RESET_I2C1 0x00000010u
#define APP_REGS_RESET_PIO0 0x00000400u
#define APP_REGS_RESET_PWM  0x00004000u

/*!
 * @brief Address of a pin's pad control register (PADS_BANK0 GPIOx).
 * @param[in] pin GPIO number, 0 to APP_REGS_GPIO_NUM - 1.
 * @return The register address.
 */
static inline uint32_t app_regs_pad (uint32_t pin)
{
    return (APP_REGS_PADS_BANK0_BASE + 0x4u) + (pin * 4u);
}

/*!
 * @brief Address of a pin's control register (IO_BANK0 GPIOx_CTRL).
 * @param[in] pin GPIO number, 0 to APP_REGS_GPIO_NUM - 1.
 * @return The register address.
 */
static inline uint32_t app_regs_gpio_ctrl (uint32_t pin)
{
    return (APP_REGS_IO_BANK0_BASE + 0x4u) + (pin * 8u);
}

/* Build stops here if a value above differs from the template's sysdef.h.
   NOTE: each sysdef.h constant is non-negative and below 2^32, so the
   (uint32_t) casts below keep its value. */
typedef uint8_t app_regs_check_t
    [(((uint32_t)IO_BANK0_BASE == APP_REGS_IO_BANK0_BASE)
      && ((uint32_t)GPIO_IN == APP_REGS_GPIO_IN)
      && ((uint32_t)GPIO_OUT_SET == APP_REGS_GPIO_OUT_SET)
      && ((uint32_t)GPIO_OUT_CLR == APP_REGS_GPIO_OUT_CLR)
      && ((uint32_t)GPIO_OUT_XOR == APP_REGS_GPIO_OUT_XOR)
      && ((uint32_t)GPIO_OE_SET == APP_REGS_GPIO_OE_SET)
      && ((uint32_t)GPIO_OE_CLR == APP_REGS_GPIO_OE_CLR)
      && ((uint32_t)PADS_BANK0_BASE == APP_REGS_PADS_BANK0_BASE)
      && ((uint32_t)PWM_BASE == APP_REGS_PWM_BASE)
      && ((uint32_t)GPIO_NUM == APP_REGS_GPIO_NUM)
      && ((uint32_t)GPIO_CTRL_FUNCSEL_I2C == APP_REGS_FUNCSEL_I2C)
      && ((uint32_t)GPIO_CTRL_FUNCSEL_PWM == APP_REGS_FUNCSEL_PWM)
      && ((uint32_t)GPIO_CTRL_FUNCSEL_SIO == APP_REGS_FUNCSEL_SIO)
      && ((uint32_t)GPIO_CTRL_FUNCSEL_PIO0 == APP_REGS_FUNCSEL_PIO0)
      && ((uint32_t)GPIO_CTRL_FUNCSEL_NULL == APP_REGS_FUNCSEL_NULL)
      && ((uint32_t)GPIO_OD == APP_REGS_PAD_OD)
      && ((uint32_t)GPIO_IE == APP_REGS_PAD_IE)
      && ((uint32_t)GPIO_DRIVE_4MA == APP_REGS_PAD_DRIVE_4MA)
      && ((uint32_t)GPIO_PUE == APP_REGS_PAD_PUE)
      && ((uint32_t)GPIO_PDE == APP_REGS_PAD_PDE)
      && ((uint32_t)GPIO_SHEMITT == APP_REGS_PAD_SHEMITT)
      && ((uint32_t)PWM_CH_CSR_EN == APP_REGS_PWM_CSR_EN)
      && ((uint32_t)RESETS_RESET_ADC == APP_REGS_RESET_ADC)
      && ((uint32_t)RESETS_RESET_DMA == APP_REGS_RESET_DMA)
      && ((uint32_t)RESETS_RESET_I2C1 == APP_REGS_RESET_I2C1)
      && ((uint32_t)RESETS_RESET_PIO0 == APP_REGS_RESET_PIO0)
      && ((uint32_t)RESETS_RESET_PWM == APP_REGS_RESET_PWM))
         ? 1
         : -1];

#endif /* APP_REGS_H */

/*** end of file ***/
