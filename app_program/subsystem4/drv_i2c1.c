/*
 *  drv_i2c1.c
 *  Subsystem 4 -- polled I2C1 master.  See drv_i2c1.h.
 *  Preserves the robot's 100 kHz timing. Recovery releases a stuck SDA
 *  with up to nine open-drain SCL pulses, followed by a STOP.
 */

#include "drv_i2c1.h"
#include "subsystem4.h"

#define I2C_RECOVERY_CLOCKS  9u /* SCL pulses that free a stuck slave */
#define I2C_RECOVERY_HALF_US 5u /* half period: 100 kHz */

#define I2C1_BASE_ADDR   0x40048000u
#define IC_CON           (I2C1_BASE_ADDR + 0x00u)
#define IC_TAR           (I2C1_BASE_ADDR + 0x04u)
#define IC_DATA_CMD      (I2C1_BASE_ADDR + 0x10u)
#define IC_FS_SCL_HCNT   (I2C1_BASE_ADDR + 0x1Cu)
#define IC_FS_SCL_LCNT   (I2C1_BASE_ADDR + 0x20u)
#define IC_RAW_INTR_STAT (I2C1_BASE_ADDR + 0x34u)
#define IC_RX_TL         (I2C1_BASE_ADDR + 0x38u)
#define IC_TX_TL         (I2C1_BASE_ADDR + 0x3Cu)
#define IC_CLR_INTR      (I2C1_BASE_ADDR + 0x40u)
#define IC_CLR_TX_ABRT   (I2C1_BASE_ADDR + 0x54u)
#define IC_ENABLE        (I2C1_BASE_ADDR + 0x6Cu)
#define IC_STATUS        (I2C1_BASE_ADDR + 0x70u)
#define IC_RXFLR         (I2C1_BASE_ADDR + 0x78u)
#define IC_SDA_HOLD      (I2C1_BASE_ADDR + 0x7Cu)
#define IC_ENABLE_STATUS (I2C1_BASE_ADDR + 0x9Cu)
#define IC_FS_SPKLEN     (I2C1_BASE_ADDR + 0xA0u)

#define CON_MASTER \
    ((1uL << 0) | (2uL << 1) | (1uL << 5) | (1uL << 6) | (1uL << 8))
/* master, fast mode, restart, slave off, TX_EMPTY_CTRL */
#define CMD_READ        (1uL << 8)
#define CMD_STOP        (1uL << 9)
#define CMD_RESTART     (1uL << 10)
#define INTR_TX_ABRT    (1uL << 6)
#define INTR_STOP_DET   (1uL << 9)
#define STATUS_ACTIVITY (1uL << 0)
#define STATUS_TFE      (1uL << 2)

#define I2C_TIMEOUT_US 2000u

static bool timed_out (uint32_t t_start)
{
    return ((uint32_t)(app_time_us() - t_start) > I2C_TIMEOUT_US);
}

static ER set_target (uint8_t addr7)
{
    uint32_t const t_start = (uint32_t)app_time_us();

    out_w(IC_ENABLE, 0u);
    while ((in_w(IC_ENABLE_STATUS) & 1u) != 0u)
    {
        if (timed_out(t_start))
        {
            return E_TMOUT;
        }
    }
    out_w(IC_TAR, addr7);
    (void)in_w(IC_CLR_INTR);
    out_w(IC_ENABLE, 1u);
    return E_OK;
}

/* Wait for the STOP of the transfer, then check for an abort (NACK). */
static ER finish (uint32_t t_start)
{
    while (0u == (in_w(IC_RAW_INTR_STAT) & (INTR_STOP_DET | INTR_TX_ABRT)))
    {
        if (timed_out(t_start))
        {
            return E_TMOUT;
        }
    }
    if ((in_w(IC_RAW_INTR_STAT) & INTR_TX_ABRT) != 0u)
    {
        (void)in_w(IC_CLR_TX_ABRT);
        (void)in_w(IC_CLR_INTR);
        return E_IO;
    }
    (void)in_w(IC_CLR_INTR);
    return E_OK;
}

/* Open-drain recovery: drive LOW or release; never drive either line HIGH.
 * NXP UM10204 section 3.1.16: up to nine clocks if SDA is stuck LOW.
 * All waits are bounded. A shorted/unpowered device remains an error.
 */
static void recovery_delay (void)
{
    uint32_t const t_start = (uint32_t)app_time_us();
    while ((uint32_t)((uint32_t)app_time_us() - t_start) < I2C_RECOVERY_HALF_US)
    {
    }
}

static ER scl_release (void)
{
    uint32_t const t_start = (uint32_t)app_time_us();
    out_w(APP_REGS_GPIO_OE_CLR, 1uL << S4_PIN_IMU_SCL);
    while (0u == (in_w(APP_REGS_GPIO_IN) & (1uL << S4_PIN_IMU_SCL)))
    {
        if ((uint32_t)((uint32_t)app_time_us() - t_start) >= 100u)
        {
            return E_TMOUT;
        }
    }
    recovery_delay();
    return E_OK;
}

static ER clear_bus (void)
{
    uint32_t const sda = 1uL << S4_PIN_IMU_SDA;
    uint32_t const scl = 1uL << S4_PIN_IMU_SCL;
    uint8_t        idx;
    ER             err;
    out_w(APP_REGS_GPIO_OE_CLR, sda | scl);
    out_w(APP_REGS_GPIO_OUT_CLR, sda | scl);
    out_w(app_regs_gpio_ctrl(S4_PIN_IMU_SDA), APP_REGS_FUNCSEL_SIO);
    out_w(app_regs_gpio_ctrl(S4_PIN_IMU_SCL), APP_REGS_FUNCSEL_SIO);
    out_w(app_regs_pad(S4_PIN_IMU_SDA), APP_REGS_PAD_IE | APP_REGS_PAD_DRIVE_4MA
                                            | APP_REGS_PAD_PUE
                                            | APP_REGS_PAD_SHEMITT);
    out_w(app_regs_pad(S4_PIN_IMU_SCL), APP_REGS_PAD_IE | APP_REGS_PAD_DRIVE_4MA
                                            | APP_REGS_PAD_PUE
                                            | APP_REGS_PAD_SHEMITT);
    err = scl_release();
    if ((E_OK == err) && (0u == (in_w(APP_REGS_GPIO_IN) & sda)))
    {
        for (idx = 0u; (idx < I2C_RECOVERY_CLOCKS)
                       && (0u == (in_w(APP_REGS_GPIO_IN) & sda));
             idx++)
        {
            out_w(APP_REGS_GPIO_OE_SET, scl);
            recovery_delay();
            err = scl_release();
            if (err != E_OK)
            {
                break;
            }
        }
        if (E_OK == err)
        {
            /* STOP: SDA rises while SCL is HIGH. */
            out_w(APP_REGS_GPIO_OE_SET, scl);
            out_w(APP_REGS_GPIO_OE_SET, sda);
            recovery_delay();
            err = scl_release();
            out_w(APP_REGS_GPIO_OE_CLR, sda);
            recovery_delay();
        }
    }
    out_w(APP_REGS_GPIO_OE_CLR, sda | scl);
    if ((E_OK == err)
        && ((in_w(APP_REGS_GPIO_IN) & (sda | scl)) != (sda | scl)))
    {
        err = E_IO;
    }
    return err;
}

EXPORT ER drv_i2c1_init (void)
{
    /* I2C1 is not released at boot and has one owner: a full reset pulse
       also recovers a wedged controller. */
    ER bus_er;
    set_w(RESETS_RESET, APP_REGS_RESET_I2C1);
    bus_er = clear_bus();
    app_reset_release(APP_REGS_RESET_I2C1);

    out_w(app_regs_gpio_ctrl(S4_PIN_IMU_SDA), APP_REGS_FUNCSEL_I2C);
    out_w(app_regs_pad(S4_PIN_IMU_SDA), APP_REGS_PAD_IE | APP_REGS_PAD_DRIVE_4MA
                                            | APP_REGS_PAD_PUE
                                            | APP_REGS_PAD_SHEMITT);
    out_w(app_regs_gpio_ctrl(S4_PIN_IMU_SCL), APP_REGS_FUNCSEL_I2C);
    out_w(app_regs_pad(S4_PIN_IMU_SCL), APP_REGS_PAD_IE | APP_REGS_PAD_DRIVE_4MA
                                            | APP_REGS_PAD_PUE
                                            | APP_REGS_PAD_SHEMITT);

    out_w(IC_ENABLE, 0u);
    out_w(IC_CON, CON_MASTER);
    out_w(IC_RX_TL, 0u);
    out_w(IC_TX_TL, 0u);
    out_w(IC_FS_SCL_HCNT, 500u);
    out_w(IC_FS_SCL_LCNT, 750u);
    out_w(IC_FS_SPKLEN, 11u);
    out_w(IC_SDA_HOLD, 38u);
    if (bus_er != E_OK)
    {
        return bus_er;
    }
    return (750u == in_w(IC_FS_SCL_LCNT)) ? E_OK : E_IO;
}

EXPORT ER drv_i2c1_write_reg (uint8_t addr7, uint8_t reg, uint8_t value)
{
    uint32_t t_start;
    ER       err = set_target(addr7);

    if (err != E_OK)
    {
        return err;
    }
    t_start = (uint32_t)app_time_us();
    out_w(IC_DATA_CMD, reg);
    out_w(IC_DATA_CMD, (uint32_t)value | CMD_STOP);
    return finish(t_start);
}

EXPORT ER drv_i2c1_read (uint8_t addr7, uint8_t reg, uint8_t * p_buf,
                         uint8_t n_bytes)
{
    uint32_t t_start;
    ER       err;
    uint8_t  idx;

    if ((0 == p_buf) || (addr7 > 0x7Fu) || (0u == n_bytes) || (n_bytes > 16u))
    {
        return E_PAR;
    }
    err = set_target(addr7);
    if (err != E_OK)
    {
        return err;
    }
    t_start = (uint32_t)app_time_us();
    out_w(IC_DATA_CMD, reg); /* register address */
    for (idx = 0u; idx < n_bytes; idx++)
    {
        uint32_t cmd = CMD_READ;

        if (0u == idx)
        {
            cmd |= CMD_RESTART;
        }
        if (idx == (n_bytes - 1u))
        {
            cmd |= CMD_STOP;
        }
        out_w(IC_DATA_CMD, cmd);
    }
    err = finish(t_start);
    if (err != E_OK)
    {
        return err;
    }
    for (idx = 0u; idx < n_bytes; idx++)
    {
        while (0u == in_w(IC_RXFLR))
        {
            if (timed_out(t_start))
            {
                return E_TMOUT;
            }
        }
        p_buf[idx] = (uint8_t)in_w(IC_DATA_CMD);
    }
    return E_OK;
}

EXPORT ER drv_i2c1_probe (uint8_t addr7)
{
    uint8_t byte;

    return drv_i2c1_read(addr7, 0x00u, &byte, 1u);
}

/*** end of file ***/
