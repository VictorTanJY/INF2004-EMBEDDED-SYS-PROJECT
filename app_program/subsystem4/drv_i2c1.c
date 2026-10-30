/*
 *  drv_i2c1.c
 *  Subsystem 4 -- polled I2C1 master.  See drv_i2c1.h.
 *  Preserves the robot's 100 kHz timing. Recovery releases a stuck SDA
 *  with up to nine open-drain SCL pulses, followed by a STOP.
 */

#include "drv_i2c1.h"
#include "subsystem4.h"

#define I2C1_BASE_ADDR      0x40048000u
#define IC_CON              (I2C1_BASE_ADDR + 0x00u)
#define IC_TAR              (I2C1_BASE_ADDR + 0x04u)
#define IC_DATA_CMD         (I2C1_BASE_ADDR + 0x10u)
#define IC_FS_SCL_HCNT      (I2C1_BASE_ADDR + 0x1Cu)
#define IC_FS_SCL_LCNT      (I2C1_BASE_ADDR + 0x20u)
#define IC_RAW_INTR_STAT    (I2C1_BASE_ADDR + 0x34u)
#define IC_RX_TL            (I2C1_BASE_ADDR + 0x38u)
#define IC_TX_TL            (I2C1_BASE_ADDR + 0x3Cu)
#define IC_CLR_INTR         (I2C1_BASE_ADDR + 0x40u)
#define IC_CLR_TX_ABRT      (I2C1_BASE_ADDR + 0x54u)
#define IC_ENABLE           (I2C1_BASE_ADDR + 0x6Cu)
#define IC_STATUS           (I2C1_BASE_ADDR + 0x70u)
#define IC_RXFLR            (I2C1_BASE_ADDR + 0x78u)
#define IC_SDA_HOLD         (I2C1_BASE_ADDR + 0x7Cu)
#define IC_ENABLE_STATUS    (I2C1_BASE_ADDR + 0x9Cu)
#define IC_FS_SPKLEN        (I2C1_BASE_ADDR + 0xA0u)

#define CON_MASTER          ((1uL << 0) | (2uL << 1) | (1uL << 5) | (1uL << 6) | (1uL << 8))
                            /* master, fast mode, restart, slave off, TX_EMPTY_CTRL */
#define CMD_READ            (1uL << 8)
#define CMD_STOP            (1uL << 9)
#define CMD_RESTART         (1uL << 10)
#define INTR_TX_ABRT        (1uL << 6)
#define INTR_STOP_DET       (1uL << 9)
#define STATUS_ACTIVITY     (1uL << 0)
#define STATUS_TFE          (1uL << 2)

#define I2C_TIMEOUT_US      2000u

static bool timed_out(uint32_t t0)
{
    return ((uint32_t)(app_time_us() - t0) > I2C_TIMEOUT_US);
}

static ER set_target(uint8_t addr7)
{
    uint32_t const t0 = (uint32_t)app_time_us();

    out_w(IC_ENABLE, 0u);
    while ((in_w(IC_ENABLE_STATUS) & 1u) != 0u)
    {
        if (timed_out(t0))
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
static ER finish(uint32_t t0)
{
    while ((in_w(IC_RAW_INTR_STAT) & (INTR_STOP_DET | INTR_TX_ABRT)) == 0u)
    {
        if (timed_out(t0))
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
static void recovery_delay(void)
{
    uint32_t const t0 = (uint32_t)app_time_us();
    while ((uint32_t)((uint32_t)app_time_us() - t0) < 5u) { }
}

static ER scl_release(void)
{
    uint32_t const t0 = (uint32_t)app_time_us();
    out_w(GPIO_OE_CLR, 1uL << S4_PIN_IMU_SCL);
    while ((in_w(GPIO_IN) & (1uL << S4_PIN_IMU_SCL)) == 0u)
    {
        if ((uint32_t)((uint32_t)app_time_us() - t0) >= 100u)
            return E_TMOUT;
    }
    recovery_delay();
    return E_OK;
}

static ER clear_bus(void)
{
    uint32_t const sda = 1uL << S4_PIN_IMU_SDA;
    uint32_t const scl = 1uL << S4_PIN_IMU_SCL;
    uint8_t i;
    ER er;
    out_w(GPIO_OE_CLR, sda | scl);
    out_w(GPIO_OUT_CLR, sda | scl);
    out_w(GPIO_CTRL(S4_PIN_IMU_SDA), GPIO_CTRL_FUNCSEL_SIO);
    out_w(GPIO_CTRL(S4_PIN_IMU_SCL), GPIO_CTRL_FUNCSEL_SIO);
    out_w(GPIO(S4_PIN_IMU_SDA), GPIO_IE | GPIO_DRIVE_4MA | GPIO_PUE | GPIO_SHEMITT);
    out_w(GPIO(S4_PIN_IMU_SCL), GPIO_IE | GPIO_DRIVE_4MA | GPIO_PUE | GPIO_SHEMITT);
    er = scl_release();
    if (er == E_OK && ((in_w(GPIO_IN) & sda) == 0u))
    {
        for (i = 0u; i < 9u && ((in_w(GPIO_IN) & sda) == 0u); i++)
        {
            out_w(GPIO_OE_SET, scl);
            recovery_delay();
            er = scl_release();
            if (er != E_OK) break;
        }
        if (er == E_OK)
        {
            /* STOP: SDA rises while SCL is HIGH. */
            out_w(GPIO_OE_SET, scl);
            out_w(GPIO_OE_SET, sda);
            recovery_delay();
            er = scl_release();
            out_w(GPIO_OE_CLR, sda);
            recovery_delay();
        }
    }
    out_w(GPIO_OE_CLR, sda | scl);
    if (er == E_OK && ((in_w(GPIO_IN) & (sda | scl)) != (sda | scl)))
        er = E_IO;
    return er;
}

EXPORT ER drv_i2c1_init(void)
{
    /* I2C1 is not released at boot and has one owner: a full reset pulse
       also recovers a wedged controller. */
    ER bus_er;
    set_w(RESETS_RESET, RESETS_RESET_I2C1);
    bus_er = clear_bus();
    app_reset_release(RESETS_RESET_I2C1);

    out_w(GPIO_CTRL(S4_PIN_IMU_SDA), GPIO_CTRL_FUNCSEL_I2C);
    out_w(GPIO(S4_PIN_IMU_SDA), GPIO_IE | GPIO_DRIVE_4MA | GPIO_PUE | GPIO_SHEMITT);
    out_w(GPIO_CTRL(S4_PIN_IMU_SCL), GPIO_CTRL_FUNCSEL_I2C);
    out_w(GPIO(S4_PIN_IMU_SCL), GPIO_IE | GPIO_DRIVE_4MA | GPIO_PUE | GPIO_SHEMITT);

    out_w(IC_ENABLE, 0u);
    out_w(IC_CON, CON_MASTER);
    out_w(IC_RX_TL, 0u);
    out_w(IC_TX_TL, 0u);
    out_w(IC_FS_SCL_HCNT, 500u);
    out_w(IC_FS_SCL_LCNT, 750u);
    out_w(IC_FS_SPKLEN, 11u);
    out_w(IC_SDA_HOLD, 38u);
    if (bus_er != E_OK) return bus_er;
    return (in_w(IC_FS_SCL_LCNT) == 750u) ? E_OK : E_IO;
}

EXPORT ER drv_i2c1_write_reg(uint8_t addr7, uint8_t reg, uint8_t value)
{
    uint32_t t0;
    ER             er = set_target(addr7);

    if (er != E_OK)
    {
        return er;
    }
    t0 = (uint32_t)app_time_us();
    out_w(IC_DATA_CMD, reg);
    out_w(IC_DATA_CMD, (uint32_t)value | CMD_STOP);
    return finish(t0);
}

EXPORT ER drv_i2c1_read(uint8_t addr7, uint8_t reg, uint8_t * p_buf, uint8_t n)
{
    uint32_t t0;
    ER             er;
    uint8_t        i;

    if ((p_buf == 0) || (addr7 > 0x7Fu) || (n == 0u) || (n > 16u))
    {
        return E_PAR;
    }
    er = set_target(addr7);
    if (er != E_OK)
    {
        return er;
    }
    t0 = (uint32_t)app_time_us();
    out_w(IC_DATA_CMD, reg);                            /* register address */
    for (i = 0u; i < n; i++)
    {
        uint32_t cmd = CMD_READ;

        if (i == 0u)
        {
            cmd |= CMD_RESTART;
        }
        if (i == (n - 1u))
        {
            cmd |= CMD_STOP;
        }
        out_w(IC_DATA_CMD, cmd);
    }
    er = finish(t0);
    if (er != E_OK)
    {
        return er;
    }
    for (i = 0u; i < n; i++)
    {
        while (in_w(IC_RXFLR) == 0u)
        {
            if (timed_out(t0))
            {
                return E_TMOUT;
            }
        }
        p_buf[i] = (uint8_t)in_w(IC_DATA_CMD);
    }
    return E_OK;
}

EXPORT ER drv_i2c1_probe(uint8_t addr7)
{
    uint8_t b;

    return drv_i2c1_read(addr7, 0x00u, &b, 1u);
}
