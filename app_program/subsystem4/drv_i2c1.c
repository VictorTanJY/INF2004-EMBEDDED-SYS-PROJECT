/*
 *  drv_i2c1.c
 *  Subsystem 4 -- polled I2C1 master.  See drv_i2c1.h.
 *  Timing values as in the template's kernel driver
 *  (device/i2c/sysdepend/rp2040/i2c_rp2040.c): 125 MHz / (126 + 187) ~ 400 kHz.
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
    out_w(IC_ENABLE, 1u);
    (void)in_w(IC_CLR_INTR);
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

EXPORT ER drv_i2c1_init(void)
{
    /* I2C1 is not released at boot and has one owner: a full reset pulse
       also recovers a wedged controller. */
    set_w(RESETS_RESET, RESETS_RESET_I2C1);
    app_reset_release(RESETS_RESET_I2C1);

    out_w(GPIO_CTRL(S4_PIN_IMU_SDA), GPIO_CTRL_FUNCSEL_I2C);
    out_w(GPIO(S4_PIN_IMU_SDA), GPIO_IE | GPIO_DRIVE_4MA | GPIO_PUE | GPIO_SHEMITT);
    out_w(GPIO_CTRL(S4_PIN_IMU_SCL), GPIO_CTRL_FUNCSEL_I2C);
    out_w(GPIO(S4_PIN_IMU_SCL), GPIO_IE | GPIO_DRIVE_4MA | GPIO_PUE | GPIO_SHEMITT);

    out_w(IC_ENABLE, 0u);
    out_w(IC_CON, CON_MASTER);
    out_w(IC_RX_TL, 0u);
    out_w(IC_TX_TL, 0u);
    out_w(IC_FS_SCL_HCNT, 126u);
    out_w(IC_FS_SCL_LCNT, 187u);
    out_w(IC_FS_SPKLEN, 187u / 16u);
    out_w(IC_SDA_HOLD, 38u);
    return (in_w(IC_FS_SCL_LCNT) == 187u) ? E_OK : E_IO;
}

EXPORT ER drv_i2c1_write_reg(uint8_t addr7, uint8_t reg, uint8_t value)
{
    uint32_t const t0 = (uint32_t)app_time_us();
    ER             er = set_target(addr7);

    if (er != E_OK)
    {
        return er;
    }
    out_w(IC_DATA_CMD, reg);
    out_w(IC_DATA_CMD, (uint32_t)value | CMD_STOP);
    return finish(t0);
}

EXPORT ER drv_i2c1_read(uint8_t addr7, uint8_t reg, uint8_t * p_buf, uint8_t n)
{
    uint32_t const t0 = (uint32_t)app_time_us();
    ER             er;
    uint8_t        i;

    if ((n == 0u) || (n > 16u))
    {
        return E_PAR;
    }
    er = set_target(addr7);
    if (er != E_OK)
    {
        return er;
    }
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
