/* LSM303D driver for the robot identified by WHO_AM_I 0x49 at 0x1D.
 * Datasheet: https://www.st.com/resource/en/datasheet/lsm303d.pdf
 * API units and sensor-axis order match the existing imu_core interface.
 */
#include "drv_lsm303.h"
#include "drv_i2c1.h"
#include "imu_cfg.h"
#include "subsystem4.h"

#define SENSOR_ADDR DRV_LSM303_ADDR
#define AUTO_INC    0x80u

typedef struct
{
    uint8_t reg;
    uint8_t value;
} reg_init_t;

static reg_init_t const k_init[] =
{
    {0x22u, 0x00u}, /* Disconnect INT1 while configuring. */
    {0x30u, 0x00u}, /* Disable interrupt generator during setup. */
    {0x1Fu, 0x02u}, /* High-pass filter for interrupt generator 1 only. */
    {0x21u, 0x08u}, /* Acceleration +/-4 g, 16-bit, self-test off. */
    {0x23u, 0x00u}, /* No routes to INT2. */
    {0x24u, 0x70u}, /* Magnetic high resolution, 50 Hz; IRQ not latched. */
    {0x25u, 0x20u}, /* Magnetic +/-4 gauss. */
    {0x26u, 0x00u}, /* Continuous magnetic conversion; unfiltered accel output. */
    {0x12u, 0x08u}, /* Push-pull, active-high interrupt pins. */
    {0x32u, (uint8_t)IMU_INT1_THS},
    {0x33u, (uint8_t)IMU_INT1_DUR}, /* Minimum duration, samples at 100 Hz. */
    {0x20u, 0x6Fu}  /* Acceleration 100 Hz, XYZ on, block data update. */
};

static uint32_t g_diag;

/* Diagnostic layout:
 * bits 31..24: stage (0 = success, 1 = identity read,
 *                    2 = unexpected identity, 4 = configuration,
 *                    5 = acceleration stuck at full scale after 2 tries)
 * bits 23..16: negated error code, low byte
 * bits 15..8:  observed WHO_AM_I value
 * bits 7..0:   reserved
 */
static ER init_result(uint8_t stage, ER er, uint8_t id)
{
    g_diag = ((uint32_t)stage << 24)
           | (((uint32_t)(-er) & 0xFFu) << 16)
           | ((uint32_t)id << 8);
    return er;
}

EXPORT uint32_t drv_lsm303_diag(void)
{
    return g_diag;
}

/* 2 Nov: twice after a Pico reset (the chip keeps power) the accelerometer
   read full scale on two axes with no noise (level calibration: up axis
   3996 mg, shake 0; 45 deg tilt at boot).  The chip is now rebooted
   (CTRL0 BOOT) before it is configured, and the first readings are
   checked: full scale on any axis = stuck, so reboot and configure again. */
#define BOOT_WAIT_MS        20      /* CTRL0 BOOT: reload trimming, ~5 ms */
#define FIRST_DATA_WAIT_MS  50      /* 5 samples at 100 Hz                 */
#define STUCK_MG            3900    /* +/-4 g full scale is 3997 mg        */

/* Sleep in 10 ms steps and keep ImuTask's heartbeat (100 ms timeout)
   alive: a reboot and retry take up to ~150 ms. */
static void wait_ms(uint32_t ms)
{
    uint32_t t;

    for (t = 0u; t < ms; t += 10u)
    {
        (void)tk_dly_tsk(10);
        safety_heartbeat(SAFETY_HB_IMU);
    }
}

static ER init_once(void)
{
    uint8_t id = 0u;
    uint8_t reference[3];
    int32_t acc[3];
    uint32_t i;
    ER er;

    g_diag = 0u;

    er = drv_i2c1_read(SENSOR_ADDR, 0x0Fu, &id, 1u);
    if (er != E_OK)
        return init_result(1u, er, id);

    if (id != 0x49u)
        return init_result(2u, E_OBJ, id);

    /* Reboot the chip's memory content first (CTRL0 BOOT). */
    er = drv_i2c1_write_reg(SENSOR_ADDR, 0x1Fu, 0x80u);
    if (er != E_OK)
        return init_result(4u, er, id);
    wait_ms(BOOT_WAIT_MS);

    for (i = 0u; i < sizeof(k_init) / sizeof(k_init[0]); ++i)
    {
        uint8_t check = 0u;

        er = drv_i2c1_write_reg(
            SENSOR_ADDR, k_init[i].reg, k_init[i].value);
        if (er != E_OK)
            return init_result(4u, er, id);

        er = drv_i2c1_read(
            SENSOR_ADDR, k_init[i].reg, &check, 1u);
        if (er != E_OK)
            return init_result(4u, er, id);

        if (check != k_init[i].value)
            return init_result(4u, E_IO, id);
    }

    /* Reset high-pass references before enabling INT1. */
    er = drv_i2c1_read(
        SENSOR_ADDR, (uint8_t)(0x1Cu | AUTO_INC), reference, 3u);
    if (er != E_OK)
        return init_result(4u, er, id);

    /* OR-of-positive-axis high-g interrupt. */
    er = drv_i2c1_write_reg(SENSOR_ADDR, 0x30u, 0x2Au);
    if (er != E_OK)
        return init_result(4u, er, id);

    /* Route interrupt generator 1 to INT1. */
    er = drv_i2c1_write_reg(SENSOR_ADDR, 0x22u, 0x20u);
    if (er != E_OK)
        return init_result(4u, er, id);

    /* First readings: an axis at full scale means the output is stuck. */
    wait_ms(FIRST_DATA_WAIT_MS);
    er = drv_lsm303_read_accel(acc);
    if (er != E_OK)
        return init_result(4u, er, id);
    for (i = 0u; i < 3u; ++i)
    {
        if ((acc[i] >= STUCK_MG) || (acc[i] <= -STUCK_MG))
            return init_result(5u, E_IO, id);
    }

    return init_result(0u, E_OK, id);
}

EXPORT ER drv_lsm303_init(void)
{
    ER er = init_once();

    if ((er != E_OK) && ((g_diag >> 24) == 5u))
    {
        er = init_once();               /* stuck: reboot and try once more */
    }
    return er;
}

static int16_t le16(uint8_t lo, uint8_t hi)
{
    uint16_t const word =
        (uint16_t)((uint16_t)lo | ((uint16_t)hi << 8));

    /* Avoid implementation-defined unsigned-to-signed overflow. */
    return (int16_t)((word & 0x8000u)
        ? (int32_t)word - 65536
        : (int32_t)word);
}

EXPORT ER drv_lsm303_read_accel(int32_t acc_mg[3])
{
    uint8_t b[6];
    uint8_t k;
    ER er;

    er = drv_i2c1_read(
        SENSOR_ADDR, (uint8_t)(0x28u | AUTO_INC), b, 6u);
    if (er != E_OK)
        return er;

    for (k = 0u; k < 3u; ++k)
    {
        /* LSM303D +/-4 g: 0.122 mg per full 16-bit count. */
        acc_mg[k] =
            ((int32_t)le16(b[2u * k], b[2u * k + 1u]) * 122)
            / 1000;
    }

    return E_OK;
}

EXPORT ER drv_lsm303_read_mag(int16_t mag[3])
{
    uint8_t b[6];
    uint8_t k;
    ER er;

    er = drv_i2c1_read(
        SENSOR_ADDR, (uint8_t)(0x08u | AUTO_INC), b, 6u);
    if (er != E_OK)
        return er;

    for (k = 0u; k < 3u; ++k)
        mag[k] = le16(b[2u * k], b[2u * k + 1u]);

    return E_OK;
}