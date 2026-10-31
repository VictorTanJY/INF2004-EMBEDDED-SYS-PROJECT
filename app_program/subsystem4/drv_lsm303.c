/* LSM303D driver for the robot identified by WHO_AM_I 0x49 at 0x1E.
 * Datasheet: https://www.st.com/resource/en/datasheet/lsm303d.pdf
 * API units and sensor-axis order match the existing imu_core interface.
 */
#include "drv_lsm303.h"
#include "drv_i2c1.h"
#include "imu_cfg.h"

#define SENSOR_ADDR 0x1Eu
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
    {0x33u, 0x00u}, /* No minimum interrupt duration. */
    {0x20u, 0x6Fu}  /* Acceleration 100 Hz, XYZ on, block data update. */
};

static uint32_t g_diag;

/* Diagnostic layout:
 * bits 31..24: stage (0 = success, 1 = identity read,
 *                    2 = unexpected identity, 4 = configuration)
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

EXPORT ER drv_lsm303_init(void)
{
    uint8_t id = 0u;
    uint8_t reference[3];
    uint32_t i;
    ER er;

    g_diag = 0u;

    er = drv_i2c1_read(SENSOR_ADDR, 0x0Fu, &id, 1u);
    if (er != E_OK)
        return init_result(1u, er, id);

    if (id != 0x49u)
        return init_result(2u, E_OBJ, id);

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

    return init_result(0u, E_OK, id);
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