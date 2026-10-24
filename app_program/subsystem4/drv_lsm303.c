/*
 *  drv_lsm303.c
 *  Subsystem 4 -- LSM303DLHC driver.  See drv_lsm303.h.
 */

#include "drv_lsm303.h"
#include "drv_i2c1.h"
#include "imu_cfg.h"

#define ACC_ADDR            0x19u
#define MAG_ADDR            0x1Eu

#define CTRL_REG1_A         0x20u
#define CTRL_REG2_A         0x21u
#define CTRL_REG3_A         0x22u
#define CTRL_REG4_A         0x23u
#define CTRL_REG5_A         0x24u
#define OUT_X_L_A           0x28u
#define INT1_CFG_A          0x30u
#define INT1_THS_A          0x32u
#define INT1_DURATION_A     0x33u
#define AUTO_INC            0x80u

#define CRA_REG_M           0x00u
#define CRB_REG_M           0x01u
#define MR_REG_M            0x02u
#define OUT_X_H_M           0x03u
#define IRA_REG_M           0x0Au

typedef struct
{
    uint8_t addr;
    uint8_t reg;
    uint8_t value;
} reg_init_t;

static reg_init_t const k_init[] =
{
    { ACC_ADDR, CTRL_REG1_A,     0x57u },   /* 100 Hz, normal power, X Y Z on      */
    { ACC_ADDR, CTRL_REG2_A,     0x04u },   /* high-pass filter on INT1 (HPIS1)    */
    { ACC_ADDR, CTRL_REG4_A,     0x18u },   /* +-4 g (FS = 01), high resolution    */
    { ACC_ADDR, CTRL_REG5_A,     0x00u },   /* INT1 not latched                    */
    { ACC_ADDR, INT1_THS_A,      (uint8_t)IMU_INT1_THS },
    { ACC_ADDR, INT1_DURATION_A, 0x00u },
    { ACC_ADDR, INT1_CFG_A,      0x2Au },   /* OR of X, Y, Z high events           */
    { ACC_ADDR, CTRL_REG3_A,     0x40u },   /* I1_AOI1: event 1 on the INT1 pin    */
    { MAG_ADDR, CRA_REG_M,       0x18u },   /* 75 Hz                               */
    { MAG_ADDR, CRB_REG_M,       0x20u },   /* +-1.3 gauss                         */
    { MAG_ADDR, MR_REG_M,        0x00u }    /* continuous conversion               */
};

EXPORT ER drv_lsm303_init(void)
{
    uint8_t  id = 0u;
    ER       er;
    uint32_t i;

    er = drv_i2c1_read(MAG_ADDR, IRA_REG_M, &id, 1u);
    if (er != E_OK)
    {
        return er;
    }
    if (id != 0x48u)
    {
        return E_OBJ;
    }
    for (i = 0u; i < (uint32_t)(sizeof(k_init) / sizeof(k_init[0])); i++)
    {
        er = drv_i2c1_write_reg(k_init[i].addr, k_init[i].reg, k_init[i].value);
        if (er != E_OK)
        {
            return er;
        }
    }
    return E_OK;
}

EXPORT ER drv_lsm303_read_accel(int32_t acc_mg[3])
{
    uint8_t b[6];
    ER      er = drv_i2c1_read(ACC_ADDR, (uint8_t)(OUT_X_L_A | AUTO_INC), b, 6u);
    uint8_t k;

    if (er != E_OK)
    {
        return er;
    }
    for (k = 0u; k < 3u; k++)
    {
        /* 12-bit value, left-justified in a little-endian 16-bit word */
        int16_t const raw = (int16_t)(uint16_t)((uint16_t)b[2u * k] | ((uint16_t)b[(2u * k) + 1u] << 8));

        acc_mg[k] = ((int32_t)raw / 16) * IMU_ACC_MG_PER_LSB;
    }
    return E_OK;
}

EXPORT ER drv_lsm303_read_mag(int16_t mag[3])
{
    uint8_t b[6];
    ER      er = drv_i2c1_read(MAG_ADDR, OUT_X_H_M, b, 6u);

    if (er != E_OK)
    {
        return er;
    }
    /* order on the wire: X (H, L), Z (H, L), Y (H, L) */
    mag[0] = (int16_t)(uint16_t)(((uint16_t)b[0] << 8) | b[1]);
    mag[2] = (int16_t)(uint16_t)(((uint16_t)b[2] << 8) | b[3]);
    mag[1] = (int16_t)(uint16_t)(((uint16_t)b[4] << 8) | b[5]);
    return E_OK;
}
