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
/* LSM303AGR magnetometer (same address; sold on some GY-511 boards) */
#define WHO_AM_I_M          0x4Fu
#define CFG_REG_A_M         0x60u
#define CFG_REG_C_M         0x62u
#define OUTX_L_REG_M        0x68u
#define ID_DLHC             0x48u
#define ID_AGR              0x40u

typedef enum
{
    CHIP_NONE = 0,
    CHIP_DLHC = 1,
    CHIP_AGR  = 2
} chip_t;

static uint8_t  g_chip = (uint8_t)CHIP_NONE;
static uint32_t g_diag;

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

static uint32_t diag(uint8_t stage, ER er, uint8_t id_dlhc, uint8_t id_agr)
{
    return ((uint32_t)stage << 24) | (((uint32_t)(-er) & 0xFFu) << 16)
         | ((uint32_t)id_dlhc << 8) | (uint32_t)id_agr;
}

EXPORT uint32_t drv_lsm303_diag(void)
{
    return g_diag;
}

EXPORT ER drv_lsm303_init(void)
{
    uint8_t  id_dlhc = 0u;
    uint8_t  id_agr  = 0u;
    uint8_t  b;
    ER       er;
    uint32_t i;

    g_chip = (uint8_t)CHIP_NONE;

    /* 1. Magnetometer identity: LSM303DLHC (IRA_REG_M = 0x48), else
          LSM303AGR (WHO_AM_I_M = 0x40), both at 0x1E. */
    er = drv_i2c1_read(MAG_ADDR, IRA_REG_M, &id_dlhc, 1u);
    if (er != E_OK)
    {
        g_diag = diag(1u, er, 0u, 0u);
        return er;
    }
    if (id_dlhc == ID_DLHC)
    {
        g_chip = (uint8_t)CHIP_DLHC;
    }
    else
    {
        er = drv_i2c1_read(MAG_ADDR, WHO_AM_I_M, &id_agr, 1u);
        if ((er == E_OK) && (id_agr == ID_AGR))
        {
            g_chip = (uint8_t)CHIP_AGR;
        }
        else
        {
            g_diag = diag(2u, (er != E_OK) ? er : E_OBJ, id_dlhc, id_agr);
            return E_OBJ;
        }
    }

    /* 2. Accelerometer answers at 0x19 (register map shared by both chips). */
    er = drv_i2c1_read(ACC_ADDR, CTRL_REG1_A, &b, 1u);
    if (er != E_OK)
    {
        g_diag = diag(3u, er, id_dlhc, id_agr);
        g_chip = (uint8_t)CHIP_NONE;
        return er;
    }

    /* 3. Configuration. */
    for (i = 0u; i < (uint32_t)(sizeof(k_init) / sizeof(k_init[0])); i++)
    {
        if ((k_init[i].addr == MAG_ADDR) && (g_chip == (uint8_t)CHIP_AGR))
        {
            continue;                   /* DLHC magnetometer registers */
        }
        er = drv_i2c1_write_reg(k_init[i].addr, k_init[i].reg, k_init[i].value);
        if (er != E_OK)
        {
            g_diag = diag(4u, er, id_dlhc, id_agr);
            g_chip = (uint8_t)CHIP_NONE;
            return er;
        }
    }
    if (g_chip == (uint8_t)CHIP_AGR)
    {
        /* 100 Hz continuous, temperature compensated; block data update */
        er = drv_i2c1_write_reg(MAG_ADDR, CFG_REG_A_M, 0x8Cu);
        if (er == E_OK)
        {
            er = drv_i2c1_write_reg(MAG_ADDR, CFG_REG_C_M, 0x10u);
        }
        if (er != E_OK)
        {
            g_diag = diag(4u, er, id_dlhc, id_agr);
            g_chip = (uint8_t)CHIP_NONE;
            return er;
        }
    }
    g_diag = diag(0u, E_OK, id_dlhc, id_agr);
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
    ER      er;

    if (g_chip == (uint8_t)CHIP_AGR)
    {
        /* AGR: X, Y, Z little-endian from 0x68, 1.5 mG/LSB.  Only the
           direction is used (heading), so the scale difference to the
           DLHC (1100 LSB/G) does not matter. */
        er = drv_i2c1_read(MAG_ADDR, OUTX_L_REG_M, b, 6u);
        if (er != E_OK)
        {
            return er;
        }
        mag[0] = (int16_t)(uint16_t)((uint16_t)b[0] | ((uint16_t)b[1] << 8));
        mag[1] = (int16_t)(uint16_t)((uint16_t)b[2] | ((uint16_t)b[3] << 8));
        mag[2] = (int16_t)(uint16_t)((uint16_t)b[4] | ((uint16_t)b[5] << 8));
        return E_OK;
    }
    er = drv_i2c1_read(MAG_ADDR, OUT_X_H_M, b, 6u);
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
