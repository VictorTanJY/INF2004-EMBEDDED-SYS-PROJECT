/*
 *  imu_cfg.h
 *  Subsystem 4 -- IMU mounting, hump detection and safety tuning.
 *  Owner: Buddy 4.  Values marked MEASURE are placeholders (A8, A9).
 *  Pure C header.
 */

#ifndef IMU_CFG_H
#define IMU_CFG_H

/* ---- Mounting: which accelerometer axis points forward / left / up,
   and its sign.  MEASURE (A8): tilt the robot nose-up and left-side-up by
   hand and check that pitch and roll go positive.                       */
#define IMU_AXIS_FWD                0           /* 0 = X, 1 = Y, 2 = Z */
#define IMU_SIGN_FWD                (1)
#define IMU_AXIS_LEFT               1
#define IMU_SIGN_LEFT               (1)
#define IMU_AXIS_UP                 2
#define IMU_SIGN_UP                 (-1)

/* LSM303DLHC accelerometer at +-4 g, high resolution: 2 mg per LSB of the
   12-bit left-justified value (datasheet table 3).                      */
#define IMU_ACC_MG_PER_LSB          2
#define IMU_ACC_LPF_SHIFT           2           /* EMA, alpha = 1/4 */

/* ---- Hump detection (FR6, FR7, Week 6 Section 6.3) ------------------- */
#define IMU_HUMP_ON_CDEG            500         /* +5 deg: climbing          */
#define IMU_CREST_CDEG              200         /* below +2 deg: crest       */
#define IMU_DESC_CDEG               (-500)      /* below -5 deg: descending  */
#define IMU_FLAT_CDEG               200         /* |pitch| < 2 deg: flat     */
#define IMU_DWELL_UM                20000       /* 20 mm of travel guards    */
#define IMU_RAMP_UM                 600000      /* climb > 600 mm = ramp     */
#define IMU_ACC_GATE_MM_S2          500         /* |dv/dt| < 0.5 m/s^2       */
/* Wheelbase for the short-hump geometry check, peak >= L x sin(theta max):
   distance from the drive axle to the front caster.  MEASURE.           */
#define IMU_WHEELBASE_MM            110

/* ---- Safety ---------------------------------------------------------- */
#define IMU_TILT_ON_CDEG            3000        /* > 30 deg: TILT (FR15)     */
#define IMU_TILT_OFF_CDEG           2500
#define IMU_IMPACT_MG               1500        /* |delta a| > 1.5 g         */
#define IMU_IMPACT_HOLD_MS          300
#define IMU_I2C_FAULT_COUNT         3           /* consecutive errors        */
#define IMU_RECOVER_SAMPLES         10          /* good samples before pitch_valid */

/* INT1 high-g interrupt threshold, high-pass filtered: 1.5 g at +-4 g
   full scale is 1500 / 32 mg per LSB = 47 (datasheet INT1_THS_A).       */
#define IMU_INT1_THS                47

/* ---- Motion classification (FR8) -------------------------------------- */
#define IMU_STATIONARY_MM_S         10
#define IMU_TURNING_CDEG_S          3000        /* > 30 deg/s */
#define IMU_ACCEL_MM_S2             300
#define IMU_CLASS_HOLD_MS           200         /* minimum time between reports */

/* ---- Calibration ----------------------------------------------------- */
#define IMU_LEVEL_MAX_SPREAD_MG     60          /* robot must stand still      */
#define IMU_MAG_MIN_SPAN            100         /* raw counts, per axis, in the spin */

#endif /* IMU_CFG_H */
