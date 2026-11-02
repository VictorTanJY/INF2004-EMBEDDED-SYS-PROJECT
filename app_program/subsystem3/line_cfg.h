/*
 *  line_cfg.h
 *  Subsystem 3 -- line-following and barcode tuning.  Owner: Buddy 3.
 *
 *  Values marked MEASURE are placeholders until measured on the track
 *  (A6, A7, NFR2, NFR3, NFR11).  Pure C header.
 */

#ifndef LINE_CFG_H
#define LINE_CFG_H

/* ---- Sensors --------------------------------------------------------- */
#define LINE_CH_L                   0u          /* ADC0 = GP26 */
#define LINE_CH_R                   1u          /* ADC1 = GP27 */
#define LINE_CH_BC                  2u          /* ADC2 = GP28 */
#define LINE_CH_COUNT               3u
#define LINE_SAMPLES_PER_BLOCK      10u         /* per channel, 1 kHz, 10 ms */
/* Barcode sensor lateral offset from the robot centre, + right, - left.
   Barcodes are always on the LEFT of the line in the direction of travel
   (team, 25 Oct), so the sensor is at the far left: -45 mm, MEASURE.
   Its ground speed is v + yaw rate x offset (yaw + left), i.e. a left
   sensor slows down in a left turn.                                     */
#define LINE_BC_OFFSET_MM           (-45)

/* 1 if the module's AO voltage is HIGHER over black (less reflection).
   MEASURE (A7): log raw values over black and white.                    */
#define LINE_BLACK_IS_HIGH          1
#define LINE_CONTRAST_MIN_X10       30          /* 3:1 (Week 6 Section 5)  */
#define LINE_CAL_MIN_SPAN           200         /* raw counts black - white */

/* Normalised 0..1000 (1000 = black).  The two line sensors straddle the
   line edges, so on the line both read ~500: "black" (junction) means
   fully black, "white" (gap / lost) means fully white; between the two
   is normal tracking.  Each has its own hysteresis.                     */
#define LINE_BLACK_ON               750
#define LINE_BLACK_OFF              600
#define LINE_WHITE_ON               150
#define LINE_WHITE_OFF              250
/* Barcode sensor: FIXED levels, not calibrated by the GP21 spin.  The
   spin sees the solid 18 mm line, which reads darker than a 3 mm bar under the
   sensor's spot, so spin levels made narrow bars too weak to detect.
   MEASURE (A7) on the robot: raw ADC over white paper and over a solid
   black bar, at the mounted height (LOG 34 after a GP21 spin over the
   track shows max << 16 | min of the barcode sensor).  Bar thresholds
   are per mille of (black - white).  150 / 80 suits a sensor whose
   white reads 0-2 %; 350 / 220 keeps a margin for white-paper
   noise (host model: white 8 % +- 12 %).  If narrow bars are missed on
   the robot, lower them toward 150 / 80.                               */
#define LINE_BC_WHITE_RAW           117         /* measured 1 Nov, LOG 34    */
#define LINE_BC_BLACK_RAW           3345        /* measured 1 Nov, LOG 34    */
#define BC_BLACK_ON                 350
#define BC_BLACK_OFF                220

/* ---- Steering (DRIVE every 10 ms) ----------------------------------- */
/* omega [mrad/s] = KP x err / 1000 + KD x d(err)/dt.
   err = norm_l - norm_r (-1000..+1000, + = line towards the left sensor):
   a plain difference changes gradually as the robot drifts, unlike the
   earlier (L - R) / (L + R), which jumped to +-1000 as soon as one sensor
   neared white and made the steering act nearly on/off.  MEASURE: tune
   on the track. */
/* 1 Nov (video, 150 mm/s): with KP 2600 / KD 60 the robot twitched left-
   right at the line edge and crawled at ~20 mm/s.  The D term acted on
   sensor noise (a 20-count jump in err gave 1.2 rad/s) and the output hit
   the +-3.5 rad/s clamp, so the wheels reversed in turn.  Gentler gains: */
/* Second video (19:10): smooth but still weaving +-10 deg at ~1.5 Hz.
   The two-sensor error saturates within a few mm of the edges, so the
   loop acts nearly on/off: lower P, a little more D to damp the swing. */
/* Third setting (1 Nov evening): cruise 50 mm/s, difference error, P only.
   KP 850 = 0.85 rad/s at full error (+-45 mm/s between the wheels). */
#define LINE_KP                     850
#define LINE_KD                     0           /* mrad/s per (err per 10 ms) */
#define LINE_STEER_SIGN             (1)         /* -1 if the robot steers away */
#define LINE_OMEGA_MAX              2000        /* mrad/s: inner wheel ~0 at 100 mm/s */
#define LINE_SLOW_K                 600         /* v x (1 - 0.6 |err|/1000) */
#define LINE_V_MIN_PCT              40          /* never below 40 % of cruise */
/* Both sensors went white: if the error just before was beyond this, the
   line left past one sensor (a drift, not a gap): keep turning toward it. */
#define LINE_EXIT_ERR               400         /* line clearly on one side (difference error) */
#define LINE_RECOVER_OMEGA          900         /* mrad/s: upper limit, see below */
/* 2 Nov: at the slow curve speed (36 mm/s at full error) the turn asked
   for more than the speed allows: the inner wheel target fell below the
   10 mm/s brake point or reversed, so it braked and restarted (jerks).
   The turn rate is now capped so the inner wheel keeps rolling forward at
   least this fast: tightest turn ~9 cm radius at 36 mm/s.  0 = the inner
   wheel may stop but never reverses (tighter turns). */
#define LINE_INNER_MIN_MM_S         12

/* ---- Junction / line loss (FR2) -------------------------------------- */
#define LINE_STRAIGHT_OMEGA         500         /* |omega| < 0.5 rad/s ...   */
#define LINE_STRAIGHT_UM            50000       /* ... over the last 50 mm   */
#define LINE_JUNCTION_REFRACT_UM    150000      /* no 2nd junction within 150 mm */
#define LINE_ERR_VALID_SUM          80          /* L + R below this: no error */

/* ---- Barcode (FR3, A6) -----------------------------------------------
   Element widths are measured in encoder distance, so they do not depend
   on speed.  Nothing assumes a fixed print size: the instructor's sheets
   come in three scales (narrow bar ~3.4 / 5.1 / 6.9 mm on screen, ~144 /
   213 / 288 mm per code when printed on A4), so the end of a code is a
   white run longer than both the quiet zone and 2.5 x its widest element. */
#define BC_QUIET_UM                 25000       /* 25 mm white before a code */
#define BC_END_FACTOR_X10           25          /* end: white > 2.5 x widest */
#define BC_MAX_LEN_UM               420000      /* 420 mm: longer = REJECT   */
#define BC_MAX_ELEMENTS             59          /* 6 characters              */
#define BC_COOLDOWN_UM              100000      /* 100 mm after a read       */
#define BC_RATIO_MIN_X10            15          /* wide >= 1.5 x narrow      */

#endif /* LINE_CFG_H */
