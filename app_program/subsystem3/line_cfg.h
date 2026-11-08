/*
 *  line_cfg.h
 *  Subsystem 3 -- line-following and barcode tuning.  Owner: Buddy 3.
 *
 *  Values marked MEASURE are placeholders until measured on the track
 *  (A6, A7, NFR2, NFR3, NFR11).  Pure C header.
 */

#ifndef LINE_CFG_H
#define LINE_CFG_H

/* ---- Turn slip, measured in the calibration spin (3 Nov) ------------ */
/* The car spins in place over the line; the pair of line sensors crosses
   it again after 180 deg of REAL rotation, so the encoder heading there
   / 180 is how much the encoders over-count a spin on this floor.  The
   bypass turns are asked for x this gain.  Blocks are 10 ms: 90 deg/s
   for 360 deg is 400 blocks; 800 allow a slow spin.                     */
#define LINE_SLIP_MAX_BLOCKS        800u
#define LINE_SLIP_SKIP_DDEG         600         /* ignore the line the spin starts on */
#define LINE_SLIP_DARK              500         /* normalised: a sensor on the line   */
#define LINE_SLIP_MIN_BLOCKS        3u          /* an 18 mm line at 120 mm, 90 deg/s: ~10 */
#define LINE_SLIP_GAIN_MIN_PM       800         /* plausible gains, permille          */
#define LINE_SLIP_GAIN_MAX_PM       1900

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
   track shows max << 16 | min of the barcode sensor).  These only scale
   the reading to 0..1000; bars are found by swing (BC_DELTA below), so
   an exact level matters much less than a sharp, low-mounted sensor.   */
#define LINE_BC_WHITE_RAW           117         /* measured 1 Nov, LOG 34    */
#define LINE_BC_BLACK_RAW           3345        /* measured 1 Nov, LOG 34    */
/* Bars and gaps by swing (2 Nov; replaces the fixed 350 / 220 levels,
   which missed every 3 mm element on the robot: darkest ~900 on wide
   bars, 3 elements instead of 29).  A bar starts when the reading rises
   BC_DELTA above the lightest point of the gap before it, and ends when
   it falls BC_DELTA below the darkest point of the bar.  Must stay above
   the peak-to-peak sample noise (host model +-120).                    */
#define BC_DELTA                    250
#define BC_DELTA_MIN                200         /* a bar is at least this dark */

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
/* 2 Nov: the side is judged from the strongest error over this distance
   before both went white, not the last one: on a curve the error fades as
   the line slides out past the outer sensor, and the car took the curve
   for a gap and drove straight off it. */
#define LINE_EXIT_PEAK_UM           40000       /* 40 mm */
#define LINE_RECOVER_OMEGA          900         /* mrad/s: firm, never reverses a wheel */
/* Steering ramp (2 Nov): the turn command changes by at most this much per
   10 ms block, so straight -> full steering (850) takes ~0.09 s instead of
   one step.  Lower = smoother but slower to react; 2000 = off (instant). */
#define LINE_OMEGA_STEP             2000        /* mrad/s per 10 ms: ramp off (50 and 100 lost the line on curves) */

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
/* 7 Nov: and at least this.  On the robot only the 9 mm bars of "*Z*" were
   seen (3 mm bars too faint at speed): the white between them inside the
   code reached 33 mm and ended the capture after 51 mm, before the
   profile pass could look for the faint bars.  After a real code the mat
   is white, so a longer end costs nothing.                            */
#define BC_END_MIN_UM               40000
#define BC_MAX_LEN_UM               420000      /* 420 mm: longer = REJECT   */
#define BC_MAX_ELEMENTS             59          /* 6 characters              */
#define BC_COOLDOWN_UM              100000      /* 100 mm after a read       */
#define BC_RATIO_MIN_X10            15          /* wide >= 1.5 x narrow      */
/* 7 Nov second try: the capture keeps the darkest reading of every
   BC_PROF_BIN_UM of travel; if the swing reading does not decode, bars
   are found again from that profile against this code's own levels
   (light = BC_PROF_WHITE_PCT percentile, dark = BC_PROF_DARK_PCT):
   first at mid-way, then at BC_PROF_LOW_DIV of the way up for faint thin
   bars.  Bars or spaces under BC_PROF_MIN_BINS bins are noise.          */
#define BC_PROF_BIN_UM              330
/* 7 Nov: the darkness record also keeps this much from before the first
   bar the swing reading saw ("*" starts with a narrow bar, and faint
   narrow bars do not start a capture): 25 mm = the quiet zone.       */
#define BC_PRE_BINS                 (BC_QUIET_UM / BC_PROF_BIN_UM)
#define BC_PROF_BINS                ((BC_MAX_LEN_UM / BC_PROF_BIN_UM) + BC_PRE_BINS + 64u)
#define BC_PROF_WHITE_PCT           10u
#define BC_PROF_DARK_PCT            95u
#define BC_PROF_LOW_DIV             4u
#define BC_PROF_MIN_BINS            3u
/* 9 Nov: a light print (bars only ~700 / 1000 on the robot, 3 mm bars
   rising less than BC_DELTA) is read once more by swing over the
   darkness record, with the swing set by THIS code: BC_ADAPT_PCT % of
   its dark - white span (9 Nov: then BC_ADAPT_PCT2 and BC_ADAPT_PCT3 %,
   01:54 run: 3 of 9 thin bars just reached the 20 % swing), but never under BC_ADAPT_NOISE_X x the white
   noise measured before the code, nor under BC_ADAPT_MIN.  Only used
   when that is below BC_DELTA (otherwise it is the first reading).    */
#define BC_ADAPT_PCT                20u
#define BC_ADAPT_PCT2               15u
#define BC_ADAPT_PCT3               10u
#define BC_ADAPT_NOISE_X            4u
#define BC_ADAPT_MIN                60u
/* Reading mode (7 Nov): while a code is being captured and the line is
   near the centre, steering is limited so the side-mounted barcode IR
   crosses the bars on a steady path (a weave swings it sideways and
   breaks the read up).  Curves (larger error) are not limited.        */
/* 7 Nov: 300 let a weave carry the car off the line onto the barcode
   (17:46 and 18:33 runs: line lost mid-read); 1000 still damps a swing. */
#define LINE_BC_OMEGA_MAX           1000        /* mrad/s while reading      */
#define LINE_BC_HOLD_ERR            400         /* |err| above this: no limit */

#endif /* LINE_CFG_H */
