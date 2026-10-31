/*
 *	app.h
 *	Definitions shared by the whole team.  Changed only by team agreement.
 *
 *	Anything that belongs to ONE subsystem goes in that subsystem's own
 *	header instead (app_program/subsystemN/subsystemN.h).
 */

#ifndef APP_H
#define APP_H

#include <stdint.h>
#include <stdbool.h>
#include <tk/tkernel.h>		/* micro T-Kernel API (tk_xxx_yyy) */
#include <tm/tmonitor.h>	/* T-Monitor API: tm_printf() console output */
#include <bsp/libbsp.h>		/* Board helpers: gpio_set_pin/val, pwm_... */

/* ---------------------------------------------------------------------
 *  Pins: free for projects are GP0..GP22 and GP26..GP28.  GP0/GP1 are
 *  taken by UART0 when built with "make CRASH_REPORT=1" (see README).
 *  Never use GP23/24/25/29: they are wired to the Pico W radio.
 *  Claim every pin in TEAM.md section 2 before using it.
 * --------------------------------------------------------------------- */

/* ---------------------------------------------------------------------
 *  Task priorities: 1 is the highest; the kernel's USB console (3) and
 *  WiFi (4) tasks sit above every application task.  Each subsystem
 *  has its own band (TEAM.md section 4), defined in its own header.
 * --------------------------------------------------------------------- */
#define TASK_STACK_SIZE		2048	/* bytes; tm_printf needs ~1 KB */

/* Build switches (timing pins, bench mode): app_config.h */
#include "app_config.h"

/* ---------------------------------------------------------------------
 *  PicoCar core placement (Week 6 design review, Section 3).
 *  Control loops run on core 1; network, console and I2C-bound tasks on
 *  core 0, with the template's USB (3) and WiFi (4) services.
 * --------------------------------------------------------------------- */
#define APP_CORE0		TP_PRC1		/* processor 1 = core 0 */
#define APP_CORE1		TP_PRC2		/* processor 2 = core 1 */

/* Subsystem numbers, used as the "src" of telemetry records. */
#define APP_SRC_COMMS		1u	/* S1: CommsTask / HmiTask       */
#define APP_SRC_MOTION		2u	/* S2: MotionTask                */
#define APP_SRC_LINE		3u	/* S3: LineBarcodeTask           */
#define APP_SRC_SAFETY_IMU	4u	/* S4: SafetyTask / ImuTask      */
#define APP_SRC_OBST_MISSION	5u	/* S5: ObstacleTask / MissionTask */

/* ---------------------------------------------------------------------
 *  Snapshot lock (replaces the Week 6 mtx_snap mutex; Tribunal 1 gap 2).
 *
 *  Every status snapshot is read across cores (CommsTask on core 0 reads
 *  them all), and TA_INHERIT cannot bound a wait across cores: a core-0
 *  holder raised to the waiter's priority still runs below CommsTask (4)
 *  and the USB console (3).  Each snapshot therefore has its own lock,
 *  taken with interrupts masked on the holder's core, so the holder
 *  cannot be pre-empted while copying.  Rules:
 *    - hold it only to copy the snapshot struct in or out (a few us);
 *    - never call a kernel service, tm_printf or another lock inside;
 *    - one lock per snapshot, owned by the subsystem that writes it.
 *  On the single-core build (SMP=0) masking interrupts is enough.
 * --------------------------------------------------------------------- */
#if TK_SUPPORT_SMP
#include <tk/smp.h>
typedef T_SPLOCK	app_lock_t;

Inline void app_lock_init(app_lock_t *p_lock)
{
	(void)InitSpinLock(p_lock);
}

Inline UINT app_lock(app_lock_t *p_lock)
{
	UINT	intsts = 0;

	(void)ISpinLock(p_lock, &intsts);
	return intsts;
}

Inline void app_unlock(app_lock_t *p_lock, UINT intsts)
{
	(void)ISpinUnlock(p_lock, intsts);
}

/* Atomic increment of a shared 32-bit counter (writers on both cores). */
Inline void app_atomic_inc(UW *p_count)
{
	(void)atomic_inc(p_count);
}
#else
typedef UW		app_lock_t;

Inline void app_lock_init(app_lock_t *p_lock)
{
	*p_lock = 0;
}

Inline UINT app_lock(app_lock_t *p_lock)
{
	UINT	intsts;

	(void)p_lock;
	DI(intsts);
	return intsts;
}

Inline void app_unlock(app_lock_t *p_lock, UINT intsts)
{
	(void)p_lock;
	EI(intsts);
}

Inline void app_atomic_inc(UW *p_count)
{
	UINT	intsts;

	DI(intsts);
	(*p_count)++;
	EI(intsts);
}
#endif

/* Milliseconds since boot (low 32 bits of the kernel's operating time;
   wraps after 49.7 days).  Resolution is the 10 ms kernel tick.
   tk_get_otm, not tk_get_tim: the system clock is the operating time
   plus a TRON-epoch offset, so its low word starts near 3.36e9 at boot
   and a deadline initialised to 0 looks 2^31 ms in the future (first
   hardware run: telemetry never published). */
Inline UW app_time_ms(void)
{
	SYSTIM	tim;

	(void)tk_get_otm(&tim);
	return (UW)tim.lo;
}

/* RP2040 1 us system timer, low 32 bits (wraps every 71.6 min).  Correct
   only after the watchdog tick is set to 12 cycles (drv_watchdog, S4):
   from a cold power-on it otherwise runs 42.7 x slow (IS07 finding). */
Inline UW app_time_us(void)
{
	return in_w(TIMER_TIMERAWL);
}

/* 64-bit microseconds, read without a lock (high word re-checked). */
Inline UD app_time_us64(void)
{
	UW hi0;
	UW lo;
	UW hi1;

	do {
		hi0 = in_w(TIMER_TIMERAWH);
		lo  = in_w(TIMER_TIMERAWL);
		hi1 = in_w(TIMER_TIMERAWH);
	} while (hi0 != hi1);
	return ((UD)hi0 << 32) | lo;
}

/* Milliseconds from the 1 us timer (32 bits, wraps after 49.7 days):
   used by the 10 ms loops, which need better than the 10 ms tick. */
Inline UW app_time_ms_hw(void)
{
	return (UW)(app_time_us64() / 1000u);
}

/* Take peripheral blocks out of reset.  Never ASSERT a reset here: PWM,
   PIO0 and DMA are shared between subsystems (and with the radio). */
Inline void app_reset_release(UW mask)
{
	clr_w(RESETS_RESET, mask);
	while ((in_w(RESETS_RESET_DONE) & mask) != mask) {
		;
	}
}

/* Create and start a task.  prc = TP_PRC1 (core 0), TP_PRC2 (core 1), or
   0 for "any core".  Defined in app_main.c. */
IMPORT ID app_start_task(FP entry, PRI pri, UW prc, const char *name);

/* ---------------------------------------------------------------------
 *  Every subsystem provides these.  usermain() calls ALL the _init()
 *  functions first and only then ALL the _start() functions, so every
 *  kernel object exists before any task can use it.
 *    subsystemN_init()   create this subsystem's kernel objects
 *    subsystemN_start()  create and start this subsystem's tasks
 *    subsystemN_test()   on-target tests, only in test builds (make TEST=1)
 * --------------------------------------------------------------------- */
IMPORT ER subsystem1_init(void);
IMPORT ER subsystem2_init(void);
IMPORT ER subsystem3_init(void);
IMPORT ER subsystem4_init(void);
IMPORT ER subsystem5_init(void);

IMPORT ER subsystem1_start(void);
IMPORT ER subsystem2_start(void);
IMPORT ER subsystem3_start(void);
IMPORT ER subsystem4_start(void);
IMPORT ER subsystem5_start(void);

#if APP_TEST
IMPORT void subsystem1_test(void);
IMPORT void subsystem2_test(void);
IMPORT void subsystem3_test(void);
IMPORT void subsystem4_test(void);
IMPORT void subsystem5_test(void);
#endif

/* Print a readable message if a kernel call failed (negative error code) */
#define CHECK(er, what)							\
	do {								\
		INT check_er_ = (INT)(er);	/* evaluate once */	\
		if (check_er_ < E_OK) {					\
			tm_printf((UB*)"ERROR %s: %d\n", (what), check_er_); \
		}							\
	} while (0)

#endif /* APP_H */
