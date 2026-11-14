/*
 *	test_report.h
 *	Minimal on-target test helpers.
 *
 *	Every check prints ONE line in a fixed format, so a saved console log
 *	can be searched for results ("grep '\[TEST\]' log.txt"):
 *
 *	    [TEST] S1-U01 PASS  ADC reads in range
 *	    [TEST] S1-U02 FAIL  queue rejects overflow   (task_x.c:57)
 *	    [TEST] S2-U08 SKIP  wheels did not turn   (no encoder counts)
 *	    [TEST] subsystem1 SUMMARY 1 passed, 1 failed, 0 skipped
 *
 *	Usage, inside a test function:
 *	    TEST_INIT();
 *	    TEST_CHECK("S1-U01", value >= 0 && value <= 4095, "ADC reads in range");
 *	    TEST_SUMMARY("subsystem1");
 *
 *	Test IDs must match the ones in testing/<subsystem>/TEST_LOG.md.
 *
 *	TEST_SKIP is for a check whose set-up is missing (no wall in front,
 *	motor battery off, or the emulator, which has no sensors or motors).
 *	A SKIP is NOT a pass: on the robot, fix the set-up and run again.
 *	Measured values are printed on "[INFO]" lines for the test log.
 */

#ifndef TEST_REPORT_H
#define TEST_REPORT_H

#include <tm/tmonitor.h>

#define TEST_INIT()	INT test_pass_ = 0, test_fail_ = 0, test_skip_ = 0

#define TEST_CHECK(id, cond, desc)					\
	do {								\
		if (cond) {						\
			test_pass_++;					\
			tm_printf((UB*)"[TEST] %s PASS  %s\n", (id), (desc)); \
		} else {						\
			test_fail_++;					\
			tm_printf((UB*)"[TEST] %s FAIL  %s   (%s:%d)\n",	\
				  (id), (desc), __FILE__, __LINE__);	\
		}							\
	} while (0)

#define TEST_SKIP(id, desc, why)					\
	do {								\
		test_skip_++;						\
		tm_printf((UB*)"[TEST] %s SKIP  %s   (%s)\n",		\
			  (id), (desc), (why));				\
	} while (0)

#define TEST_SUMMARY(name)						\
	tm_printf((UB*)"[TEST] %s SUMMARY %d passed, %d failed, %d skipped\n", \
		  (name), test_pass_, test_fail_, test_skip_)

#endif /* TEST_REPORT_H */
