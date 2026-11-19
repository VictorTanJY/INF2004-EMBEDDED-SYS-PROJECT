/** @file subsystem4_test.h
 *
 *  @brief On-target tests of subsystem 4, run by usermain() in test builds
 * (make TEST=1).
 */

#ifndef SUBSYSTEM4_TEST_H
#define SUBSYSTEM4_TEST_H

#include <tk/tkernel.h>

/** Run subsystem 4's on-target self-tests and print the [TEST] lines. */
IMPORT void subsystem4_test_run(void);

#endif /* SUBSYSTEM4_TEST_H */

/*** end of file ***/
