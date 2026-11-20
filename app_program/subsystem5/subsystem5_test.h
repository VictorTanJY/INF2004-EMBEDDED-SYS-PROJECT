/** @file subsystem5_test.h
 *
 *  @brief On-target tests of subsystem 5, run by usermain() in test builds
 * (make TEST=1).
 */

#ifndef SUBSYSTEM5_TEST_H
#define SUBSYSTEM5_TEST_H

#include <tk/tkernel.h>

/** Run subsystem 5's on-target self-tests and print the [TEST] lines. */
IMPORT void subsystem5_test_run(void);

#endif /* SUBSYSTEM5_TEST_H */

/*** end of file ***/
