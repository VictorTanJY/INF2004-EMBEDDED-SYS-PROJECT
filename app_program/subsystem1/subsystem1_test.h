/** @file subsystem1_test.h
 *
 *  @brief On-target tests of subsystem 1, run by usermain() in test builds
 * (make TEST=1).
 */

#ifndef SUBSYSTEM1_TEST_H
#define SUBSYSTEM1_TEST_H

#include <tk/tkernel.h>

/** Run subsystem 1's on-target self-tests and print the [TEST] lines. */
IMPORT void subsystem1_test_run(void);

#endif /* SUBSYSTEM1_TEST_H */

/*** end of file ***/
