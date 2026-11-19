/** @file subsystem2_test.h
 *
 *  @brief On-target tests of subsystem 2, run by usermain() in test builds
 * (make TEST=1).
 */

#ifndef SUBSYSTEM2_TEST_H
#define SUBSYSTEM2_TEST_H

#include <tk/tkernel.h>

/** Run subsystem 2's on-target self-tests and print the [TEST] lines. */
IMPORT void subsystem2_test_run(void);

#endif /* SUBSYSTEM2_TEST_H */

/*** end of file ***/
