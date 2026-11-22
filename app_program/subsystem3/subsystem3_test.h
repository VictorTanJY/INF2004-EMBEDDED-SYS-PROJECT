/** @file subsystem3_test.h
 *
 *  @brief On-target tests of subsystem 3, run by usermain() in test builds
 * (make TEST=1).
 */

#ifndef SUBSYSTEM3_TEST_H
#define SUBSYSTEM3_TEST_H

#include <tk/tkernel.h>

/** Run subsystem 3's on-target self-tests and print the [TEST] lines. */
IMPORT void subsystem3_test_run(void);

#endif /* SUBSYSTEM3_TEST_H */

/*** end of file ***/
