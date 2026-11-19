/** @file line_task.h
 *
 *  @brief LineBarcodeTask: line following and barcode decoding. Created and
 * started by subsystem3_start().
 */

#ifndef LINE_TASK_H
#define LINE_TASK_H

#include <tk/tkernel.h>

/** LineBarcodeTask: line following, junctions and barcodes from each 10 ms ADC
    block.  Created by subsystem3_start(). */
IMPORT void line_task_thread(INT stacd, void * p_exinf);

#endif /* LINE_TASK_H */

/*** end of file ***/
