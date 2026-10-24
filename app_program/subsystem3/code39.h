/*
 *  code39.h
 *  Subsystem 3 -- Code 39 decoding from measured element widths (FR3).
 *  Owner: Buddy 3.  Pure C (host-tested with the instructor's sample
 *  sheets: testing/subsystem3/host).
 *
 *  Input: the widths (any unit) of the alternating bars and spaces between
 *  the quiet zones, starting with a bar.  A code of k characters has
 *  10 k - 1 elements (9 per character + 1 gap between characters).
 *  Each character's 3 widest elements are wide; the full 44-character set
 *  is decoded ('0'-'9', 'A'-'Z', '-', '.', ' ', '$', '/', '+', '%', '*').
 *  The code is accepted forwards or reversed (read after a U-turn, or
 *  the robot travelling the other way), so A reads as A in both directions.
 *  Nothing is guessed: any element that cannot be classified rejects.
 */

#ifndef CODE39_H
#define CODE39_H

#include <stdint.h>
#include <stdbool.h>

typedef enum
{
    CODE39_OK             = 0,
    CODE39_NOT_BARCODE    = 1,  /* fewer than 9 elements (e.g. a crossing line): ignore */
    CODE39_ERR_COUNT      = 2,  /* element count is not 10 k - 1     */
    CODE39_ERR_RATIO      = 3,  /* wide and narrow not separable       */
    CODE39_ERR_PATTERN    = 4,  /* a character is not in the table     */
    CODE39_ERR_START_STOP = 5,  /* not framed by '*' in either direction */
    CODE39_ERR_LENGTH     = 6,  /* not exactly one data character      */
    CODE39_ERR_TOO_LONG   = 7,  /* physical length guard (capture)     */
    CODE39_ERR_TOO_MANY   = 8   /* too many elements (capture)         */
} code39_result_t;

#define CODE39_MAX_CHARS        6u

/* 9-bit pattern of a character, element 0 in bit 8, wide = 1; 0 if none. */
uint16_t code39_pattern(char c);

/* Character of a 9-bit pattern, or 0 if it is not a Code 39 character. */
char     code39_char(uint16_t pattern);

/*
 * Decode n widths.  On success p_text holds the characters between the
 * '*' start and stop (NUL-terminated, at most CODE39_MAX_CHARS - 2), and
 * *p_b_reversed says whether the widths had to be read backwards.
 * ratio_min_x10: wide must be >= ratio / 10 x narrow (15 = 1.5).
 */
code39_result_t code39_decode(uint32_t const * p_widths, uint16_t n, uint16_t ratio_min_x10,
                              char * p_text, bool * p_b_reversed);

/* Single data character (FR3): the decoded *X*, or an error. */
code39_result_t code39_decode_one(uint32_t const * p_widths, uint16_t n, uint16_t ratio_min_x10,
                                  char * p_char, bool * p_b_reversed);

#endif /* CODE39_H */
