/*
 *	app_pack.h
 *	Minimum-bit data representation (Change Requirement 1).
 *
 *	The helpers every packed layout in the firmware is built from.  Pure C
 *	(stdint only), so the same code runs in the firmware and in the host
 *	tests (testing/shared/host, SH-H05).
 *
 *	Rules for every packed field:
 *	  - a field holds only the bits its valid range needs;
 *	  - an unsigned value SATURATES at the largest value of its field, so
 *	    a value out of range is never stored as a wrong, wrapped value;
 *	  - a signed value is stored offset-binary in an unsigned field
 *	    (stored = value + 2^(width-1)), saturating at both ends, so no
 *	    signed bit-field and no bit operation on signed data is used;
 *	  - counters that only feed differences (encoder counts, distance,
 *	    sequence numbers, timestamps) keep their low bits and WRAP; the
 *	    reader takes modular differences (pk_wrap_diff).
 *
 *	Three forms:
 *	  - words in RAM (messages): pk_set() / pk_get() place a field at a
 *	    fixed bit position of a uint32_t with masks and shifts;
 *	  - records in RAM (shared snapshots): a layout table (pk_field_t)
 *	    lists each field of the task-local struct with its width and kind;
 *	    pk_pack_rec() / pk_unpack_rec() move the fields into / out of an
 *	    array of words, one after the other.  The table is also the data
 *	    representation table of that record;
 *	  - byte streams (MQTT payloads): pk_writer_t / pk_reader_t append
 *	    fields LSB first, so bit 0 of the stream is bit 0 of byte 0 and the
 *	    layout does not depend on the compiler or the CPU's byte order.
 *	    The laptop decoder (dashboard/picocar_codec.py) reads the same way.
 */

#ifndef APP_PACK_H
#define APP_PACK_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* Mask of the low width bits (width 1..32). */
static inline uint32_t pk_mask(uint8_t width)
{
	return (width >= 32u) ? 0xFFFFFFFFuL : ((1uL << width) - 1uL);
}

/* Unsigned value saturated to a width-bit field. */
static inline uint32_t pk_sat_u(uint32_t value, uint8_t width)
{
	uint32_t const max = pk_mask(width);

	return (value > max) ? max : value;
}

/* Signed value -> offset-binary field of width bits (2..32), saturated to
   -2^(width-1) .. 2^(width-1) - 1. */
static inline uint32_t pk_off_s(int32_t value, uint8_t width)
{
	int64_t const half = (int64_t)1 << (width - 1u);
	int64_t       clamped    = (int64_t)value;

	if (clamped < -half)
	{
		clamped = -half;
	}
	else if (clamped > (half - 1))
	{
		clamped = half - 1;
	}
	else
	{
		/* in range */
	}
	return (uint32_t)(uint64_t)(clamped + half);
}

/* Offset-binary field -> signed value. */
static inline int32_t pk_unoff_s(uint32_t field, uint8_t width)
{
	int64_t const half = (int64_t)1 << (width - 1u);

	return (int32_t)((int64_t)(field & pk_mask(width)) - half);
}

/* Difference a - b of two width-bit wrapping counters, as a signed value
   (correct while the true difference is within +-2^(width-1)). */
static inline int32_t pk_wrap_diff(uint32_t lhs, uint32_t rhs, uint8_t width)
{
	uint32_t const diff    = (lhs - rhs) & pk_mask(width);
	uint32_t const half = 1uL << (width - 1u);

	return (diff >= half) ? (int32_t)((int64_t)diff - ((int64_t)1 << width)) : (int32_t)diff;
}

/* ---- Fields in a word: value already in range (saturate first) ------ */
static inline uint32_t pk_set(uint32_t word, uint8_t shift, uint8_t width, uint32_t value)
{
	uint32_t const mask = pk_mask(width) << shift;

	return (word & ~mask) | ((value << shift) & mask);
}

static inline uint32_t pk_get(uint32_t word, uint8_t shift, uint8_t width)
{
	return (word >> shift) & pk_mask(width);
}

/* ---- Byte streams, LSB first ---------------------------------------- */
typedef struct
{
	uint8_t  *p_buf;
	uint16_t  cap;          /* bytes                                    */
	uint16_t  bits;         /* bits written                             */
	bool      b_overflow;   /* a field did not fit: the frame is invalid */
} pk_writer_t;

typedef struct
{
	uint8_t const *p_buf;
	uint16_t       len;     /* bytes                                    */
	uint16_t       bits;    /* bits read                                */
	bool           b_short; /* read past the end: the frame is invalid  */
} pk_reader_t;

void     pk_writer_init(pk_writer_t *p_w, uint8_t *p_buf, uint16_t cap);
void     pk_put(pk_writer_t *p_w, uint32_t value, uint8_t width);  /* low width bits, 1..32 */
uint16_t pk_writer_len(pk_writer_t const *p_w);                     /* bytes, 0 if overflowed */
void     pk_reader_init(pk_reader_t *p_r, uint8_t const *p_buf, uint16_t len);
uint32_t pk_take(pk_reader_t *p_r, uint8_t width);                  /* 0 and b_short past the end */

/* ---- Code 39 data character in 6 bits (index 1..43, 0 = none) ------- */
#define PK_SYM_COUNT	43u
uint8_t  pk_sym_index(char chr);
char     pk_sym_char(uint8_t index);

/* ---- Records: table-driven packing of a struct into words ------------ */
typedef enum
{
	PK_U   = 0,		/* unsigned, saturates at the field maximum          */
	PK_S   = 1,		/* signed, offset-binary, saturates at both ends     */
	PK_W   = 2,		/* counter / time: low bits, wraps (32 = whole)      */
	PK_UN  = 3,		/* unsigned with a "none" value: the C type's maximum
				   (0xFF, 0xFFFF, ...) <-> the field's all-ones;
				   other values saturate one below it              */
	PK_SYM = 4		/* char: Code 39 symbol index, 6 bits                */
} pk_kind_t;

typedef struct
{
	uint16_t offset;	/* offsetof() the member                      */
	uint8_t  size;		/* sizeof() the member: 1, 2 or 4             */
	uint8_t  width;		/* bits stored, 1..32                          */
	uint8_t  kind;		/* pk_kind_t                                   */
} pk_field_t;

#define PK_FIELD(type, member, width, kind) \
	{ (uint16_t)offsetof(type, member), (uint8_t)sizeof(((type *)0)->member), (uint8_t)(width), (uint8_t)(kind) }

/* Words needed for a number of bits. */
#define PK_WORDS(bits)	(((bits) + 31u) / 32u)

uint32_t pk_layout_bits(pk_field_t const *p_f, uint16_t n_fields);
/* Pack every field of p_rec into p_words (nwords, cleared first). */
void     pk_pack_rec(pk_field_t const *p_f, uint16_t n_fields, void const *p_rec,
		     uint32_t *p_words, uint16_t nwords);
/* Unpack p_words into the fields of p_rec (other bytes are left as they are). */
void     pk_unpack_rec(pk_field_t const *p_f, uint16_t n_fields, uint32_t const *p_words, void *p_rec);

#endif /* APP_PACK_H */
