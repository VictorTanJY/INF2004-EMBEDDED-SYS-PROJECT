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
 *	Two forms:
 *	  - words in RAM (messages, snapshots): pk_set() / pk_get() place a
 *	    field at a fixed bit position of a uint32_t with masks and shifts;
 *	  - byte streams (MQTT payloads): pk_writer_t / pk_reader_t append
 *	    fields LSB first, so bit 0 of the stream is bit 0 of byte 0 and the
 *	    layout does not depend on the compiler or the CPU's byte order.
 *	    The laptop decoder (dashboard/picocar_codec.py) reads the same way.
 */

#ifndef APP_PACK_H
#define APP_PACK_H

#include <stdint.h>
#include <stdbool.h>

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
	int64_t       v    = (int64_t)value;

	if (v < -half)
	{
		v = -half;
	}
	else if (v > (half - 1))
	{
		v = half - 1;
	}
	else
	{
		/* in range */
	}
	return (uint32_t)(uint64_t)(v + half);
}

/* Offset-binary field -> signed value. */
static inline int32_t pk_unoff_s(uint32_t field, uint8_t width)
{
	int64_t const half = (int64_t)1 << (width - 1u);

	return (int32_t)((int64_t)(field & pk_mask(width)) - half);
}

/* Difference a - b of two width-bit wrapping counters, as a signed value
   (correct while the true difference is within +-2^(width-1)). */
static inline int32_t pk_wrap_diff(uint32_t a, uint32_t b, uint8_t width)
{
	uint32_t const d    = (a - b) & pk_mask(width);
	uint32_t const half = 1uL << (width - 1u);

	return (d >= half) ? (int32_t)((int64_t)d - ((int64_t)1 << width)) : (int32_t)d;
}

/* ---- Fields in a word: value already in range (saturate first) ------ */
static inline uint32_t pk_set(uint32_t word, uint8_t shift, uint8_t width, uint32_t value)
{
	uint32_t const m = pk_mask(width) << shift;

	return (word & ~m) | ((value << shift) & m);
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

static inline void pk_writer_init(pk_writer_t *p_w, uint8_t *p_buf, uint16_t cap)
{
	uint16_t i;

	for (i = 0u; i < cap; i++)
	{
		p_buf[i] = 0u;
	}
	p_w->p_buf      = p_buf;
	p_w->cap        = cap;
	p_w->bits       = 0u;
	p_w->b_overflow = false;
}

/* Append the low width bits (1..32) of value. */
static inline void pk_put(pk_writer_t *p_w, uint32_t value, uint8_t width)
{
	uint8_t i;

	if ((width > 32u) || (((uint32_t)p_w->bits + width) > ((uint32_t)p_w->cap * 8u)))
	{
		p_w->b_overflow = true;
		return;
	}
	for (i = 0u; i < width; i++)
	{
		if (((value >> i) & 1u) != 0u)
		{
			p_w->p_buf[p_w->bits >> 3] |= (uint8_t)(1u << (p_w->bits & 7u));
		}
		p_w->bits++;
	}
}

/* Frame length in whole bytes, 0 if anything overflowed. */
static inline uint16_t pk_writer_len(pk_writer_t const *p_w)
{
	return p_w->b_overflow ? 0u : (uint16_t)((p_w->bits + 7u) >> 3);
}

static inline void pk_reader_init(pk_reader_t *p_r, uint8_t const *p_buf, uint16_t len)
{
	p_r->p_buf   = p_buf;
	p_r->len     = len;
	p_r->bits    = 0u;
	p_r->b_short = false;
}

/* Read the next width bits (1..32); 0 and b_short once past the end. */
static inline uint32_t pk_take(pk_reader_t *p_r, uint8_t width)
{
	uint32_t value = 0u;
	uint8_t  i;

	if ((width > 32u) || (((uint32_t)p_r->bits + width) > ((uint32_t)p_r->len * 8u)))
	{
		p_r->b_short = true;
		return 0u;
	}
	for (i = 0u; i < width; i++)
	{
		if ((p_r->p_buf[p_r->bits >> 3] & (uint8_t)(1u << (p_r->bits & 7u))) != 0u)
		{
			value |= (1uL << i);
		}
		p_r->bits++;
	}
	return value;
}

#endif /* APP_PACK_H */
