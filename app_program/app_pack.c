/*
 *	app_pack.c
 *	Minimum-bit data representation (Change Requirement 1): byte streams
 *	for MQTT frames, Code 39 symbols and table-driven record packing.
 *	Pure C; see app_pack.h for the rules.
 */

#include <string.h>
#include "app_pack.h"

/* Code 39 data characters; a symbol index is the position here + 1. */
static char const k_sym[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ-. $/+%";

typedef char pk_check_sym[((sizeof(k_sym) - 1u) == PK_SYM_COUNT) ? 1 : -1];

/* ---- Byte streams, LSB first ---------------------------------------- */
void pk_writer_init(pk_writer_t *p_w, uint8_t *p_buf, uint16_t cap)
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
void pk_put(pk_writer_t *p_w, uint32_t value, uint8_t width)
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
uint16_t pk_writer_len(pk_writer_t const *p_w)
{
	return p_w->b_overflow ? 0u : (uint16_t)((p_w->bits + 7u) >> 3);
}

void pk_reader_init(pk_reader_t *p_r, uint8_t const *p_buf, uint16_t len)
{
	p_r->p_buf   = p_buf;
	p_r->len     = len;
	p_r->bits    = 0u;
	p_r->b_short = false;
}

/* Read the next width bits (1..32); 0 and b_short once past the end. */
uint32_t pk_take(pk_reader_t *p_r, uint8_t width)
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

/* ---- Code 39 data character in 6 bits (index 1..43, 0 = none) ------- */
uint8_t pk_sym_index(char c)
{
	uint8_t i;

	for (i = 0u; i < PK_SYM_COUNT; i++)
	{
		if (k_sym[i] == c)
		{
			return (uint8_t)(i + 1u);
		}
	}
	return 0u;
}

char pk_sym_char(uint8_t index)
{
	return ((index >= 1u) && (index <= PK_SYM_COUNT)) ? k_sym[index - 1u] : '\0';
}

/* ---- Records: table-driven packing of a struct into words ------------ */
uint32_t pk_layout_bits(pk_field_t const *p_f, uint16_t n)
{
	uint32_t bits = 0u;
	uint16_t i;

	for (i = 0u; i < n; i++)
	{
		bits += p_f[i].width;
	}
	return bits;
}

static void pk_word_put(uint32_t *p_w, uint32_t *p_pos, uint32_t v, uint8_t width)
{
	uint32_t const word = *p_pos >> 5;
	uint32_t const bit  = *p_pos & 31u;

	v &= pk_mask(width);
	p_w[word] |= v << bit;
	if ((bit + width) > 32u)
	{
		p_w[word + 1u] |= v >> (32u - bit);
	}
	*p_pos += width;
}

static uint32_t pk_word_take(uint32_t const *p_w, uint32_t *p_pos, uint8_t width)
{
	uint32_t const word = *p_pos >> 5;
	uint32_t const bit  = *p_pos & 31u;
	uint32_t       v    = p_w[word] >> bit;

	if ((bit + width) > 32u)
	{
		v |= p_w[word + 1u] << (32u - bit);
	}
	*p_pos += width;
	return v & pk_mask(width);
}

/* Read a member of size 1, 2 or 4 as unsigned / signed. */
static uint32_t pk_member_u(void const *p_rec, pk_field_t const *p_f)
{
	uint8_t const *p = (uint8_t const *)p_rec + p_f->offset;
	uint32_t v = 0u;

	if (p_f->size == 1u)
	{
		uint8_t b;

		(void)memcpy(&b, p, 1u);
		v = b;
	}
	else if (p_f->size == 2u)
	{
		uint16_t h;

		(void)memcpy(&h, p, 2u);
		v = h;
	}
	else
	{
		(void)memcpy(&v, p, 4u);
	}
	return v;
}

static int32_t pk_member_s(void const *p_rec, pk_field_t const *p_f)
{
	uint32_t const u = pk_member_u(p_rec, p_f);

	if (p_f->size == 1u)
	{
		return (int32_t)(int8_t)(uint8_t)u;
	}
	if (p_f->size == 2u)
	{
		return (int32_t)(int16_t)(uint16_t)u;
	}
	return (int32_t)u;
}

static void pk_member_set(void *p_rec, pk_field_t const *p_f, uint32_t v)
{
	uint8_t *p = (uint8_t *)p_rec + p_f->offset;

	if (p_f->size == 1u)
	{
		uint8_t const b = (uint8_t)v;

		(void)memcpy(p, &b, 1u);
	}
	else if (p_f->size == 2u)
	{
		uint16_t const h = (uint16_t)v;

		(void)memcpy(p, &h, 2u);
	}
	else
	{
		(void)memcpy(p, &v, 4u);
	}
}

/* Pack every field of p_rec into p_words (nwords, cleared first). */
void pk_pack_rec(pk_field_t const *p_f, uint16_t n, void const *p_rec,
			       uint32_t *p_words, uint16_t nwords)
{
	uint32_t pos = 0u;
	uint16_t i;

	for (i = 0u; i < nwords; i++)
	{
		p_words[i] = 0u;
	}
	for (i = 0u; i < n; i++)
	{
		uint8_t const  w = p_f[i].width;
		uint32_t const u = pk_member_u(p_rec, &p_f[i]);
		uint32_t       v;

		switch (p_f[i].kind)
		{
			case PK_S:
				v = pk_off_s(pk_member_s(p_rec, &p_f[i]), w);
				break;
			case PK_W:
				v = u;
				break;
			case PK_UN:
			{
				uint32_t const none = (p_f[i].size >= 4u) ? 0xFFFFFFFFuL
						    : ((1uL << (8u * p_f[i].size)) - 1uL);

				v = (u == none) ? pk_mask(w) : pk_sat_u(u, w);
				if ((u != none) && (v == pk_mask(w)))
				{
					v = pk_mask(w) - 1u;
				}
				break;
			}
			case PK_SYM:
				v = pk_sym_index((char)(uint8_t)u);
				break;
			default:	/* PK_U */
				v = pk_sat_u(u, w);
				break;
		}
		pk_word_put(p_words, &pos, v, w);
	}
}

/* Unpack p_words into the fields of p_rec (other bytes are left as they are). */
void pk_unpack_rec(pk_field_t const *p_f, uint16_t n, uint32_t const *p_words,
				 void *p_rec)
{
	uint32_t pos = 0u;
	uint16_t i;

	for (i = 0u; i < n; i++)
	{
		uint8_t const  w = p_f[i].width;
		uint32_t const v = pk_word_take(p_words, &pos, w);
		uint32_t       u;

		switch (p_f[i].kind)
		{
			case PK_S:
				u = (uint32_t)pk_unoff_s(v, w);
				break;
			case PK_UN:
				u = (v == pk_mask(w)) ? 0xFFFFFFFFuL : v;
				break;
			case PK_SYM:
				u = (uint8_t)pk_sym_char((uint8_t)v);
				break;
			default:	/* PK_U, PK_W */
				u = v;
				break;
		}
		pk_member_set(p_rec, &p_f[i], u);
	}
}
