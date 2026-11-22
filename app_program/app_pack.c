/** @file app_pack.c
 *
 *  @brief Minimum-bit data representation (Change Requirement 1): byte streams
 *  for MQTT frames, Code 39 symbols and table-driven record packing.
 *  Pure C; see app_pack.h for the rules.
 */

#include <string.h>
#include "app_pack.h"

/* Code 39 data characters; a symbol index is the position here + 1. */
#define PK_SYMBOLS "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ-. $/+%"

typedef uint8_t
    pk_check_sym_t[(PK_SYM_COUNT == (sizeof(PK_SYMBOLS) - 1u)) ? 1 : -1];

static char const g_symbols[] = PK_SYMBOLS;

static void     pk_word_put(uint32_t * p_w, uint32_t * p_pos, uint32_t value,
                            uint8_t width);
static uint32_t pk_word_take(uint32_t const * p_w, uint32_t * p_pos,
                             uint8_t width);
static uint32_t pk_member_u(void const * p_rec, app_pack_field_t const * p_f);
static int32_t  pk_member_s(void const * p_rec, app_pack_field_t const * p_f);
static void     pk_member_set(void * p_rec, app_pack_field_t const * p_f,
                              uint32_t value);

/* ---- Byte streams, LSB first ---------------------------------------- */
void app_pack_writer_init (app_pack_writer_t * p_w, uint8_t * p_buf,
                           uint16_t cap)
{
    uint16_t idx;

    for (idx = 0u; idx < cap; idx++)
    {
        p_buf[idx] = 0u;
    }

    p_w->p_buf      = p_buf;
    p_w->cap        = cap;
    p_w->bits       = 0u;
    p_w->b_overflow = false;
}

/** Append the low width bits (1..32) of value. */
void app_pack_put (app_pack_writer_t * p_w, uint32_t value, uint8_t width)
{
    uint8_t idx;

    /* Casts (uint32_t)p_w->bits, (uint32_t)p_w->cap: every uint16_t value
       fits. */
    if ((width > 32u)
        || (((uint32_t)p_w->bits + width) > ((uint32_t)p_w->cap * 8u)))
    {
        p_w->b_overflow = true;
    }
    else
    {
        for (idx = 0u; idx < width; idx++)
        {
            if (((value >> idx) & 1u) != 0u)
            {
                /* Cast (uint8_t)(1u << (p_w->bits & 7u)): a single bit 0..7 of
                   a byte. */
                p_w->p_buf[p_w->bits >> 3] |= (uint8_t)(1u << (p_w->bits & 7u));
            }

            p_w->bits++;
        }
    }
}

/** Frame length in whole bytes, 0 if anything overflowed. */
uint16_t app_pack_writer_len (app_pack_writer_t const * p_w)
{
    /* Cast (uint16_t)((p_w->bits + 7u) >> 3): the value is 0..8192. */
    return p_w->b_overflow ? 0u : (uint16_t)((p_w->bits + 7u) >> 3);
}

void app_pack_reader_init (app_pack_reader_t * p_r, uint8_t const * p_buf,
                           uint16_t len)
{
    p_r->p_buf   = p_buf;
    p_r->len     = len;
    p_r->bits    = 0u;
    p_r->b_short = false;
}

/** Read the next width bits (1..32); 0 and b_short once past the end. */
uint32_t app_pack_take (app_pack_reader_t * p_r, uint8_t width)
{
    uint32_t value = 0u;
    uint8_t  idx;

    /* Casts (uint32_t)p_r->bits, (uint32_t)p_r->len: every uint16_t value
       fits. */
    if ((width > 32u)
        || (((uint32_t)p_r->bits + width) > ((uint32_t)p_r->len * 8u)))
    {
        p_r->b_short = true;
    }
    else
    {
        for (idx = 0u; idx < width; idx++)
        {
            /* Cast (uint8_t)(1u << (p_r->bits & 7u)): a single bit 0..7 of a
               byte. */
            if ((p_r->p_buf[p_r->bits >> 3] & (uint8_t)(1u << (p_r->bits & 7u)))
                != 0u)
            {
                value |= (1uL << idx);
            }

            p_r->bits++;
        }
    }

    return value;
}

/* ---- Code 39 data character in 6 bits (index 1..43, 0 = none) ------- */
uint8_t app_pack_sym_index (char chr)
{
    uint8_t idx;
    uint8_t sym_index = 0u;

    for (idx = 0u; (idx < PK_SYM_COUNT) && (0u == sym_index); idx++)
    {
        if (g_symbols[idx] == chr)
        {
            /* Cast (uint8_t)(idx + 1u): a symbol index 1..PK_SYM_COUNT (43). */
            sym_index = (uint8_t)(idx + 1u);
        }
    }

    return sym_index;
}

char app_pack_sym_char (uint8_t sym_index)
{
    return ((sym_index >= 1u) && (sym_index <= PK_SYM_COUNT))
               ? g_symbols[sym_index - 1u]
               : '\0';
}

/* ---- Records: table-driven packing of a struct into words ------------ */
uint32_t app_pack_layout_bits (app_pack_field_t const * p_f, uint16_t n_fields)
{
    uint32_t bits = 0u;
    uint16_t idx;

    for (idx = 0u; idx < n_fields; idx++)
    {
        bits += p_f[idx].width;
    }

    return bits;
}

/** Pack every field of p_rec into p_words (nwords, cleared first). */
void app_pack_pack_rec (app_pack_field_t const * p_f, uint16_t n_fields,
                        void const * p_rec, uint32_t * p_words, uint16_t nwords)
{
    uint32_t pos = 0u;
    uint16_t idx;

    for (idx = 0u; idx < nwords; idx++)
    {
        p_words[idx] = 0u;
    }

    for (idx = 0u; idx < n_fields; idx++)
    {
        uint8_t const  width = p_f[idx].width;
        uint32_t const raw   = pk_member_u(p_rec, &p_f[idx]);
        uint32_t       value;

        switch (p_f[idx].kind)
        {
            case PK_S:
                value = app_pack_off_s(pk_member_s(p_rec, &p_f[idx]), width);
            break;
            case PK_W:
                value = raw;
            break;
            case PK_UN:
            {
                uint32_t const none =
                    (p_f[idx].size >= 4u)
                        ? 0xFFFFFFFFuL
                        : ((1uL << (8u * p_f[idx].size)) - 1uL);

                value = (raw == none) ? app_pack_mask(width)
                                      : app_pack_sat_u(raw, width);

                if ((raw != none) && (value == app_pack_mask(width)))
                {
                    value = app_pack_mask(width) - 1u;
                }
            }
            break;
            case PK_SYM:
                /* Cast (uint8_t)raw: a PK_SYM member is a 1-byte char; raw
                   holds that byte. */
                value = app_pack_sym_index((uint8_t)raw);
            break;
            default: /* PK_U */
                value = app_pack_sat_u(raw, width);
            break;
        }

        pk_word_put(p_words, &pos, value, width);
    }
}

/** Unpack p_words into the fields of p_rec (other bytes are left as they are).
 */
void app_pack_unpack_rec (app_pack_field_t const * p_f, uint16_t n_fields,
                          uint32_t const * p_words, void * p_rec)
{
    uint32_t pos = 0u;
    uint16_t idx;

    for (idx = 0u; idx < n_fields; idx++)
    {
        uint8_t const  width = p_f[idx].width;
        uint32_t const value = pk_word_take(p_words, &pos, width);
        uint32_t       raw;

        switch (p_f[idx].kind)
        {
            case PK_S:
                /* Cast (uint32_t)app_pack_unoff_s(value ...): two's complement
                   bit pattern of a value of width bits. */
                raw = (uint32_t)app_pack_unoff_s(value, width);
            break;
            case PK_UN:
                raw = (value == app_pack_mask(width)) ? 0xFFFFFFFFuL : value;
            break;
            case PK_SYM:
                /* Cast (uint8_t)value: a PK_SYM field is 6 bits (0..63). */
                raw = app_pack_sym_char((uint8_t)value);
            break;
            default: /* PK_U, PK_W */
                raw = value;
            break;
        }

        pk_member_set(p_rec, &p_f[idx], raw);
    }
}

/** Write value into width bits at bit position *p_pos of the word array p_w,
    then advance *p_pos. */
static void pk_word_put (uint32_t * p_w, uint32_t * p_pos, uint32_t value,
                         uint8_t width)
{
    uint32_t const word = *p_pos >> 5;
    uint32_t const bit  = *p_pos & 31u;

    value &= app_pack_mask(width);
    p_w[word] |= value << bit;

    if ((bit + width) > 32u)
    {
        p_w[word + 1u] |= value >> (32u - bit);
    }

    *p_pos += width;
}

/** Read width bits at bit position *p_pos of the word array p_w, then advance
 *p_pos. */
static uint32_t pk_word_take (uint32_t const * p_w, uint32_t * p_pos,
                              uint8_t width)
{
    uint32_t const word  = *p_pos >> 5;
    uint32_t const bit   = *p_pos & 31u;
    uint32_t       value = p_w[word] >> bit;

    if ((bit + width) > 32u)
    {
        value |= p_w[word + 1u] << (32u - bit);
    }

    *p_pos += width;
    return value & app_pack_mask(width);
}

/** Read a member of size 1, 2 or 4 as unsigned / signed. */
static uint32_t pk_member_u (void const * p_rec, app_pack_field_t const * p_f)
{
    /* Cast (uint8_t const *)p_rec: byte view of the record; the member is read
       with memcpy. */
    uint8_t const * p_byte = (uint8_t const *)p_rec + p_f->offset;
    uint32_t        value  = 0u;

    if (1u == p_f->size)
    {
        uint8_t byte;

        (void)memcpy(&byte, p_byte, 1u);
        value = byte;
    }
    else if (2u == p_f->size)
    {
        uint16_t half;

        (void)memcpy(&half, p_byte, 2u);
        value = half;
    }
    else
    {
        (void)memcpy(&value, p_byte, 4u);
    }

    return value;
}

/** Value of the signed record member described by p_f, sign-extended to 32
    bits. */
static int32_t pk_member_s (void const * p_rec, app_pack_field_t const * p_f)
{
    uint32_t const raw = pk_member_u(p_rec, p_f);
    int32_t        value;

    if (1u == p_f->size)
    {
        /* Casts (int8_t)(uint8_t)raw: the member is 1 byte: its bit pattern
           read back as signed; (uint8_t)raw: the member is 1 byte: raw holds 8
           bits. */
        value = (int8_t)(uint8_t)raw;
    }
    else if (2u == p_f->size)
    {
        /* Casts (int16_t)(uint16_t)raw: the member is 2 bytes: its bit pattern
           read back as signed; (uint16_t)raw: the member is 2 bytes: raw holds
           16 bits. */
        value = (int16_t)(uint16_t)raw;
    }
    else
    {
        /* Cast (int32_t)raw: the member is 4 bytes: its bit pattern read back
           as signed. */
        value = (int32_t)raw;
    }

    return value;
}

/** Store value into the record member described by p_f (1, 2 or 4 bytes). */
static void pk_member_set (void * p_rec, app_pack_field_t const * p_f,
                           uint32_t value)
{
    /* Cast (uint8_t *)p_rec: byte view of the record; the member is written
       with memcpy. */
    uint8_t * p_byte = (uint8_t *)p_rec + p_f->offset;

    if (1u == p_f->size)
    {
        /* Cast (uint8_t)value: the member is 1 byte: the field width is at most
           8 bits. */
        uint8_t const byte = (uint8_t)value;

        (void)memcpy(p_byte, &byte, 1u);
    }
    else if (2u == p_f->size)
    {
        /* Cast (uint16_t)value: the member is 2 bytes: the field width is at
           most 16 bits. */
        uint16_t const half = (uint16_t)value;

        (void)memcpy(p_byte, &half, 2u);
    }
    else
    {
        (void)memcpy(p_byte, &value, 4u);
    }
}

/*** end of file ***/
