/*
 *  code39.c
 *  Subsystem 3 -- Code 39 decoder.  See code39.h.
 */

#include <stddef.h>
#include "code39.h"

typedef struct
{
    char        c;
    char const *p_bits;         /* 9 elements, bar first: 1 = wide */
} code39_entry_t;

/* Code 39 character set (ISO/IEC 16388), element order b s b s b s b s b. */
static code39_entry_t const k_table[] =
{
    { '0', "000110100" }, { '1', "100100001" }, { '2', "001100001" }, { '3', "101100000" },
    { '4', "000110001" }, { '5', "100110000" }, { '6', "001110000" }, { '7', "000100101" },
    { '8', "100100100" }, { '9', "001100100" },
    { 'A', "100001001" }, { 'B', "001001001" }, { 'C', "101001000" }, { 'D', "000011001" },
    { 'E', "100011000" }, { 'F', "001011000" }, { 'G', "000001101" }, { 'H', "100001100" },
    { 'I', "001001100" }, { 'J', "000011100" }, { 'K', "100000011" }, { 'L', "001000011" },
    { 'M', "101000010" }, { 'N', "000010011" }, { 'O', "100010010" }, { 'P', "001010010" },
    { 'Q', "000000111" }, { 'R', "100000110" }, { 'S', "001000110" }, { 'T', "000010110" },
    { 'U', "110000001" }, { 'V', "011000001" }, { 'W', "111000000" }, { 'X', "010010001" },
    { 'Y', "110010000" }, { 'Z', "011010000" },
    { '-', "010000101" }, { '.', "110000100" }, { ' ', "011000100" }, { '*', "010010100" },
    { '$', "010101000" }, { '/', "010100010" }, { '+', "010001010" }, { '%', "000101010" }
};

#define TABLE_LEN   (sizeof(k_table) / sizeof(k_table[0]))

static uint16_t bits_of(char const * p_bits)
{
    uint16_t p = 0u;
    uint8_t  i;

    for (i = 0u; i < 9u; i++)
    {
        p = (uint16_t)((p << 1) | ((p_bits[i] == '1') ? 1u : 0u));
    }
    return p;
}

uint16_t code39_pattern(char c)
{
    size_t i;

    for (i = 0u; i < TABLE_LEN; i++)
    {
        if (k_table[i].c == c)
        {
            return bits_of(k_table[i].p_bits);
        }
    }
    return 0u;
}

char code39_char(uint16_t pattern)
{
    size_t i;

    for (i = 0u; i < TABLE_LEN; i++)
    {
        if (bits_of(k_table[i].p_bits) == pattern)
        {
            return k_table[i].c;
        }
    }
    return '\0';
}

/* Width of element k, read forwards or backwards. */
static uint32_t width_at(uint32_t const * p_w, uint16_t n, uint16_t k, bool b_rev)
{
    return b_rev ? p_w[n - 1u - k] : p_w[k];
}

/* The k-th widest (k = 0, 1, ...) of the elements of one kind: bars at
   even indexes 0..8, spaces at odd ones 1..7. */
static uint32_t kth_widest(uint32_t const * p_w, uint8_t first, uint8_t k, uint8_t * p_idx)
{
    uint32_t used = 0u;
    uint32_t best = 0u;
    uint8_t  r;
    uint8_t  i;

    *p_idx = first;
    for (r = 0u; r <= k; r++)
    {
        bool b_any = false;

        for (i = first; i < 9u; i += 2u)
        {
            if (((used & (1u << i)) == 0u) && (!b_any || (p_w[i] > best)))
            {
                best   = p_w[i];
                *p_idx = i;
                b_any  = true;
            }
        }
        used |= (1u << *p_idx);
    }
    return best;
}

/* Classify one character's 9 elements (7 Nov): bars and spaces are ranked
   SEPARATELY, the 2 widest bars and the widest space being wide, each
   standing out from the rest of its kind by ratio_min.  Under the sensor's
   spot black reads wider and white narrower, so ranking all 9 together
   could take a fat narrow bar for the wide space. */
static code39_result_t classify(uint32_t const * p_w, uint16_t n, uint16_t first, bool b_rev,
                                uint16_t ratio_min_x10, uint16_t * p_pattern)
{
    uint32_t w[9];
    uint32_t wide = 0u;
    uint16_t pattern = 0u;
    uint8_t  ib1;
    uint8_t  ib2;
    uint8_t  ib3;
    uint8_t  is1;
    uint8_t  is2;
    uint32_t b2;
    uint32_t b3;
    uint32_t s1;
    uint32_t s2;
    uint8_t  i;

    for (i = 0u; i < 9u; i++)
    {
        w[i] = width_at(p_w, n, (uint16_t)(first + i), b_rev);
    }
    (void)kth_widest(w, 0u, 0u, &ib1);
    b2 = kth_widest(w, 0u, 1u, &ib2);
    b3 = kth_widest(w, 0u, 2u, &ib3);           /* widest narrow bar   */
    s1 = kth_widest(w, 1u, 0u, &is1);
    s2 = kth_widest(w, 1u, 1u, &is2);           /* widest narrow space */
    if ((b3 != 0u) && (s2 != 0u) &&
        (((uint64_t)b2 * 10u) >= ((uint64_t)b3 * ratio_min_x10)) &&
        (((uint64_t)s1 * 10u) >= ((uint64_t)s2 * ratio_min_x10)))
    {
        wide = (1u << ib1) | (1u << ib2) | (1u << is1);
    }
    else
    {
        /* '$' '/' '+' '%': three wide spaces, no wide bar */
        uint8_t        is3;
        uint8_t        is4;
        uint32_t const s3 = kth_widest(w, 1u, 2u, &is3);
        uint32_t const s4 = kth_widest(w, 1u, 3u, &is4);

        if ((s4 == 0u) || (((uint64_t)s3 * 10u) < ((uint64_t)s4 * ratio_min_x10)))
        {
            return CODE39_ERR_RATIO;
        }
        (void)s2;
        wide = (1u << is1) | (1u << is2) | (1u << is3);
    }
    for (i = 0u; i < 9u; i++)
    {
        pattern = (uint16_t)((pattern << 1) | (((wide & (1u << i)) != 0u) ? 1u : 0u));
    }
    *p_pattern = pattern;
    return CODE39_OK;
}

static code39_result_t decode_dir(uint32_t const * p_w, uint16_t n, uint16_t ratio_min_x10,
                                  bool b_rev, char * p_text)
{
    uint16_t const chars = (uint16_t)((n + 1u) / 10u);
    uint16_t       c;
    uint16_t       out = 0u;

    for (c = 0u; c < chars; c++)
    {
        uint16_t        pattern = 0u;
        code39_result_t r = classify(p_w, n, (uint16_t)(c * 10u), b_rev, ratio_min_x10, &pattern);
        char            ch;

        if (r != CODE39_OK)
        {
            return r;
        }
        ch = code39_char(pattern);
        if (ch == '\0')
        {
            return CODE39_ERR_PATTERN;
        }
        if ((c == 0u) || (c == (chars - 1u)))
        {
            if (ch != '*')
            {
                return CODE39_ERR_START_STOP;
            }
        }
        else
        {
            if (ch == '*')
            {
                return CODE39_ERR_PATTERN;  /* '*' only frames the code */
            }
            p_text[out] = ch;
            out++;
        }
    }
    p_text[out] = '\0';
    return CODE39_OK;
}

code39_result_t code39_decode(uint32_t const * p_widths, uint16_t n, uint16_t ratio_min_x10,
                              char * p_text, bool * p_b_reversed)
{
    code39_result_t fwd;
    code39_result_t rev;

    p_text[0]     = '\0';
    *p_b_reversed = false;
    if (n < 9u)
    {
        return CODE39_NOT_BARCODE;
    }
    if (((n + 1u) / 10u) > CODE39_MAX_CHARS)
    {
        return CODE39_ERR_TOO_MANY;
    }
    if ((((n + 1u) % 10u) != 0u) || (n < 29u))
    {
        return CODE39_ERR_COUNT;            /* also "**" with no data */
    }

    fwd = decode_dir(p_widths, n, ratio_min_x10, false, p_text);
    if (fwd == CODE39_OK)
    {
        return CODE39_OK;
    }
    rev = decode_dir(p_widths, n, ratio_min_x10, true, p_text);
    if (rev == CODE39_OK)
    {
        *p_b_reversed = true;
        return CODE39_OK;
    }
    p_text[0] = '\0';
    /* Report the forward reason unless only the framing was wrong. */
    return (fwd == CODE39_ERR_START_STOP) ? rev : fwd;
}

code39_result_t code39_decode_one(uint32_t const * p_widths, uint16_t n, uint16_t ratio_min_x10,
                                  char * p_char, bool * p_b_reversed)
{
    char            text[CODE39_MAX_CHARS];
    code39_result_t r = code39_decode(p_widths, n, ratio_min_x10, text, p_b_reversed);

    *p_char = '\0';
    if (r != CODE39_OK)
    {
        return r;
    }
    if ((text[0] == '\0') || (text[1] != '\0'))
    {
        return CODE39_ERR_LENGTH;
    }
    *p_char = text[0];
    return CODE39_OK;
}

/* ---- one element pair lost at an end (7 Nov) ----------------------- */
/* A read one bar short at either end still decodes: the start '*' begins
   (narrow bar, wide space) and the stop '*' ends (narrow space, narrow
   bar); a missed end bar takes its neighbouring space into the quiet
   zone, so 2 elements are gone.  Each of the four ways of putting them
   back is tried (front / back, either reading direction); the read is
   kept only if every way that decodes gives the same character. */
code39_result_t code39_decode_one_repair(uint32_t const * p_widths, uint16_t n, uint16_t ratio_min_x10,
                                         char * p_char, bool * p_b_reversed)
{
    static uint32_t w[CODE39_MAX_CHARS * 10u];
    code39_result_t r = code39_decode_one(p_widths, n, ratio_min_x10, p_char, p_b_reversed);
    uint32_t        narrow = 0xFFFFFFFFu;
    uint32_t        widest = 0u;
    char            got = '\0';
    bool            b_got_rev = false;
    uint16_t        i;
    uint8_t         k;

    if ((r == CODE39_OK) || (r == CODE39_NOT_BARCODE) || ((n + 2u) > (CODE39_MAX_CHARS * 10u)) ||
        (((n + 3u) % 10u) != 0u))
    {
        return r;
    }
    for (i = 0u; i < n; i++)
    {
        narrow = (p_widths[i] < narrow) ? p_widths[i] : narrow;
        widest = (p_widths[i] > widest) ? p_widths[i] : widest;
    }
    for (k = 0u; k < 4u; k++)
    {
        bool const     b_front = (k < 2u);
        uint32_t const e0 = (k == 0u) ? narrow : ((k == 3u) ? widest : narrow);   /* first added */
        uint32_t const e1 = (k == 0u) ? widest : narrow;                          /* second added */
        char           ch;
        bool           b_rev;
        uint16_t       o = 0u;

        if (b_front)
        {
            w[o++] = e0;
            w[o++] = e1;
        }
        for (i = 0u; i < n; i++)
        {
            w[o++] = p_widths[i];
        }
        if (!b_front)
        {
            w[o++] = (k == 2u) ? narrow : widest;       /* k 2: space n, bar n; k 3: space W, bar n */
            w[o++] = narrow;
        }
        if (code39_decode_one(w, o, ratio_min_x10, &ch, &b_rev) == CODE39_OK)
        {
            if ((got != '\0') && (ch != got))
            {
                *p_char = '\0';
                return CODE39_ERR_PATTERN;              /* repairs disagree */
            }
            got       = ch;
            b_got_rev = b_rev;
        }
    }
    if (got == '\0')
    {
        *p_char = '\0';
        return r;
    }
    *p_char       = got;
    *p_b_reversed = b_got_rev;
    return CODE39_OK;
}
