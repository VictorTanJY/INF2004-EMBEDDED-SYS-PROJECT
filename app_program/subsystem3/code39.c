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

/* Classify one character's 9 elements: the 3 widest are wide. */
static code39_result_t classify(uint32_t const * p_w, uint16_t n, uint16_t first, bool b_rev,
                                uint16_t ratio_min_x10, uint16_t * p_pattern)
{
    uint32_t w[9];
    uint32_t sorted[9];
    uint32_t min_wide;
    uint32_t max_narrow;
    uint16_t pattern = 0u;
    uint8_t  wide_count = 0u;
    uint8_t  i;
    uint8_t  j;

    for (i = 0u; i < 9u; i++)
    {
        w[i]      = width_at(p_w, n, (uint16_t)(first + i), b_rev);
        sorted[i] = w[i];
    }
    for (i = 1u; i < 9u; i++)               /* insertion sort, descending */
    {
        uint32_t v = sorted[i];

        j = i;
        while ((j > 0u) && (sorted[j - 1u] < v))
        {
            sorted[j] = sorted[j - 1u];
            j--;
        }
        sorted[j] = v;
    }
    min_wide   = sorted[2];
    max_narrow = sorted[3];
    if ((max_narrow == 0u) || (((uint64_t)min_wide * 10u) < ((uint64_t)max_narrow * ratio_min_x10)))
    {
        return CODE39_ERR_RATIO;
    }
    for (i = 0u; i < 9u; i++)
    {
        uint16_t const bit = (w[i] >= min_wide) ? 1u : 0u;

        wide_count = (uint8_t)(wide_count + bit);
        pattern    = (uint16_t)((pattern << 1) | bit);
    }
    if (wide_count != 3u)
    {
        return CODE39_ERR_RATIO;            /* ties: cannot tell which are wide */
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
