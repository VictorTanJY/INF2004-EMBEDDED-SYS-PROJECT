/*
 *  code39.c
 *  Subsystem 3 -- Code 39 decoder.  See code39.h.
 */

#include <stddef.h>
#include "code39.h"

typedef struct
{
    char        chr;
    char const *p_bits;         /* 9 elements, bar first: 1 = wide */
} code39_entry_t;

/* Code 39 character set (ISO/IEC 16388), element order b s b s b s b s b. */
static code39_entry_t const g_code39_table[] =
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

#define TABLE_LEN   (sizeof(g_code39_table) / sizeof(g_code39_table[0]))

static uint16_t bits_of(char const * p_bits)
{
    uint16_t pattern = 0u;
    uint8_t  idx;

    for (idx = 0u; idx < 9u; idx++)
    {
        pattern = (uint16_t)((pattern << 1) | ((p_bits[idx] == '1') ? 1u : 0u));
    }
    return pattern;
}

uint16_t code39_pattern(char chr)
{
    size_t idx;

    for (idx = 0u; idx < TABLE_LEN; idx++)
    {
        if (g_code39_table[idx].chr == chr)
        {
            return bits_of(g_code39_table[idx].p_bits);
        }
    }
    return 0u;
}

char code39_char(uint16_t pattern)
{
    size_t idx;

    for (idx = 0u; idx < TABLE_LEN; idx++)
    {
        if (bits_of(g_code39_table[idx].p_bits) == pattern)
        {
            return g_code39_table[idx].chr;
        }
    }
    return '\0';
}

/* Width of element k, read forwards or backwards. */
static uint32_t width_at(uint32_t const * p_w, uint16_t n_elems, uint16_t pos, bool b_rev)
{
    return b_rev ? p_w[n_elems - 1u - pos] : p_w[pos];
}

/* The k-th widest (k = 0, 1, ...) of the elements of one kind: bars at
   even indexes 0..8, spaces at odd ones 1..7. */
static uint32_t kth_widest(uint32_t const * p_w, uint8_t first, uint8_t rank, uint8_t * p_idx)
{
    uint32_t used = 0u;
    uint32_t best = 0u;
    uint8_t  rank_idx;
    uint8_t  idx;

    *p_idx = first;
    for (rank_idx = 0u; rank_idx <= rank; rank_idx++)
    {
        bool b_any = false;

        for (idx = first; idx < 9u; idx += 2u)
        {
            if (((used & (1u << idx)) == 0u) && (!b_any || (p_w[idx] > best)))
            {
                best   = p_w[idx];
                *p_idx = idx;
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
static code39_result_t classify(uint32_t const * p_w, uint16_t n_elems, uint16_t first, bool b_rev,
                                uint16_t ratio_min_x10, uint16_t * p_pattern)
{
    uint32_t widths[9];
    uint32_t wide = 0u;
    uint16_t pattern = 0u;
    uint8_t  ib1;
    uint8_t  ib2;
    uint8_t  ib3;
    uint8_t  is1;
    uint8_t  is2;
    uint32_t bar_w2;
    uint32_t bar_w3;
    uint32_t space_w1;
    uint32_t space_w2;
    uint8_t  idx;

    for (idx = 0u; idx < 9u; idx++)
    {
        widths[idx] = width_at(p_w, n_elems, (uint16_t)(first + idx), b_rev);
    }
    (void)kth_widest(widths, 0u, 0u, &ib1);
    bar_w2 = kth_widest(widths, 0u, 1u, &ib2);
    bar_w3 = kth_widest(widths, 0u, 2u, &ib3);           /* widest narrow bar   */
    space_w1 = kth_widest(widths, 1u, 0u, &is1);
    space_w2 = kth_widest(widths, 1u, 1u, &is2);           /* widest narrow space */
    if ((bar_w3 != 0u) && (space_w2 != 0u) &&
        (((uint64_t)bar_w2 * 10u) >= ((uint64_t)bar_w3 * ratio_min_x10)) &&
        (((uint64_t)space_w1 * 10u) >= ((uint64_t)space_w2 * ratio_min_x10)))
    {
        wide = (1u << ib1) | (1u << ib2) | (1u << is1);
    }
    else
    {
        /* '$' '/' '+' '%': three wide spaces, no wide bar.  9 Nov: only if
           the bars really are all alike (widest < ratio_min x narrowest);
           a faint read whose wide bars came out too thin otherwise looked
           like one of these (L read as %, R as $). */
        uint8_t        is3;
        uint8_t        is4;
        uint8_t        ib5;
        uint32_t const space_w3 = kth_widest(widths, 1u, 2u, &is3);
        uint32_t const space_w4 = kth_widest(widths, 1u, 3u, &is4);
        uint32_t const bar_w1 = kth_widest(widths, 0u, 0u, &ib1);
        uint32_t const bar_w5 = kth_widest(widths, 0u, 4u, &ib5);

        if ((space_w4 == 0u) || (((uint64_t)space_w3 * 10u) < ((uint64_t)space_w4 * ratio_min_x10)) ||
            (((uint64_t)bar_w1 * 10u) >= ((uint64_t)bar_w5 * ratio_min_x10)))
        {
            return CODE39_ERR_RATIO;
        }
        (void)space_w2;
        wide = (1u << is1) | (1u << is2) | (1u << is3);
    }
    for (idx = 0u; idx < 9u; idx++)
    {
        pattern = (uint16_t)((pattern << 1) | (((wide & (1u << idx)) != 0u) ? 1u : 0u));
    }
    *p_pattern = pattern;
    return CODE39_OK;
}

static code39_result_t decode_dir(uint32_t const * p_w, uint16_t n_elems, uint16_t ratio_min_x10,
                                  bool b_rev, char * p_text)
{
    uint16_t const chars = (uint16_t)((n_elems + 1u) / 10u);
    uint16_t       char_idx;
    uint16_t       out = 0u;

    for (char_idx = 0u; char_idx < chars; char_idx++)
    {
        uint16_t        pattern = 0u;
        code39_result_t result = classify(p_w, n_elems, (uint16_t)(char_idx * 10u), b_rev, ratio_min_x10, &pattern);
        char            chr;

        if (result != CODE39_OK)
        {
            return result;
        }
        chr = code39_char(pattern);
        if (chr == '\0')
        {
            return CODE39_ERR_PATTERN;
        }
        if ((char_idx == 0u) || (char_idx == (chars - 1u)))
        {
            if (chr != '*')
            {
                return CODE39_ERR_START_STOP;
            }
        }
        else
        {
            if (chr == '*')
            {
                return CODE39_ERR_PATTERN;  /* '*' only frames the code */
            }
            p_text[out] = chr;
            out++;
        }
    }
    p_text[out] = '\0';
    return CODE39_OK;
}

code39_result_t code39_decode(uint32_t const * p_widths, uint16_t n_elems, uint16_t ratio_min_x10,
                              char * p_text, bool * pb_reversed)
{
    code39_result_t fwd;
    code39_result_t rev;

    p_text[0]     = '\0';
    *pb_reversed = false;
    if (n_elems < 9u)
    {
        return CODE39_NOT_BARCODE;
    }
    if (((n_elems + 1u) / 10u) > CODE39_MAX_CHARS)
    {
        return CODE39_ERR_TOO_MANY;
    }
    if ((((n_elems + 1u) % 10u) != 0u) || (n_elems < 29u))
    {
        return CODE39_ERR_COUNT;            /* also "**" with no data */
    }

    fwd = decode_dir(p_widths, n_elems, ratio_min_x10, false, p_text);
    if (fwd == CODE39_OK)
    {
        return CODE39_OK;
    }
    rev = decode_dir(p_widths, n_elems, ratio_min_x10, true, p_text);
    if (rev == CODE39_OK)
    {
        *pb_reversed = true;
        return CODE39_OK;
    }
    p_text[0] = '\0';
    /* Report the forward reason unless only the framing was wrong. */
    return (fwd == CODE39_ERR_START_STOP) ? rev : fwd;
}

code39_result_t code39_decode_one(uint32_t const * p_widths, uint16_t n_elems, uint16_t ratio_min_x10,
                                  char * p_char, bool * pb_reversed)
{
    char            text[CODE39_MAX_CHARS];
    code39_result_t result = code39_decode(p_widths, n_elems, ratio_min_x10, text, pb_reversed);

    *p_char = '\0';
    if (result != CODE39_OK)
    {
        return result;
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
code39_result_t code39_decode_one_repair(uint32_t const * p_widths, uint16_t n_elems, uint16_t ratio_min_x10,
                                         char * p_char, bool * pb_reversed)
{
    static uint32_t widths[CODE39_MAX_CHARS * 10u];
    code39_result_t result = code39_decode_one(p_widths, n_elems, ratio_min_x10, p_char, pb_reversed);
    uint32_t        narrow = 0xFFFFFFFFu;
    uint32_t        widest = 0u;
    char            got = '\0';
    bool            b_got_rev = false;
    uint16_t        idx;
    uint8_t         kdx;

    if ((result == CODE39_OK) || (result == CODE39_NOT_BARCODE) || ((n_elems + 2u) > (CODE39_MAX_CHARS * 10u)) ||
        (((n_elems + 3u) % 10u) != 0u))
    {
        return result;
    }
    for (idx = 0u; idx < n_elems; idx++)
    {
        narrow = (p_widths[idx] < narrow) ? p_widths[idx] : narrow;
        widest = (p_widths[idx] > widest) ? p_widths[idx] : widest;
    }
    for (kdx = 0u; kdx < 4u; kdx++)
    {
        bool const     b_front = (kdx < 2u);
        uint32_t const add_first = (kdx == 0u) ? narrow : ((kdx == 3u) ? widest : narrow);   /* first added */
        uint32_t const add_second = (kdx == 0u) ? widest : narrow;                          /* second added */
        char           chr;
        bool           b_rev;
        uint16_t       out_n = 0u;

        if (b_front)
        {
            widths[out_n++] = add_first;
            widths[out_n++] = add_second;
        }
        for (idx = 0u; idx < n_elems; idx++)
        {
            widths[out_n++] = p_widths[idx];
        }
        if (!b_front)
        {
            widths[out_n++] = (kdx == 2u) ? narrow : widest;       /* k 2: space n, bar n; k 3: space W, bar n */
            widths[out_n++] = narrow;
        }
        if (code39_decode_one(widths, out_n, ratio_min_x10, &chr, &b_rev) == CODE39_OK)
        {
            if ((got != '\0') && (chr != got))
            {
                *p_char = '\0';
                return CODE39_ERR_PATTERN;              /* repairs disagree */
            }
            got       = chr;
            b_got_rev = b_rev;
        }
    }
    if (got == '\0')
    {
        *p_char = '\0';
        return result;
    }
    *p_char       = got;
    *pb_reversed = b_got_rev;
    return CODE39_OK;
}

/* ---- best match for one data character (8 Nov) --------------------- */
/* On the robot 3 mm elements are at the limit of the sensor's spot: a
   speck splits one bar in three (a ~1 mm "gap"), or a thin bar and the
   thin space beside it are lost in a neighbour.  Either way the count is
   no longer 29 and the exact decoders reject.  Here the read is lined up
   (dynamic programming) against the 29 printed elements of *X*.  Each
   step pairs:
     1 measured with 1 printed,
     3 measured with 1 printed (split; the 3 add up),
     1 measured with 3 printed (merge; the 3 add up),
   so bars stay bars and spaces stay spaces.  The first or last two
   printed elements (end bar + its space) may also be lost.  Each step
   costs its width error squared, in hundredths of a narrow element; a
   split, merge or lost end adds BEST_EDIT.  Only the defects the sensor
   makes are allowed: a split's middle is a speck (thinner than
   BEST_SPECK_PCT % of a narrow element of its kind), and a speck may
   not stand for a printed element; a merge loses a narrow element. */

#define BEST_T          29u         /* elements of *X*                  */
#define BEST_N_MIN      (BEST_T - (2u * BEST_MAX_EDITS))    /* 25 */
#define BEST_N_MAX      (BEST_T + (2u * BEST_MAX_EDITS))    /* 33 */
#define BEST_BAND       (2u * BEST_MAX_EDITS)   /* |measured - printed| index */
#define BEST_EDIT       2500u       /* = a 0.5 narrow width error        */
#define BEST_MAX_EDITS  2u          /* more defects than this: no read   */
#define BEST_ELEM_MAX   1600u       /* mean error <= 0.4 narrow per step */
#define BEST_MARGIN_X10 20u         /* runner-up must cost >= 2 x best   */
#define BEST_SPECK_PCT  50u         /* speck: under half a narrow       */
#define BEST_INF        0x3FFFFFFFu
#define BEST_BOUND_MAX  (((BEST_ELEM_MAX * BEST_T) * BEST_MARGIN_X10) / 10u)

static uint32_t median_of(uint32_t * p_v, uint16_t n_elems)
{
    uint16_t idx;
    uint16_t jdx;

    for (idx = 1u; idx < n_elems; idx++)            /* insertion sort, n <= 33 */
    {
        uint32_t const value = p_v[idx];

        for (jdx = idx; (jdx > 0u) && (p_v[jdx - 1u] > value); jdx--)
        {
            p_v[jdx] = p_v[jdx - 1u];
        }
        p_v[jdx] = value;
    }
    return (n_elems == 0u) ? 0u : p_v[n_elems / 2u];
}

/* This read's widths of narrow and wide bars and spaces: the median
   narrow and wide of each kind, split half-way between the median of
   all elements (most are narrow) and the median of the widest third. */
static void best_levels(uint32_t const * p_w, uint16_t n_elems, uint32_t * p_lv)
{
    static uint32_t sorted[BEST_N_MAX];   /* n <= BEST_N_MAX here */
    uint32_t        nar;
    uint32_t        wid;
    uint32_t        thr;
    uint16_t        mid;
    uint16_t        idx;
    uint8_t         kind;
    uint8_t         wide;

    for (idx = 0u; idx < n_elems; idx++)
    {
        sorted[idx] = p_w[idx];
    }
    nar = median_of(sorted, n_elems);
    wid = sorted[n_elems - 1u - ((n_elems / 3u) / 2u)];   /* median of the widest third (sorted) */
    thr = (nar + wid) / 2u;
    for (kind = 0u; kind < 2u; kind++)
    {
        for (wide = 0u; wide < 2u; wide++)
        {
            mid = 0u;
            for (idx = kind; idx < n_elems; idx += 2u)
            {
                if ((p_w[idx] >= thr) == (wide != 0u))
                {
                    sorted[mid++] = p_w[idx];
                }
            }
            p_lv[(kind * 2u) + wide] = (mid > 0u) ? median_of(sorted, mid) : ((wide != 0u) ? wid : nar);
        }
    }
}

/* Width error squared in hundredths of a narrow; inv = 100 x 65536 / narrow.
   Multiply and shift only: the Cortex-M0+ has no 64-bit divide, and this
   runs about 50 000 times per read. */
static uint32_t sq_err(uint32_t meas, uint32_t expect, uint32_t inv)
{
    uint32_t const diff = (meas > expect) ? (meas - expect) : (expect - meas);
    uint32_t       scaled;

    if (diff > 0x3FFFFu)
    {
        return 1000000u;
    }
    scaled = (diff * inv) >> 16;
    return (scaled > 1000u) ? 1000000u : (scaled * scaled);
}

static bool in_band(uint16_t idx, uint16_t jdx)
{
    uint16_t const diff = (idx > jdx) ? (uint16_t)(idx - jdx) : (uint16_t)(jdx - idx);

    return (diff <= BEST_BAND);
}

static bool is_speck(uint32_t width, uint16_t idx, uint32_t const * p_lv)
{
    uint32_t const narrow = p_lv[((idx % 2u) != 0u) ? 2u : 0u];

    return ((uint64_t)width * 100u) < ((uint64_t)narrow * BEST_SPECK_PCT);
}

/* Cost of lining p_w up with the printed widths p_e (BEST_T of them);
   p_wide: printed element wide; p_speck: measured element is a speck.
   14 Nov: gives up (BEST_INF) once every open path costs bound or more:
   such a fit can change neither the best nor the runner-up (a car read
   took over 50 ms and tripped the line task's heartbeat). */
static uint32_t best_fit(uint32_t const * p_w, uint16_t n_elems, uint32_t const * p_e, bool const * p_wide,
                         bool const * p_speck, uint32_t inv, uint32_t bound)
{
    static uint32_t cost_tbl[BEST_N_MAX + 1u][BEST_T + 1u];
    static uint8_t  edit_tbl[BEST_N_MAX + 1u][BEST_T + 1u];
    static uint32_t rmin[BEST_N_MAX + 1u];     /* smallest cost in each row */
    uint32_t        best;
    uint16_t        idx;
    uint16_t        jdx;

    /* Every defect moves the two indexes apart by 2, so only a band of
       cells can be reached; only those are used. */
    for (idx = 0u; idx <= n_elems; idx++)
    {
        for (jdx = (idx > BEST_BAND) ? (uint16_t)(idx - BEST_BAND) : 0u; (jdx <= BEST_T) && (jdx <= (idx + BEST_BAND)); jdx++)
        {
            cost_tbl[idx][jdx] = BEST_INF;
            edit_tbl[idx][jdx] = 0u;
        }
        rmin[idx] = BEST_INF;
    }
    cost_tbl[0][0] = 0u;
    rmin[0]  = 0u;
    cost_tbl[0][2] = BEST_EDIT;               /* first bar and space lost */
    edit_tbl[0][2] = 1u;
    for (idx = 0u; idx < n_elems; idx++)
    {
        /* Every path to the end passes a filled cell of row i, i + 1 or
           i + 2 (a step moves on 1 or 3 measured elements, and rows
           before i are done); costs only grow along a path. */
        uint32_t c_open = rmin[idx];

        c_open = (((idx + 1u) <= n_elems) && (rmin[idx + 1u] < c_open)) ? rmin[idx + 1u] : c_open;
        c_open = (((idx + 2u) <= n_elems) && (rmin[idx + 2u] < c_open)) ? rmin[idx + 2u] : c_open;
        if (c_open >= bound)
        {
            return BEST_INF;
        }
        for (jdx = (idx > BEST_BAND) ? (uint16_t)(idx - BEST_BAND) : 0u; (jdx < BEST_T) && (jdx <= (idx + BEST_BAND)); jdx++)
        {
            uint32_t const cost_here = cost_tbl[idx][jdx];
            uint8_t const  edits = edit_tbl[idx][jdx];

            if (cost_here >= BEST_INF)
            {
                continue;
            }
            if (!p_speck[idx])
            {   /* 1 : 1 */
                uint32_t const cost = cost_here + sq_err(p_w[idx], p_e[jdx], inv);

                if (cost < cost_tbl[idx + 1u][jdx + 1u])
                {
                    cost_tbl[idx + 1u][jdx + 1u] = cost;
                    rmin[idx + 1u] = (cost < rmin[idx + 1u]) ? cost : rmin[idx + 1u];
                    edit_tbl[idx + 1u][jdx + 1u] = edits;
                }
            }
            if ((edits < BEST_MAX_EDITS) && ((idx + 3u) <= n_elems) && in_band((uint16_t)(idx + 3u), (uint16_t)(jdx + 1u)) &&
                p_speck[idx + 1u])
            {   /* 3 measured : 1 printed */
                uint32_t const cost = cost_here + BEST_EDIT +
                                   sq_err(p_w[idx] + p_w[idx + 1u] + p_w[idx + 2u], p_e[jdx], inv);

                if (cost < cost_tbl[idx + 3u][jdx + 1u])
                {
                    cost_tbl[idx + 3u][jdx + 1u] = cost;
                    rmin[idx + 3u] = (cost < rmin[idx + 3u]) ? cost : rmin[idx + 3u];
                    edit_tbl[idx + 3u][jdx + 1u] = (uint8_t)(edits + 1u);
                }
            }
            if ((edits < BEST_MAX_EDITS) && ((jdx + 3u) <= BEST_T) && in_band((uint16_t)(idx + 1u), (uint16_t)(jdx + 3u)) &&
                !p_wide[jdx + 1u])
            {   /* 1 measured : 3 printed */
                uint32_t const cost = cost_here + BEST_EDIT +
                                   sq_err(p_w[idx], p_e[jdx] + p_e[jdx + 1u] + p_e[jdx + 2u], inv);

                if (cost < cost_tbl[idx + 1u][jdx + 3u])
                {
                    cost_tbl[idx + 1u][jdx + 3u] = cost;
                    rmin[idx + 1u] = (cost < rmin[idx + 1u]) ? cost : rmin[idx + 1u];
                    edit_tbl[idx + 1u][jdx + 3u] = (uint8_t)(edits + 1u);
                }
            }
        }
    }
    best = in_band(n_elems, BEST_T) ? cost_tbl[n_elems][BEST_T] : BEST_INF;
    if (in_band(n_elems, (uint16_t)(BEST_T - 2u)) && (edit_tbl[n_elems][BEST_T - 2u] < BEST_MAX_EDITS) &&
        (cost_tbl[n_elems][BEST_T - 2u] < BEST_INF) &&
        ((cost_tbl[n_elems][BEST_T - 2u] + BEST_EDIT) < best))
    {
        best = cost_tbl[n_elems][BEST_T - 2u] + BEST_EDIT;      /* last space and bar lost */
    }
    return best;
}

code39_result_t code39_decode_one_best(uint32_t const * p_widths, uint16_t n_elems,
                                       char * p_char, bool * pb_reversed)
{
    uint32_t       levels[4];               /* bar n, bar W, space n, space W */
    uint32_t       expect[BEST_T];
    bool           b_wide[BEST_T];
    bool           b_speck[BEST_N_MAX];      /* measured element is a speck */
    uint16_t const star = code39_pattern('*');
    uint32_t       nar;
    uint32_t       inv;
    uint32_t       best_cost = BEST_INF;       /* best cost and its character */
    uint32_t       next_cost = BEST_INF;       /* best cost of any other character */
    char           ch1 = '\0';
    bool           b_rev1 = false;
    size_t         entry;
    uint8_t        dir;

    *p_char       = '\0';
    *pb_reversed = false;
    if (n_elems < 9u)
    {
        return CODE39_NOT_BARCODE;
    }
    if ((n_elems < BEST_N_MIN) || (n_elems > BEST_N_MAX) || ((n_elems % 2u) == 0u))
    {
        return CODE39_ERR_COUNT;        /* a read starts and ends with a bar */
    }
    best_levels(p_widths, n_elems, levels);
    nar = (levels[0] + levels[2]) / 2u;
    if ((nar == 0u) || (levels[1] <= levels[0]) || (levels[3] <= levels[2]))
    {
        return CODE39_ERR_RATIO;
    }
    inv = (100u << 16) / nar;
    for (entry = 0u; entry < n_elems; entry++)
    {
        b_speck[entry] = is_speck(p_widths[entry], (uint16_t)entry, levels);
    }
    for (entry = 0u; entry < TABLE_LEN; entry++)
    {
        uint16_t const pat = bits_of(g_code39_table[entry].p_bits);
        uint8_t        kdx;

        if (g_code39_table[entry].chr == '*')
        {
            continue;
        }
        /* *X*: start, gap, X, gap, stop -- the gaps are narrow spaces */
        for (kdx = 0u; kdx < BEST_T; kdx++)
        {
            uint8_t const  char_idx   = (uint8_t)(kdx / 10u);
            uint8_t const  pos = (uint8_t)(kdx % 10u);
            uint16_t const pattern   = (char_idx == 1u) ? pat : star;
            bool const     b_w = (pos < 9u) && (((pattern >> (8u - pos)) & 1u) != 0u);

            expect[kdx]  = levels[(((kdx % 2u) != 0u) ? 2u : 0u) + (b_w ? 1u : 0u)];
            b_wide[kdx] = b_w;
        }
        for (dir = 0u; dir < 2u; dir++)
        {
            uint32_t cost;

            if (dir == 1u)
            {
                for (kdx = 0u; kdx < (BEST_T / 2u); kdx++)
                {
                    uint32_t const exp_width  = expect[kdx];
                    bool const     b_expect_wide = b_wide[kdx];

                    expect[kdx]  = expect[BEST_T - 1u - kdx];
                    b_wide[kdx] = b_wide[BEST_T - 1u - kdx];
                    expect[BEST_T - 1u - kdx]  = exp_width;
                    b_wide[BEST_T - 1u - kdx] = b_expect_wide;
                }
            }
            /* A fit costing c2 or more changes neither result, and the
               runner-up only matters below the margin times the largest
               best that is accepted. */
            cost = best_fit(p_widths, n_elems, expect, b_wide, b_speck, inv, (next_cost < BEST_BOUND_MAX) ? next_cost : BEST_BOUND_MAX);
            if (cost < best_cost)
            {
                if (g_code39_table[entry].chr != ch1)
                {
                    next_cost = best_cost;
                }
                best_cost     = cost;
                ch1    = g_code39_table[entry].chr;
                b_rev1 = (dir == 1u);
            }
            else if ((cost < next_cost) && (g_code39_table[entry].chr != ch1))
            {
                next_cost = cost;
            }
        }
    }
    if ((ch1 == '\0') || (best_cost > ((uint32_t)BEST_ELEM_MAX * BEST_T)) ||
        (((uint64_t)next_cost * 10u) < ((uint64_t)best_cost * BEST_MARGIN_X10)))
    {
        return CODE39_ERR_PATTERN;
    }
    *p_char       = ch1;
    *pb_reversed = b_rev1;
    return CODE39_OK;
}
