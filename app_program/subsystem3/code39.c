/** @file code39.c
 *
 *  @brief Subsystem 3 -- Code 39 decoder.  See code39.h.
 */

#include <stddef.h>
#include "code39.h"

#define CHAR_ELEMS   9u /* bars and spaces in one character */
#define REPAIR_TRIES 4u /* narrow or wide added at the front or back */
#define ELEM_KINDS   2u /* bars, spaces */
#define WIDTH_KINDS  2u /* narrow, wide */
#define DIRECTIONS   2u /* forwards, reversed */

typedef struct
{
    char         chr;
    char const * p_bits; /**< 9 elements, bar first: 1 = wide */
} code39_entry_t;

#define TABLE_LEN (sizeof(g_code39_table) / sizeof(g_code39_table[0]))

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

#define BEST_T          29u /* elements of *X*                  */
#define BEST_N_MIN      (BEST_T - (2u * BEST_MAX_EDITS)) /* 25 */
#define BEST_N_MAX      (BEST_T + (2u * BEST_MAX_EDITS)) /* 33 */
#define BEST_BAND       (2u * BEST_MAX_EDITS) /* |measured - printed| index */
#define BEST_EDIT       2500u /* = a 0.5 narrow width error        */
#define BEST_MAX_EDITS  2u    /* more defects than this: no read   */
#define BEST_ELEM_MAX   1600u /* mean error <= 0.4 narrow per step */
#define BEST_MARGIN_X10 20u   /* runner-up must cost >= 2 x best   */
#define BEST_SPECK_PCT  50u   /* speck: under half a narrow       */
#define BEST_INF        0x3FFFFFFFu
#define BEST_BOUND_MAX  (((BEST_ELEM_MAX * BEST_T) * BEST_MARGIN_X10) / 10u)

/** Best match: the measured read, shared by every template fitted to it. */
typedef struct
{
    uint32_t const * p_w;     /**< measured widths                    */
    uint16_t         n_elems; /**< ... how many                       */
    uint32_t const * p_lv;    /**< bar n, bar W, space n, space W      */
    bool const *     p_speck; /**< measured element is a speck        */
    uint32_t         inv;     /**< (100 << 16) / narrow width         */
} best_fit_in_t;

/** Best match: the best character so far and the runner-up's cost. */
typedef struct
{
    uint32_t best_cost; /**< best cost and its character         */
    uint32_t next_cost; /**< best cost of any other character    */
    char     ch1;
    bool     b_rev1;
} best_rank_t;

/** Best match: the edit-distance tables (5.2 KB, kept off the stack). */
typedef struct
{
    uint32_t cost[BEST_N_MAX + 1u][BEST_T + 1u];
    uint8_t  edits[BEST_N_MAX + 1u][BEST_T + 1u];
    uint32_t rmin[BEST_N_MAX + 1u]; /**< smallest cost in each row */
} best_tables_t;

/** Code 39 character set (ISO/IEC 16388), element order b s b s b s b s b. */
static code39_entry_t const g_code39_table[] = {
    {'0', "000110100"}, {'1', "100100001"}, {'2', "001100001"},
    {'3', "101100000"}, {'4', "000110001"}, {'5', "100110000"},
    {'6', "001110000"}, {'7', "000100101"}, {'8', "100100100"},
    {'9', "001100100"}, {'A', "100001001"}, {'B', "001001001"},
    {'C', "101001000"}, {'D', "000011001"}, {'E', "100011000"},
    {'F', "001011000"}, {'G', "000001101"}, {'H', "100001100"},
    {'I', "001001100"}, {'J', "000011100"}, {'K', "100000011"},
    {'L', "001000011"}, {'M', "101000010"}, {'N', "000010011"},
    {'O', "100010010"}, {'P', "001010010"}, {'Q', "000000111"},
    {'R', "100000110"}, {'S', "001000110"}, {'T', "000010110"},
    {'U', "110000001"}, {'V', "011000001"}, {'W', "111000000"},
    {'X', "010010001"}, {'Y', "110010000"}, {'Z', "011010000"},
    {'-', "010000101"}, {'.', "110000100"}, {' ', "011000100"},
    {'*', "010010100"}, {'$', "010101000"}, {'/', "010100010"},
    {'+', "010001010"}, {'%', "000101010"}};

static best_tables_t g_bt;

static code39_result_t decode_both(uint32_t const * p_widths, uint16_t n_elems,
                                   uint16_t ratio_min_x10, char * p_text,
                                   bool * pb_reversed);
static code39_result_t repair_tries(uint32_t const * p_widths, uint16_t n_elems,
                                    uint16_t ratio_min_x10, char * p_char,
                                    bool * pb_reversed, code39_result_t result);
static code39_result_t best_match(uint32_t const * p_widths, uint16_t n_elems,
                                  uint32_t const levels[4], uint32_t nar,
                                  char * p_char, bool * pb_reversed);
static uint16_t        bits_of(char const * p_bits);
static uint32_t width_at(uint32_t const * p_w, uint16_t n_elems, uint16_t pos,
                         bool b_rev);
static uint32_t kth_widest(uint32_t const * p_w, uint8_t first, uint8_t rank,
                           uint8_t * p_idx);
static code39_result_t classify(uint32_t const * p_w, uint16_t n_elems,
                                uint16_t first, bool b_rev,
                                uint16_t ratio_min_x10, uint16_t * p_pattern);
static code39_result_t decode_dir(uint32_t const * p_w, uint16_t n_elems,
                                  uint16_t ratio_min_x10, bool b_rev,
                                  char * p_text);
static uint32_t        median_of(uint32_t * p_v, uint16_t n_elems);
static void            best_levels(uint32_t const * p_w, uint16_t n_elems,
                                   uint32_t * p_lv);
static uint32_t        sq_err(uint32_t meas, uint32_t expect, uint32_t inv);
static bool            in_band(uint16_t idx, uint16_t jdx);
static bool     is_speck(uint32_t width, uint16_t idx, uint32_t const * p_lv);
static uint32_t best_fit(best_fit_in_t const * p_fit, uint32_t const * p_e,
                         bool const * p_wide, uint32_t bound);
static void best_try_char(best_fit_in_t const *  p_fit,
                          code39_entry_t const * p_entry, best_rank_t * p_rank);
static void best_template(uint16_t pat, uint32_t const levels[4],
                          uint32_t expect[BEST_T], bool b_wide[BEST_T]);
static void best_reverse(uint32_t expect[BEST_T], bool b_wide[BEST_T]);
static void best_rank(best_rank_t * p_rank, uint32_t cost, char chr,
                      bool b_rev);
static void best_clear(uint16_t n_elems);
static void best_steps(best_fit_in_t const * p_fit, uint32_t const * p_e,
                       bool const * p_wide, uint16_t idx, uint16_t jdx);
static void best_relax(uint32_t idx, uint32_t jdx, uint32_t cost,
                       uint8_t edits);

uint16_t code39_pattern (char chr)
{
    size_t   idx;
    uint16_t pattern = 0u;
    bool     b_found = false;

    for (idx = 0u; (idx < TABLE_LEN) && !b_found; idx++)
    {
        if (g_code39_table[idx].chr == chr)
        {
            pattern = bits_of(g_code39_table[idx].p_bits);
            b_found = true;
        }
    }
    return pattern;
}

char code39_char (uint16_t pattern)
{
    size_t idx;
    char   chr     = '\0';
    bool   b_found = false;

    for (idx = 0u; (idx < TABLE_LEN) && !b_found; idx++)
    {
        if (bits_of(g_code39_table[idx].p_bits) == pattern)
        {
            chr     = g_code39_table[idx].chr;
            b_found = true;
        }
    }
    return chr;
}

code39_result_t code39_decode (uint32_t const * p_widths, uint16_t n_elems,
                               uint16_t ratio_min_x10, char * p_text,
                               bool * pb_reversed)
{
    code39_result_t result;

    p_text[0]    = '\0';
    *pb_reversed = false;
    if (n_elems < 9u)
    {
        result = CODE39_NOT_BARCODE;
    }
    else if (((n_elems + 1u) / 10u) > CODE39_MAX_CHARS)
    {
        result = CODE39_ERR_TOO_MANY;
    }
    else if ((((n_elems + 1u) % 10u) != 0u) || (n_elems < 29u))
    {
        result = CODE39_ERR_COUNT; /* also "**" with no data */
    }
    else
    {
        result =
            decode_both(p_widths, n_elems, ratio_min_x10, p_text, pb_reversed);
    }
    return result;
}

code39_result_t code39_decode_one (uint32_t const * p_widths, uint16_t n_elems,
                                   uint16_t ratio_min_x10, char * p_char,
                                   bool * pb_reversed)
{
    char            text[CODE39_MAX_CHARS];
    code39_result_t result =
        code39_decode(p_widths, n_elems, ratio_min_x10, text, pb_reversed);

    *p_char = '\0';
    if ((CODE39_OK == result) && (('\0' == text[0]) || (text[1] != '\0')))
    {
        result = CODE39_ERR_LENGTH;
    }
    if (CODE39_OK == result)
    {
        *p_char = text[0];
    }
    return result;
}

/* ---- one element pair lost at an end (7 Nov) ----------------------- */
/** A read one bar short at either end still decodes: the start '*' begins
   (narrow bar, wide space) and the stop '*' ends (narrow space, narrow
   bar); a missed end bar takes its neighbouring space into the quiet
   zone, so 2 elements are gone.  Each of the four ways of putting them
   back is tried (front / back, either reading direction); the read is
   kept only if every way that decodes gives the same character. */
code39_result_t code39_decode_one_repair (uint32_t const * p_widths,
                                          uint16_t         n_elems,
                                          uint16_t ratio_min_x10, char * p_char,
                                          bool * pb_reversed)
{
    code39_result_t result = code39_decode_one(p_widths, n_elems, ratio_min_x10,
                                               p_char, pb_reversed);

    if ((result != CODE39_OK) && (result != CODE39_NOT_BARCODE)
        && ((n_elems + 2u) <= (CODE39_MAX_CHARS * 10u))
        && (0u == ((n_elems + 3u) % 10u)))
    {
        result = repair_tries(p_widths, n_elems, ratio_min_x10, p_char,
                              pb_reversed, result);
    }
    return result;
}

code39_result_t code39_decode_one_best (uint32_t const * p_widths,
                                        uint16_t n_elems, char * p_char,
                                        bool * pb_reversed)
{
    uint32_t        levels[4]; /* bar n, bar W, space n, space W */
    uint32_t        nar;
    code39_result_t result;

    *p_char      = '\0';
    *pb_reversed = false;
    if (n_elems < 9u)
    {
        result = CODE39_NOT_BARCODE;
    }
    else if ((n_elems < BEST_N_MIN) || (n_elems > BEST_N_MAX)
             || (0u == (n_elems % 2u)))
    {
        result = CODE39_ERR_COUNT; /* a read starts and ends with a bar */
    }
    else
    {
        best_levels(p_widths, n_elems, levels);
        nar = (levels[0] + levels[2]) / 2u;
        if ((0u == nar) || (levels[1] <= levels[0]) || (levels[3] <= levels[2]))
        {
            result = CODE39_ERR_RATIO;
        }
        else
        {
            result =
                best_match(p_widths, n_elems, levels, nar, p_char, pb_reversed);
        }
    }
    return result;
}

/** code39_decode() once the element count is right: forwards, then
    reversed.  Returns CODE39_OK or the forward reason (the reversed reason
    if only the forward framing was wrong). */
static code39_result_t decode_both (uint32_t const * p_widths, uint16_t n_elems,
                                    uint16_t ratio_min_x10, char * p_text,
                                    bool * pb_reversed)
{
    code39_result_t const fwd =
        decode_dir(p_widths, n_elems, ratio_min_x10, false, p_text);
    code39_result_t rev;
    code39_result_t result = CODE39_OK;

    if (fwd != CODE39_OK)
    {
        rev = decode_dir(p_widths, n_elems, ratio_min_x10, true, p_text);
        if (CODE39_OK == rev)
        {
            *pb_reversed = true;
        }
        else
        {
            p_text[0] = '\0';
            /* Report the forward reason unless only the framing was wrong. */
            result = (CODE39_ERR_START_STOP == fwd) ? rev : fwd;
        }
    }
    return result;
}

/** code39_decode_one_repair(): decode with a narrow or wide element added at
    the front or the back (REPAIR_TRIES).  Returns CODE39_OK if the repairs
    that decode agree, CODE39_ERR_PATTERN if they disagree, else result. */
static code39_result_t repair_tries (uint32_t const * p_widths,
                                     uint16_t n_elems, uint16_t ratio_min_x10,
                                     char * p_char, bool * pb_reversed,
                                     code39_result_t result)
{
    static uint32_t widths[CODE39_MAX_CHARS * 10u];
    uint32_t        narrow     = 0xFFFFFFFFu;
    uint32_t        widest     = 0u;
    char            got        = '\0';
    bool            b_got_rev  = false;
    bool            b_conflict = false;
    uint16_t        idx;
    uint8_t         kdx;

    for (idx = 0u; idx < n_elems; idx++)
    {
        narrow = (p_widths[idx] < narrow) ? p_widths[idx] : narrow;
        widest = (p_widths[idx] > widest) ? p_widths[idx] : widest;
    }
    for (kdx = 0u; (kdx < REPAIR_TRIES) && !b_conflict; kdx++)
    {
        bool const     b_front = (kdx < 2u);
        uint32_t const add_first =
            (0u == kdx) ? narrow
                        : ((3u == kdx) ? widest : narrow); /* first added */
        uint32_t const add_second =
            (0u == kdx) ? widest : narrow; /* second added */
        char     chr;
        bool     b_rev;
        uint16_t out_n = 0u;

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
            widths[out_n++] =
                (2u == kdx)
                    ? narrow
                    : widest; /* k 2: space n, bar n; k 3: space W, bar n */
            widths[out_n++] = narrow;
        }
        if (CODE39_OK
            == code39_decode_one(widths, out_n, ratio_min_x10, &chr, &b_rev))
        {
            if ((got != '\0') && (chr != got))
            {
                b_conflict = true; /* repairs disagree */
            }
            else
            {
                got       = chr;
                b_got_rev = b_rev;
            }
        }
    }
    if (b_conflict)
    {
        *p_char = '\0';
        result  = CODE39_ERR_PATTERN;
    }
    else if ('\0' == got)
    {
        *p_char = '\0';
    }
    else
    {
        *p_char      = got;
        *pb_reversed = b_got_rev;
        result       = CODE39_OK;
    }
    return result;
}

/** code39_decode_one_best() once the counts and levels are usable: the
    best-matching character, if it is good enough and clear of the runner-up.
 */
static code39_result_t best_match (uint32_t const * p_widths, uint16_t n_elems,
                                   uint32_t const levels[4], uint32_t nar,
                                   char * p_char, bool * pb_reversed)
{
    bool            b_speck[BEST_N_MAX]; /* measured element is a speck */
    best_fit_in_t   fit;
    best_rank_t     rank   = {BEST_INF, BEST_INF, '\0', false};
    code39_result_t result = CODE39_ERR_PATTERN;
    size_t          entry;

    for (entry = 0u; entry < n_elems; entry++)
    {
        /* Cast (uint16_t)entry: entry < n_elems <= BEST_N_MAX (33, tested
           above). */
        b_speck[entry] = is_speck(p_widths[entry], (uint16_t)entry, levels);
    }
    fit.p_w     = p_widths;
    fit.n_elems = n_elems;
    fit.p_lv    = levels;
    fit.p_speck = b_speck;
    fit.inv     = (100u << 16) / nar;
    for (entry = 0u; entry < TABLE_LEN; entry++)
    {
        if ('*' != g_code39_table[entry].chr)
        {
            best_try_char(&fit, &g_code39_table[entry], &rank);
        }
    }
    /* Casts (uint32_t)BEST_ELEM_MAX: a constant that fits;
       (uint64_t)rank.next_cost, (uint64_t)rank.best_cost: every uint32_t value
       fits. */
    if ((rank.ch1 != '\0')
        && (rank.best_cost <= ((uint32_t)BEST_ELEM_MAX * BEST_T))
        && (((uint64_t)rank.next_cost * 10u)
            >= ((uint64_t)rank.best_cost * BEST_MARGIN_X10)))
    {
        *p_char      = rank.ch1;
        *pb_reversed = rank.b_rev1;
        result       = CODE39_OK;
    }
    return result;
}

/** Best match: fit the template *X* for one character X, both ways round. */
static void best_try_char (best_fit_in_t const *  p_fit,
                           code39_entry_t const * p_entry, best_rank_t * p_rank)
{
    uint32_t expect[BEST_T];
    bool     b_wide[BEST_T];
    uint8_t  dir;

    best_template(bits_of(p_entry->p_bits), p_fit->p_lv, expect, b_wide);
    for (dir = 0u; dir < DIRECTIONS; dir++)
    {
        uint32_t cost;

        if (1u == dir)
        {
            best_reverse(expect, b_wide);
        }
        /* A fit costing c2 or more changes neither result, and the
           runner-up only matters below the margin times the largest
           best that is accepted. */
        cost = best_fit(p_fit, expect, b_wide,
                        (p_rank->next_cost < BEST_BOUND_MAX) ? p_rank->next_cost
                                                             : BEST_BOUND_MAX);
        best_rank(p_rank, cost, p_entry->chr, 1u == dir);
    }
}

/** *X*: start, gap, X, gap, stop -- the gaps are narrow spaces.  Expected
   width (from the four levels) and wide flag of each of the BEST_T
   elements. */
static void best_template (uint16_t pat, uint32_t const levels[4],
                           uint32_t expect[BEST_T], bool b_wide[BEST_T])
{
    uint16_t const star = code39_pattern('*');
    uint8_t        kdx;

    for (kdx = 0u; kdx < BEST_T; kdx++)
    {
        /* Casts (uint8_t)(kdx / 10u): the value is 0..25; (uint8_t)(kdx % 10u):
           the value is 0..9. */
        uint8_t const  char_idx = (uint8_t)(kdx / 10u);
        uint8_t const  pos      = (uint8_t)(kdx % 10u);
        uint16_t const pattern  = (1u == char_idx) ? pat : star;
        bool const b_w = (pos < 9u) && (((pattern >> (8u - pos)) & 1u) != 0u);

        expect[kdx] = levels[(((kdx % 2u) != 0u) ? 2u : 0u) + (b_w ? 1u : 0u)];
        b_wide[kdx] = b_w;
    }
}

/** Reverse the template in place (a barcode read backwards). */
static void best_reverse (uint32_t expect[BEST_T], bool b_wide[BEST_T])
{
    uint8_t kdx;

    for (kdx = 0u; kdx < (BEST_T / 2u); kdx++)
    {
        uint32_t const exp_width     = expect[kdx];
        bool const     b_expect_wide = b_wide[kdx];

        expect[kdx]                 = expect[(BEST_T - 1u) - kdx];
        b_wide[kdx]                 = b_wide[(BEST_T - 1u) - kdx];
        expect[(BEST_T - 1u) - kdx] = exp_width;
        b_wide[(BEST_T - 1u) - kdx] = b_expect_wide;
    }
}

/** Keep the best cost with its character and direction, and the best cost
   of any other character. */
static void best_rank (best_rank_t * p_rank, uint32_t cost, char chr,
                       bool b_rev)
{
    if (cost < p_rank->best_cost)
    {
        if (chr != p_rank->ch1)
        {
            p_rank->next_cost = p_rank->best_cost;
        }
        p_rank->best_cost = cost;
        p_rank->ch1       = chr;
        p_rank->b_rev1    = b_rev;
    }
    else if ((cost < p_rank->next_cost) && (chr != p_rank->ch1))
    {
        p_rank->next_cost = cost;
    }
    else
    {
        /* No action required. */
    }
}

/** Pattern of a table entry: its 9 elements as bits, bar first, 1 = wide. */
static uint16_t bits_of (char const * p_bits)
{
    uint16_t pattern = 0u;
    uint8_t  idx;

    for (idx = 0u; idx < CHAR_ELEMS; idx++)
    {
        /* Cast (uint16_t)((pattern << 1) | (('1' ...))): 9 one-bit shifts: 9
           bits. */
        pattern = (uint16_t)((pattern << 1) | (('1' == p_bits[idx]) ? 1u : 0u));
    }
    return pattern;
}

/** Width of element k, read forwards or backwards. */
static uint32_t width_at (uint32_t const * p_w, uint16_t n_elems, uint16_t pos,
                          bool b_rev)
{
    return b_rev ? p_w[(n_elems - 1u) - pos] : p_w[pos];
}

/** The k-th widest (k = 0, 1, ...) of the elements of one kind: bars at
   even indexes 0..8, spaces at odd ones 1..7. */
static uint32_t kth_widest (uint32_t const * p_w, uint8_t first, uint8_t rank,
                            uint8_t * p_idx)
{
    uint32_t used = 0u;
    uint32_t best = 0u;
    uint8_t  rank_idx;
    uint8_t  idx;

    *p_idx = first;
    for (rank_idx = 0u; rank_idx <= rank; rank_idx++)
    {
        bool b_any = false;

        for (idx = first; idx < CHAR_ELEMS; idx += 2u)
        {
            if ((0u == (used & (1u << idx))) && (!b_any || (p_w[idx] > best)))
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

/** Classify one character's 9 elements (7 Nov): bars and spaces are ranked
   SEPARATELY, the 2 widest bars and the widest space being wide, each
   standing out from the rest of its kind by ratio_min.  Under the sensor's
   spot black reads wider and white narrower, so ranking all 9 together
   could take a fat narrow bar for the wide space. */
static code39_result_t classify (uint32_t const * p_w, uint16_t n_elems,
                                 uint16_t first, bool b_rev,
                                 uint16_t ratio_min_x10, uint16_t * p_pattern)
{
    uint32_t        widths[9];
    uint32_t        wide    = 0u;
    uint16_t        pattern = 0u;
    code39_result_t result  = CODE39_OK;
    uint8_t         ib1;
    uint8_t         ib2;
    uint8_t         ib3;
    uint8_t         is1;
    uint8_t         is2;
    uint32_t        bar_w2;
    uint32_t        bar_w3;
    uint32_t        space_w1;
    uint32_t        space_w2;
    uint8_t         idx;

    for (idx = 0u; idx < CHAR_ELEMS; idx++)
    {
        /* Cast (uint16_t)(first + idx): first = char_idx x 10 <= n_elems - 9
           and idx < 9, so below n_elems. */
        widths[idx] = width_at(p_w, n_elems, (uint16_t)(first + idx), b_rev);
    }
    /* Casts (uint64_t)bar_w2, (uint64_t)bar_w3, (uint64_t)space_w1,
       (uint64_t)space_w2: every uint32_t value fits. */
    (void)kth_widest(widths, 0u, 0u, &ib1);
    bar_w2   = kth_widest(widths, 0u, 1u, &ib2);
    bar_w3   = kth_widest(widths, 0u, 2u, &ib3); /* widest narrow bar   */
    space_w1 = kth_widest(widths, 1u, 0u, &is1);
    space_w2 = kth_widest(widths, 1u, 1u, &is2); /* widest narrow space */
    if ((bar_w3 != 0u) && (space_w2 != 0u)
        && (((uint64_t)bar_w2 * 10u) >= ((uint64_t)bar_w3 * ratio_min_x10))
        && (((uint64_t)space_w1 * 10u) >= ((uint64_t)space_w2 * ratio_min_x10)))
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
        uint32_t const bar_w1   = kth_widest(widths, 0u, 0u, &ib1);
        uint32_t const bar_w5   = kth_widest(widths, 0u, 4u, &ib5);

        /* Casts (uint64_t)space_w3, (uint64_t)space_w4, (uint64_t)bar_w1,
           (uint64_t)bar_w5: every uint32_t value fits. */
        if ((0u == space_w4)
            || (((uint64_t)space_w3 * 10u)
                < ((uint64_t)space_w4 * ratio_min_x10))
            || (((uint64_t)bar_w1 * 10u) >= ((uint64_t)bar_w5 * ratio_min_x10)))
        {
            result = CODE39_ERR_RATIO;
        }
        else
        {
            (void)space_w2;
            wide = (1u << is1) | (1u << is2) | (1u << is3);
        }
    }
    if (CODE39_OK == result)
    {
        for (idx = 0u; idx < CHAR_ELEMS; idx++)
        {
            /* Cast (uint16_t)((pattern << 1) | (((wide ...)))): 9 one-bit
               shifts: 9 bits. */
            pattern = (uint16_t)((pattern << 1)
                                 | (((wide & (1u << idx)) != 0u) ? 1u : 0u));
        }
        *p_pattern = pattern;
    }
    return result;
}

/** Decode the characters of a read in one direction (b_rev: reversed) into
    p_text, wide/narrow split at ratio_min_x10 / 10.  Returns CODE39_OK or the
    first error. */
static code39_result_t decode_dir (uint32_t const * p_w, uint16_t n_elems,
                                   uint16_t ratio_min_x10, bool b_rev,
                                   char * p_text)
{
    /* Cast (uint16_t)((n_elems + 1u) / 10u): the value is 0..6553. */
    uint16_t const  chars = (uint16_t)((n_elems + 1u) / 10u);
    uint16_t        char_idx;
    uint16_t        out    = 0u;
    code39_result_t result = CODE39_OK;

    for (char_idx = 0u; (char_idx < chars) && (CODE39_OK == result); char_idx++)
    {
        bool const b_frame = (0u == char_idx) || (char_idx == (chars - 1u));
        uint16_t   pattern = 0u;
        char       chr;

        /* Cast (uint16_t)(char_idx * 10u): char_idx < (n_elems + 1) / 10, so
           char_idx x 10 <= n_elems. */
        result = classify(p_w, n_elems, (uint16_t)(char_idx * 10u), b_rev,
                          ratio_min_x10, &pattern);
        if (CODE39_OK == result)
        {
            chr = code39_char(pattern);
            if ('\0' == chr)
            {
                result = CODE39_ERR_PATTERN;
            }
            else if (b_frame && (chr != '*'))
            {
                result = CODE39_ERR_START_STOP;
            }
            else if (!b_frame && ('*' == chr))
            {
                result = CODE39_ERR_PATTERN; /* '*' only frames the code */
            }
            else if (!b_frame)
            {
                p_text[out] = chr;
                out++;
            }
            else
            {
                /* a '*' at either end */
            }
        }
    }
    if (CODE39_OK == result)
    {
        p_text[out] = '\0';
    }
    return result;
}

/** Median of n_elems values; sorts p_v in place. */
static uint32_t median_of (uint32_t * p_v, uint16_t n_elems)
{
    uint16_t idx;
    uint16_t jdx;

    for (idx = 1u; idx < n_elems; idx++) /* insertion sort, n <= 33 */
    {
        uint32_t const value = p_v[idx];

        for (jdx = idx; (jdx > 0u) && (p_v[jdx - 1u] > value); jdx--)
        {
            p_v[jdx] = p_v[jdx - 1u];
        }
        p_v[jdx] = value;
    }
    return (0u == n_elems) ? 0u : p_v[n_elems / 2u];
}

/** This read's widths of narrow and wide bars and spaces: the median
   narrow and wide of each kind, split half-way between the median of
   all elements (most are narrow) and the median of the widest third. */
static void best_levels (uint32_t const * p_w, uint16_t n_elems,
                         uint32_t * p_lv)
{
    static uint32_t sorted[BEST_N_MAX]; /* n <= BEST_N_MAX here */
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
    wid =
        sorted[(n_elems - 1u) - ((n_elems / 3u) / 2u)]; /* median of the widest
                                                         third (sorted) */
    thr = (nar + wid) / 2u;
    for (kind = 0u; kind < ELEM_KINDS; kind++)
    {
        for (wide = 0u; wide < WIDTH_KINDS; wide++)
        {
            mid = 0u;
            for (idx = kind; idx < n_elems; idx += 2u)
            {
                if ((p_w[idx] >= thr) == (wide != 0u))
                {
                    sorted[mid++] = p_w[idx];
                }
            }
            p_lv[(kind * 2u) + wide] = (mid > 0u) ? median_of(sorted, mid)
                                                  : ((wide != 0u) ? wid : nar);
        }
    }
}

/** Width error squared in hundredths of a narrow; inv = 100 x 65536 / narrow.
   Multiply and shift only: the Cortex-M0+ has no 64-bit divide, and this
   runs about 50 000 times per read. */
static uint32_t sq_err (uint32_t meas, uint32_t expect, uint32_t inv)
{
    uint32_t const diff = (meas > expect) ? (meas - expect) : (expect - meas);
    uint32_t       scaled;
    uint32_t       err = 1000000u;

    if (diff <= 0x3FFFFu)
    {
        scaled = (diff * inv) >> 16;
        err    = (scaled > 1000u) ? 1000000u : (scaled * scaled);
    }
    return err;
}

/** True if measured index idx and printed index jdx are close enough for the
    best-match table (BEST_BAND). */
static bool in_band (uint16_t idx, uint16_t jdx)
{
    /* Casts (uint16_t)(idx - jdx): idx > jdx on this branch;
       (uint16_t)(jdx - idx): jdx >= idx on this branch. */
    uint16_t const diff =
        (idx > jdx) ? (uint16_t)(idx - jdx) : (uint16_t)(jdx - idx);

    return (diff <= BEST_BAND);
}

/** True if a measured element is a speck: thinner than BEST_SPECK_PCT % of a
    narrow element of its kind (bar or space, by idx). */
static bool is_speck (uint32_t width, uint16_t idx, uint32_t const * p_lv)
{
    uint32_t const narrow = p_lv[((idx % 2u) != 0u) ? 2u : 0u];

    /* Casts (uint64_t)width, (uint64_t)narrow: every uint32_t value fits. */
    return ((uint64_t)width * 100u) < ((uint64_t)narrow * BEST_SPECK_PCT);
}

/** Cost of lining p_w up with the printed widths p_e (BEST_T of them);
   p_wide: printed element wide; p_speck: measured element is a speck.
   14 Nov: gives up (BEST_INF) once every open path costs bound or more:
   such a fit can change neither the best nor the runner-up (a car read
   took over 50 ms and tripped the line task's heartbeat). */
static uint32_t best_fit (best_fit_in_t const * p_fit, uint32_t const * p_e,
                          bool const * p_wide, uint32_t bound)
{
    uint16_t const n_elems = p_fit->n_elems;
    uint32_t       best    = BEST_INF;
    bool           b_over  = false;
    uint16_t       idx;
    uint16_t       jdx;

    best_clear(n_elems);
    for (idx = 0u; (idx < n_elems) && !b_over; idx++)
    {
        /* Every path to the end passes a filled cell of row i, i + 1 or
           i + 2 (a step moves on 1 or 3 measured elements, and rows
           before i are done); costs only grow along a path. */
        uint32_t c_open = g_bt.rmin[idx];

        c_open = (((idx + 1u) <= n_elems) && (g_bt.rmin[idx + 1u] < c_open))
                     ? g_bt.rmin[idx + 1u]
                     : c_open;
        c_open = (((idx + 2u) <= n_elems) && (g_bt.rmin[idx + 2u] < c_open))
                     ? g_bt.rmin[idx + 2u]
                     : c_open;
        if (c_open >= bound)
        {
            b_over = true; /* no fit below the bound: BEST_INF */
        }
        /* Cast (uint16_t)(idx - BEST_BAND): idx > BEST_BAND on this branch. */
        for (jdx = (idx > BEST_BAND) ? (uint16_t)(idx - BEST_BAND) : 0u;
             !b_over && (jdx < BEST_T) && (jdx <= (idx + BEST_BAND)); jdx++)
        {
            if (g_bt.cost[idx][jdx] < BEST_INF)
            {
                best_steps(p_fit, p_e, p_wide, idx, jdx);
            }
        }
    }
    if (!b_over)
    {
        best = in_band(n_elems, BEST_T) ? g_bt.cost[n_elems][BEST_T] : BEST_INF;
        if (in_band(n_elems, (BEST_T - 2u))
            && (g_bt.edits[n_elems][BEST_T - 2u] < BEST_MAX_EDITS)
            && (g_bt.cost[n_elems][BEST_T - 2u] < BEST_INF)
            && ((g_bt.cost[n_elems][BEST_T - 2u] + BEST_EDIT) < best))
        {
            /* last space and bar lost */
            best = g_bt.cost[n_elems][BEST_T - 2u] + BEST_EDIT;
        }
    }
    return best;
}

/** Every defect moves the two indexes apart by 2, so only a band of cells
   can be reached; only those are used. */
static void best_clear (uint16_t n_elems)
{
    uint16_t idx;
    uint16_t jdx;

    for (idx = 0u; idx <= n_elems; idx++)
    {
        /* Cast (uint16_t)(idx - BEST_BAND): idx > BEST_BAND on this branch. */
        for (jdx = (idx > BEST_BAND) ? (uint16_t)(idx - BEST_BAND) : 0u;
             (jdx <= BEST_T) && (jdx <= (idx + BEST_BAND)); jdx++)
        {
            g_bt.cost[idx][jdx]  = BEST_INF;
            g_bt.edits[idx][jdx] = 0u;
        }
        g_bt.rmin[idx] = BEST_INF;
    }
    g_bt.cost[0][0]  = 0u;
    g_bt.rmin[0]     = 0u;
    g_bt.cost[0][2]  = BEST_EDIT; /* first bar and space lost */
    g_bt.edits[0][2] = 1u;
}

/** The three steps out of the filled cell (idx measured, jdx printed). */
static void best_steps (best_fit_in_t const * p_fit, uint32_t const * p_e,
                        bool const * p_wide, uint16_t idx, uint16_t jdx)
{
    uint32_t const * const p_w       = p_fit->p_w;
    uint32_t const         cost_here = g_bt.cost[idx][jdx];
    uint8_t const          edits     = g_bt.edits[idx][jdx];

    if (!p_fit->p_speck[idx])
    {
        /* 1 : 1 */
        best_relax(idx + 1u, jdx + 1u,
                   cost_here + sq_err(p_w[idx], p_e[jdx], p_fit->inv), edits);
    }
    /* Casts (uint16_t)(idx + 3u): idx + 3 <= n_elems (tested in the same
       statement); (uint16_t)(jdx + 1u): jdx <= BEST_T (29), a table index. */
    if ((edits < BEST_MAX_EDITS) && ((idx + 3u) <= p_fit->n_elems)
        && in_band((uint16_t)(idx + 3u), (uint16_t)(jdx + 1u))
        && p_fit->p_speck[idx + 1u])
    {
        /* 3 measured : 1 printed */
        /* Cast (uint8_t)(edits + 1u): edits < BEST_MAX_EDITS (2, tested
           above). */
        best_relax(idx + 3u, jdx + 1u,
                   cost_here + BEST_EDIT
                       + sq_err(p_w[idx] + p_w[idx + 1u] + p_w[idx + 2u],
                                p_e[jdx], p_fit->inv),
                   (uint8_t)(edits + 1u));
    }
    /* Casts (uint16_t)(idx + 1u): idx <= n_elems <= BEST_N_MAX (33), a table
       index; (uint16_t)(jdx + 3u): jdx + 3 <= BEST_T (tested in the same
       statement). */
    if ((edits < BEST_MAX_EDITS) && ((jdx + 3u) <= BEST_T)
        && in_band((uint16_t)(idx + 1u), (uint16_t)(jdx + 3u))
        && !p_wide[jdx + 1u])
    {
        /* 1 measured : 3 printed */
        /* Cast (uint8_t)(edits + 1u): edits < BEST_MAX_EDITS (2, tested
           above). */
        best_relax(idx + 1u, jdx + 3u,
                   cost_here + BEST_EDIT
                       + sq_err(p_w[idx],
                                p_e[jdx] + p_e[jdx + 1u] + p_e[jdx + 2u],
                                p_fit->inv),
                   (uint8_t)(edits + 1u));
    }
}

/** A path reaches cell (idx, jdx) at this cost with this many edits: keep
   it if it is cheaper. */
static void best_relax (uint32_t idx, uint32_t jdx, uint32_t cost,
                        uint8_t edits)
{
    if (cost < g_bt.cost[idx][jdx])
    {
        g_bt.cost[idx][jdx]  = cost;
        g_bt.rmin[idx]       = (cost < g_bt.rmin[idx]) ? cost : g_bt.rmin[idx];
        g_bt.edits[idx][jdx] = edits;
    }
}

/*** end of file ***/
