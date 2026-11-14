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
        /* '$' '/' '+' '%': three wide spaces, no wide bar.  9 Nov: only if
           the bars really are all alike (widest < ratio_min x narrowest);
           a faint read whose wide bars came out too thin otherwise looked
           like one of these (L read as %, R as $). */
        uint8_t        is3;
        uint8_t        is4;
        uint8_t        ib5;
        uint32_t const s3 = kth_widest(w, 1u, 2u, &is3);
        uint32_t const s4 = kth_widest(w, 1u, 3u, &is4);
        uint32_t const b1 = kth_widest(w, 0u, 0u, &ib1);
        uint32_t const b5 = kth_widest(w, 0u, 4u, &ib5);

        if ((s4 == 0u) || (((uint64_t)s3 * 10u) < ((uint64_t)s4 * ratio_min_x10)) ||
            (((uint64_t)b1 * 10u) >= ((uint64_t)b5 * ratio_min_x10)))
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

static uint32_t median_of(uint32_t * p_v, uint16_t n)
{
    uint16_t i;
    uint16_t j;

    for (i = 1u; i < n; i++)            /* insertion sort, n <= 33 */
    {
        uint32_t const v = p_v[i];

        for (j = i; (j > 0u) && (p_v[j - 1u] > v); j--)
        {
            p_v[j] = p_v[j - 1u];
        }
        p_v[j] = v;
    }
    return (n == 0u) ? 0u : p_v[n / 2u];
}

/* This read's widths of narrow and wide bars and spaces: the median
   narrow and wide of each kind, split half-way between the median of
   all elements (most are narrow) and the median of the widest third. */
static void best_levels(uint32_t const * p_w, uint16_t n, uint32_t * p_lv)
{
    static uint32_t s[BEST_N_MAX];   /* n <= BEST_N_MAX here */
    uint32_t        nar;
    uint32_t        wid;
    uint32_t        thr;
    uint16_t        m;
    uint16_t        i;
    uint8_t         kind;
    uint8_t         wide;

    for (i = 0u; i < n; i++)
    {
        s[i] = p_w[i];
    }
    nar = median_of(s, n);
    wid = s[n - 1u - ((n / 3u) / 2u)];   /* median of the widest third (sorted) */
    thr = (nar + wid) / 2u;
    for (kind = 0u; kind < 2u; kind++)
    {
        for (wide = 0u; wide < 2u; wide++)
        {
            m = 0u;
            for (i = kind; i < n; i += 2u)
            {
                if ((p_w[i] >= thr) == (wide != 0u))
                {
                    s[m++] = p_w[i];
                }
            }
            p_lv[(kind * 2u) + wide] = (m > 0u) ? median_of(s, m) : ((wide != 0u) ? wid : nar);
        }
    }
}

/* Width error squared in hundredths of a narrow; inv = 100 x 65536 / narrow.
   Multiply and shift only: the Cortex-M0+ has no 64-bit divide, and this
   runs about 50 000 times per read. */
static uint32_t sq_err(uint32_t meas, uint32_t expect, uint32_t inv)
{
    uint32_t const d = (meas > expect) ? (meas - expect) : (expect - meas);
    uint32_t       e;

    if (d > 0x3FFFFu)
    {
        return 1000000u;
    }
    e = (d * inv) >> 16;
    return (e > 1000u) ? 1000000u : (e * e);
}

static bool in_band(uint16_t i, uint16_t j)
{
    uint16_t const d = (i > j) ? (uint16_t)(i - j) : (uint16_t)(j - i);

    return (d <= BEST_BAND);
}

static bool is_speck(uint32_t w, uint16_t i, uint32_t const * p_lv)
{
    uint32_t const narrow = p_lv[((i % 2u) != 0u) ? 2u : 0u];

    return ((uint64_t)w * 100u) < ((uint64_t)narrow * BEST_SPECK_PCT);
}

/* Cost of lining p_w up with the printed widths p_e (BEST_T of them);
   p_wide: printed element wide; p_speck: measured element is a speck.
   14 Nov: gives up (BEST_INF) once every open path costs bound or more:
   such a fit can change neither the best nor the runner-up (a car read
   took over 50 ms and tripped the line task's heartbeat). */
static uint32_t best_fit(uint32_t const * p_w, uint16_t n, uint32_t const * p_e, bool const * p_wide,
                         bool const * p_speck, uint32_t inv, uint32_t bound)
{
    static uint32_t dp[BEST_N_MAX + 1u][BEST_T + 1u];
    static uint8_t  ed[BEST_N_MAX + 1u][BEST_T + 1u];
    static uint32_t rmin[BEST_N_MAX + 1u];     /* smallest cost in each row */
    uint32_t        best;
    uint16_t        i;
    uint16_t        j;

    /* Every defect moves the two indexes apart by 2, so only a band of
       cells can be reached; only those are used. */
    for (i = 0u; i <= n; i++)
    {
        for (j = (i > BEST_BAND) ? (uint16_t)(i - BEST_BAND) : 0u; (j <= BEST_T) && (j <= (i + BEST_BAND)); j++)
        {
            dp[i][j] = BEST_INF;
            ed[i][j] = 0u;
        }
        rmin[i] = BEST_INF;
    }
    dp[0][0] = 0u;
    rmin[0]  = 0u;
    dp[0][2] = BEST_EDIT;               /* first bar and space lost */
    ed[0][2] = 1u;
    for (i = 0u; i < n; i++)
    {
        /* Every path to the end passes a filled cell of row i, i + 1 or
           i + 2 (a step moves on 1 or 3 measured elements, and rows
           before i are done); costs only grow along a path. */
        uint32_t c_open = rmin[i];

        c_open = (((i + 1u) <= n) && (rmin[i + 1u] < c_open)) ? rmin[i + 1u] : c_open;
        c_open = (((i + 2u) <= n) && (rmin[i + 2u] < c_open)) ? rmin[i + 2u] : c_open;
        if (c_open >= bound)
        {
            return BEST_INF;
        }
        for (j = (i > BEST_BAND) ? (uint16_t)(i - BEST_BAND) : 0u; (j < BEST_T) && (j <= (i + BEST_BAND)); j++)
        {
            uint32_t const c0 = dp[i][j];
            uint8_t const  e0 = ed[i][j];

            if (c0 >= BEST_INF)
            {
                continue;
            }
            if (!p_speck[i])
            {   /* 1 : 1 */
                uint32_t const c = c0 + sq_err(p_w[i], p_e[j], inv);

                if (c < dp[i + 1u][j + 1u])
                {
                    dp[i + 1u][j + 1u] = c;
                    rmin[i + 1u] = (c < rmin[i + 1u]) ? c : rmin[i + 1u];
                    ed[i + 1u][j + 1u] = e0;
                }
            }
            if ((e0 < BEST_MAX_EDITS) && ((i + 3u) <= n) && in_band((uint16_t)(i + 3u), (uint16_t)(j + 1u)) &&
                p_speck[i + 1u])
            {   /* 3 measured : 1 printed */
                uint32_t const c = c0 + BEST_EDIT +
                                   sq_err(p_w[i] + p_w[i + 1u] + p_w[i + 2u], p_e[j], inv);

                if (c < dp[i + 3u][j + 1u])
                {
                    dp[i + 3u][j + 1u] = c;
                    rmin[i + 3u] = (c < rmin[i + 3u]) ? c : rmin[i + 3u];
                    ed[i + 3u][j + 1u] = (uint8_t)(e0 + 1u);
                }
            }
            if ((e0 < BEST_MAX_EDITS) && ((j + 3u) <= BEST_T) && in_band((uint16_t)(i + 1u), (uint16_t)(j + 3u)) &&
                !p_wide[j + 1u])
            {   /* 1 measured : 3 printed */
                uint32_t const c = c0 + BEST_EDIT +
                                   sq_err(p_w[i], p_e[j] + p_e[j + 1u] + p_e[j + 2u], inv);

                if (c < dp[i + 1u][j + 3u])
                {
                    dp[i + 1u][j + 3u] = c;
                    rmin[i + 1u] = (c < rmin[i + 1u]) ? c : rmin[i + 1u];
                    ed[i + 1u][j + 3u] = (uint8_t)(e0 + 1u);
                }
            }
        }
    }
    best = in_band(n, BEST_T) ? dp[n][BEST_T] : BEST_INF;
    if (in_band(n, (uint16_t)(BEST_T - 2u)) && (ed[n][BEST_T - 2u] < BEST_MAX_EDITS) &&
        (dp[n][BEST_T - 2u] < BEST_INF) &&
        ((dp[n][BEST_T - 2u] + BEST_EDIT) < best))
    {
        best = dp[n][BEST_T - 2u] + BEST_EDIT;      /* last space and bar lost */
    }
    return best;
}

code39_result_t code39_decode_one_best(uint32_t const * p_widths, uint16_t n,
                                       char * p_char, bool * p_b_reversed)
{
    uint32_t       lv[4];               /* bar n, bar W, space n, space W */
    uint32_t       e[BEST_T];
    bool           wd[BEST_T];
    bool           sp[BEST_N_MAX];      /* measured element is a speck */
    uint16_t const star = code39_pattern('*');
    uint32_t       nar;
    uint32_t       inv;
    uint32_t       c1 = BEST_INF;       /* best cost and its character */
    uint32_t       c2 = BEST_INF;       /* best cost of any other character */
    char           ch1 = '\0';
    bool           b_rev1 = false;
    size_t         t;
    uint8_t        d;

    *p_char       = '\0';
    *p_b_reversed = false;
    if (n < 9u)
    {
        return CODE39_NOT_BARCODE;
    }
    if ((n < BEST_N_MIN) || (n > BEST_N_MAX) || ((n % 2u) == 0u))
    {
        return CODE39_ERR_COUNT;        /* a read starts and ends with a bar */
    }
    best_levels(p_widths, n, lv);
    nar = (lv[0] + lv[2]) / 2u;
    if ((nar == 0u) || (lv[1] <= lv[0]) || (lv[3] <= lv[2]))
    {
        return CODE39_ERR_RATIO;
    }
    inv = (100u << 16) / nar;
    for (t = 0u; t < n; t++)
    {
        sp[t] = is_speck(p_widths[t], (uint16_t)t, lv);
    }
    for (t = 0u; t < TABLE_LEN; t++)
    {
        uint16_t const pat = bits_of(k_table[t].p_bits);
        uint8_t        k;

        if (k_table[t].c == '*')
        {
            continue;
        }
        /* *X*: start, gap, X, gap, stop -- the gaps are narrow spaces */
        for (k = 0u; k < BEST_T; k++)
        {
            uint8_t const  c   = (uint8_t)(k / 10u);
            uint8_t const  pos = (uint8_t)(k % 10u);
            uint16_t const p   = (c == 1u) ? pat : star;
            bool const     b_w = (pos < 9u) && (((p >> (8u - pos)) & 1u) != 0u);

            e[k]  = lv[(((k % 2u) != 0u) ? 2u : 0u) + (b_w ? 1u : 0u)];
            wd[k] = b_w;
        }
        for (d = 0u; d < 2u; d++)
        {
            uint32_t cost;

            if (d == 1u)
            {
                for (k = 0u; k < (BEST_T / 2u); k++)
                {
                    uint32_t const x  = e[k];
                    bool const     bx = wd[k];

                    e[k]  = e[BEST_T - 1u - k];
                    wd[k] = wd[BEST_T - 1u - k];
                    e[BEST_T - 1u - k]  = x;
                    wd[BEST_T - 1u - k] = bx;
                }
            }
            /* A fit costing c2 or more changes neither result, and the
               runner-up only matters below the margin times the largest
               best that is accepted. */
            cost = best_fit(p_widths, n, e, wd, sp, inv, (c2 < BEST_BOUND_MAX) ? c2 : BEST_BOUND_MAX);
            if (cost < c1)
            {
                if (k_table[t].c != ch1)
                {
                    c2 = c1;
                }
                c1     = cost;
                ch1    = k_table[t].c;
                b_rev1 = (d == 1u);
            }
            else if ((cost < c2) && (k_table[t].c != ch1))
            {
                c2 = cost;
            }
        }
    }
    if ((ch1 == '\0') || (c1 > ((uint32_t)BEST_ELEM_MAX * BEST_T)) ||
        (((uint64_t)c2 * 10u) < ((uint64_t)c1 * BEST_MARGIN_X10)))
    {
        return CODE39_ERR_PATTERN;
    }
    *p_char       = ch1;
    *p_b_reversed = b_rev1;
    return CODE39_OK;
}
