/*
 *  line_barcode.c
 *  Subsystem 3 -- barcode capture.  See line_barcode.h.
 */

#include "line_barcode.h"

static void pre_reset(bc_capture_t * p_bc)
{
    p_bc->pre_n   = 0u;
    p_bc->pre_i   = 0u;
    p_bc->pre_cur = 0u;
    p_bc->pre_um  = 0u;
}

/* While armed: keep the darkness record of the last BC_PRE_BINS bins. */
static void pre_feed(bc_capture_t * p_bc, uint16_t norm, uint32_t step_um)
{
    p_bc->pre_cur = (norm > p_bc->pre_cur) ? norm : p_bc->pre_cur;
    p_bc->pre_um += step_um;
    while (p_bc->pre_um >= BC_PROF_BIN_UM)
    {
        p_bc->pre_um -= BC_PROF_BIN_UM;
        p_bc->pre[p_bc->pre_i] = p_bc->pre_cur;
        p_bc->pre_i = (uint16_t)((p_bc->pre_i + 1u) % BC_PRE_BINS);
        if (p_bc->pre_n < BC_PRE_BINS)
        {
            p_bc->pre_n++;
        }
        p_bc->pre_cur = norm;
    }
}

void bc_init(bc_capture_t * p_bc)
{
    p_bc->state       = (uint8_t)LINE_BC_WAIT_QUIET;
    p_bc->b_black     = false;
    p_bc->run_um      = 0u;
    p_bc->white_um    = 0u;
    p_bc->total_um    = 0u;
    p_bc->max_elem_um = 0u;
    p_bc->cooldown_um = 0u;
    p_bc->n           = 0u;
    p_bc->peak        = 0u;
    p_bc->ext         = 0u;
    p_bc->prof_n      = 0u;
    p_bc->bin_um      = 0u;
    p_bc->pass        = 0u;
    p_bc->q_lo        = 0u;
    p_bc->q_hi        = 0u;
    p_bc->q_wlo       = 0xFFFFu;
    p_bc->q_whi       = 0u;
    pre_reset(p_bc);
}

static void start_cooldown(bc_capture_t * p_bc)
{
    p_bc->state       = (uint8_t)LINE_BC_COOLDOWN;
    p_bc->cooldown_um = 0u;
}

void bc_prof_levels(uint16_t const * p_prof, uint16_t n_bins, uint16_t * p_white, uint16_t * p_dark)
{
    uint16_t hist[64];
    uint32_t sum = 0u;
    uint16_t i;
    bool     b_white = false;

    for (i = 0u; i < 64u; i++)
    {
        hist[i] = 0u;
    }
    for (i = 0u; i < n_bins; i++)
    {
        uint16_t const b = (uint16_t)(p_prof[i] / 16u);

        hist[(b < 64u) ? b : 63u]++;
    }
    *p_white = 0u;
    *p_dark  = 1000u;
    for (i = 0u; i < 64u; i++)
    {
        sum += hist[i];
        if (!b_white && ((sum * 100u) >= ((uint32_t)n_bins * BC_PROF_WHITE_PCT)))
        {
            *p_white = (uint16_t)((i * 16u) + 8u);
            b_white  = true;
        }
        if ((sum * 100u) >= ((uint32_t)n_bins * BC_PROF_DARK_PCT))
        {
            *p_dark = (uint16_t)((i * 16u) + 8u);
            break;
        }
    }
}

/* Runs (in bins, bar first) to widths: runs shorter than
   BC_PROF_MIN_BINS merge into their neighbours (a speck inside a bar or a
   glint inside a space), keeping the bar / space alternation. */
static uint16_t runs_to_widths(uint16_t * p_runs, uint16_t nr, uint32_t * p_widths, uint16_t max)
{
    uint16_t k = 0u;
    uint16_t i;
    uint16_t out = 0u;

    while (k < nr)
    {
        if ((p_runs[k] < BC_PROF_MIN_BINS) && (k > 0u) && ((k + 1u) < nr))
        {
            p_runs[k - 1u] = (uint16_t)(p_runs[k - 1u] + p_runs[k] + p_runs[k + 1u]);
            for (i = k; (i + 2u) < nr; i++)
            {
                p_runs[i] = p_runs[i + 2u];
            }
            nr = (uint16_t)(nr - 2u);
        }
        else
        {
            k++;
        }
    }
    for (k = 0u; (k < nr) && (out < max); k++)
    {
        p_widths[out] = (uint32_t)p_runs[k] * BC_PROF_BIN_UM;
        out++;
    }
    return out;
}

uint16_t bc_prof_elements(uint16_t const * p_prof, uint16_t n_bins, uint16_t thr, uint16_t hyst,
                          uint32_t * p_widths, uint16_t max)
{
    static uint16_t runs[BC_PROF_BINS / 2u];
    uint16_t nr = 0u;
    uint16_t i = 0u;
    uint16_t run = 0u;
    bool     b_black = true;

    /* skip to the first bar */
    while ((i < n_bins) && (p_prof[i] < (uint16_t)(thr + hyst)))
    {
        i++;
    }
    for (; i < n_bins; i++)
    {
        bool const b_now = b_black ? (p_prof[i] + hyst >= thr) : (p_prof[i] >= (uint16_t)(thr + hyst));

        if ((b_now != b_black) && (run > 0u))
        {
            if (nr < (uint16_t)(sizeof(runs) / sizeof(runs[0])))
            {
                runs[nr] = run;
                nr++;
            }
            run     = 0u;
            b_black = b_now;
        }
        run++;
    }
    if (b_black && (run > 0u) && (nr < (uint16_t)(sizeof(runs) / sizeof(runs[0]))))
    {
        runs[nr] = run;                 /* a bar ran to the end: keep it  */
        nr++;
    }
    return runs_to_widths(runs, nr, p_widths, max);
}

uint16_t bc_prof_swing(uint16_t const * p_prof, uint16_t n_bins, uint16_t delta,
                       uint32_t * p_widths, uint16_t max)
{
    static uint16_t runs[BC_PROF_BINS / 2u];
    uint16_t nr = 0u;
    uint16_t i;
    uint16_t run = 0u;
    uint16_t ext;
    bool     b_black = false;
    bool     b_bar_seen = false;

    if (n_bins == 0u)
    {
        return 0u;
    }
    ext = p_prof[0];
    for (i = 0u; i < n_bins; i++)
    {
        uint16_t const v = p_prof[i];
        bool           b_now = b_black;

        if (b_black)
        {
            ext = (v > ext) ? v : ext;
            if (((uint32_t)v + delta) < ext)
            {
                b_now = false;
                ext   = v;
            }
        }
        else
        {
            ext = (v < ext) ? v : ext;
            if ((uint32_t)v > ((uint32_t)ext + delta))
            {
                b_now = true;
                ext   = v;
            }
        }
        if (b_now != b_black)
        {
            if (b_bar_seen && (nr < (uint16_t)(sizeof(runs) / sizeof(runs[0]))))
            {
                runs[nr] = run;
                nr++;
            }
            b_bar_seen = b_bar_seen || b_now;
            run     = 0u;
            b_black = b_now;
        }
        run++;
    }
    if (b_black && (run > 0u) && (nr < (uint16_t)(sizeof(runs) / sizeof(runs[0]))))
    {
        runs[nr] = run;                 /* a bar ran to the end: keep it  */
        nr++;
    }
    return runs_to_widths(runs, nr, p_widths, max);
}

/* Elements found again from the darkness profile at threshold k (1 mid,
   2 low for faint thin bars) against this code's own levels, or k 3 by
   swing sized to this code (BC_ADAPT_*); 0 if none or not useful. */
static uint16_t prof_read(bc_capture_t const * p_bc, uint8_t k, uint32_t * p_w)
{
    uint16_t white;
    uint16_t dark;
    uint16_t span;
    uint16_t thr;

    if (p_bc->prof_n == 0u)
    {
        return 0u;
    }
    bc_prof_levels(p_bc->prof, p_bc->prof_n, &white, &dark);
    if (dark <= white)
    {
        return 0u;
    }
    span = (uint16_t)(dark - white);
    if (k >= 3u)
    {
        uint32_t const pct   = (k == 3u) ? BC_ADAPT_PCT : ((k == 4u) ? BC_ADAPT_PCT2 : BC_ADAPT_PCT3);
        uint32_t const noise = (p_bc->q_hi > p_bc->q_lo) ? (uint32_t)(p_bc->q_hi - p_bc->q_lo) : 0u;
        uint32_t       delta = ((uint32_t)span * pct) / 100u;

        delta = (delta < (noise * BC_ADAPT_NOISE_X)) ? (noise * BC_ADAPT_NOISE_X) : delta;
        delta = (delta < BC_ADAPT_MIN) ? BC_ADAPT_MIN : delta;
        if (delta >= (uint32_t)BC_DELTA)
        {
            return 0u;                  /* no smaller than the first reading */
        }
        return bc_prof_swing(p_bc->prof, p_bc->prof_n, (uint16_t)delta, p_w, (uint16_t)BC_MAX_ELEMENTS);
    }
    thr  = (uint16_t)(white + ((k == 1u) ? (span / 2u) : (span / BC_PROF_LOW_DIV)));
    return bc_prof_elements(p_bc->prof, p_bc->prof_n, thr, (uint16_t)(span / 16u),
                            p_w, (uint16_t)BC_MAX_ELEMENTS);
}

/* 14 Nov: a speck before the start or after the stop adds a bar and a
   space at that end, so the read has 31 or 33 elements and decodes as
   nothing (two car reads of "*A*" on 14 Nov, both with a 1.4 mm bar
   outside the code).  An end bar under BC_SPECK_PCT % of the read's
   median inner bar is dropped with the space beside it; *p_first is
   where the rest starts, and the count of the rest is returned (n if
   nothing was dropped).  Most bars are narrow, so the median is one. */
static uint16_t trim_specks(uint32_t const * p_w, uint16_t n, uint16_t * p_first)
{
    static uint32_t s[(BC_MAX_ELEMENTS / 2u) + 1u];
    uint16_t        m = 0u;
    uint16_t        i;
    uint16_t        j;
    uint16_t        last = n;
    uint32_t        med;

    *p_first = 0u;
    if ((n < 11u) || ((n % 2u) == 0u))
    {
        return n;                       /* a read starts and ends with a bar */
    }
    for (i = 2u; (i + 2u) < n; i += 2u)  /* inner bars, insertion sort */
    {
        uint32_t const v = p_w[i];

        for (j = m; (j > 0u) && (s[j - 1u] > v); j--)
        {
            s[j] = s[j - 1u];
        }
        s[j] = v;
        m++;
    }
    med = s[m / 2u];
    if (((uint64_t)p_w[0] * 100u) < ((uint64_t)med * BC_SPECK_PCT))
    {
        *p_first = 2u;
    }
    if (((uint64_t)p_w[n - 1u] * 100u) < ((uint64_t)med * BC_SPECK_PCT))
    {
        last = (uint16_t)(n - 2u);
    }
    return (uint16_t)(last - *p_first);
}

/* Exact decode (with the end repair) of one reading; if it fails, again
   without an end speck.  *p_b_trim says the second one decoded.  The
   first result is returned on failure (NOT_BARCODE must stay itself). */
static code39_result_t decode_exact(uint32_t const * p_w, uint16_t n, char * p_ch, bool * p_b_rev,
                                    bool * p_b_trim)
{
    code39_result_t r = code39_decode_one_repair(p_w, n, BC_RATIO_MIN_X10, p_ch, p_b_rev);

    *p_b_trim = false;
    if ((r != CODE39_OK) && (r != CODE39_NOT_BARCODE))
    {
        uint16_t       first;
        uint16_t const n2 = trim_specks(p_w, n, &first);

        if ((n2 != n) &&
            (code39_decode_one_repair(&p_w[first], n2, BC_RATIO_MIN_X10, p_ch, p_b_rev) == CODE39_OK))
        {
            *p_b_trim = true;
            r = CODE39_OK;
        }
    }
    return r;
}

/* The trailing white run ended the code: decode what was captured.
   7 Nov: the swing reading first; if it does not decode, bars found
   again from the darkness profile against this code's own levels, at
   mid-way and then low for faint thin bars; each reading may also be
   repaired for a bar lost at either end.
   9 Nov: then by swing over the darkness record, the swing sized to this
   code's own contrast and noise (a light print): 20, 15 then 10 % of it,
   never under 4 x the noise; pass 3.
   8 Nov: if none decodes exactly, the best match (code39_decode_one_best:
   a bar split by a speck, or two thin elements run together); pass 4.
   14 Nov: every exact decode is tried again without an end speck (pass
   5).  The best match runs on ONE reading only, the one whose count
   (without end specks) is nearest the 29 of a code (pass 4, or 6 when a
   speck was dropped): on the car it ran on up to four, took over 50 ms
   and tripped the line task's heartbeat (SafetyTask stopped the car,
   once right after reading the A). */
static bc_result_t finish(bc_capture_t * p_bc, bc_out_t * p_out)
{
    static uint32_t w2[BC_MAX_ELEMENTS + 1u];
    code39_result_t r;
    uint8_t         k;
    bool            b_trim;

    p_out->elements  = p_bc->n;
    p_out->length_um = p_bc->total_um - p_bc->run_um;   /* without the trailing white */
    p_out->peak_norm = p_bc->peak;
    p_bc->pass       = 0u;
    r = decode_exact(p_bc->widths, p_bc->n, &p_out->ch, &p_out->b_reversed, &b_trim);
    if (b_trim)
    {
        p_bc->pass = 5u;
    }
    for (k = 1u; (k <= 5u) && (r != CODE39_OK); k++)
    {
        uint16_t const n2 = prof_read(p_bc, k, w2);
        char           ch;
        bool           b_rev;

        if ((n2 > 0u) && (decode_exact(w2, n2, &ch, &b_rev, &b_trim) == CODE39_OK))
        {
            p_out->ch         = ch;
            p_out->b_reversed = b_rev;
            p_out->elements   = n2;
            p_bc->pass        = b_trim ? 5u : ((k > 3u) ? 3u : k);
            r = CODE39_OK;
        }
    }
    if ((r != CODE39_OK) && (r != CODE39_NOT_BARCODE))
    {
        uint8_t  k_pick = BC_READINGS;
        uint16_t d_pick = 0xFFFFu;

        for (k = 0u; k < BC_READINGS; k++)
        {
            uint16_t       first;
            uint16_t const n2 = bc_reading(p_bc, k, w2);
            uint16_t const nt = trim_specks(w2, n2, &first);
            uint16_t const d  = (nt > 29u) ? (uint16_t)(nt - 29u) : (uint16_t)(29u - nt);

            if ((nt >= 25u) && (nt <= 33u) && ((nt % 2u) != 0u) && (d < d_pick))
            {
                d_pick = d;
                k_pick = k;
            }
        }
        if (k_pick < BC_READINGS)
        {
            uint16_t       first;
            uint16_t const n2 = bc_reading(p_bc, k_pick, w2);
            uint16_t const nt = trim_specks(w2, n2, &first);
            char           ch;
            bool           b_rev;

            if (code39_decode_one_best(&w2[first], nt, &ch, &b_rev) == CODE39_OK)
            {
                p_out->ch         = ch;
                p_out->b_reversed = b_rev;
                p_out->elements   = nt;
                p_bc->pass        = (nt != n2) ? 6u : 4u;
                r = CODE39_OK;
            }
        }
    }
    if (r == CODE39_NOT_BARCODE)
    {
        /* A crossing line or a speck: not a barcode, not an error.  The
           white that ended it already counts as the next quiet zone. */
        p_bc->state = (uint8_t)LINE_BC_ARMED;
        return BC_NONE;
    }
    start_cooldown(p_bc);
    if (r == CODE39_OK)
    {
        return BC_VALID;
    }
    p_out->reason = (uint8_t)r;
    return BC_REJECT;
}

uint16_t bc_reading(bc_capture_t const * p_bc, uint8_t reading, uint32_t * p_widths)
{
    uint16_t k;

    if (reading == 0u)
    {
        for (k = 0u; (k < p_bc->n) && (k <= BC_MAX_ELEMENTS); k++)
        {
            p_widths[k] = p_bc->widths[k];
        }
        return k;
    }
    return prof_read(p_bc, reading, p_widths);
}

bc_result_t bc_sample(bc_capture_t * p_bc, uint16_t norm, uint32_t step_um, bc_out_t * p_out)
{
    bool const b_was_black = p_bc->b_black;

    /* Bars and gaps by SWING (2 Nov): a level change needs the reading to
       move BC_DELTA from the extreme of the current element.  Fixed levels
       (350 / 220 before) missed every 3 mm element on the robot: under a
       sensor spot wider than 3 mm a narrow bar never got dark enough and a
       narrow gap between wide bars never got light enough, but both still
       swing clearly against their neighbours.  BC_DELTA is above the
       peak-to-peak sample noise, so noise alone cannot toggle the level.
       Rising and falling edges are both detected one BC_DELTA after the
       extreme, so for a full swing bar and gap widths stay unbiased. */
    if (p_bc->b_black)
    {
        p_bc->ext = (norm > p_bc->ext) ? norm : p_bc->ext;          /* darkest so far */
        if (((uint32_t)norm + BC_DELTA) < p_bc->ext)
        {
            p_bc->b_black = false;
            p_bc->ext     = norm;
        }
    }
    else
    {
        p_bc->ext = (norm < p_bc->ext) ? norm : p_bc->ext;          /* lightest so far */
        if ((norm >= BC_DELTA_MIN) && ((uint32_t)norm > ((uint32_t)p_bc->ext + BC_DELTA)))
        {
            p_bc->b_black = true;
            p_bc->ext     = norm;
        }
    }

    switch (p_bc->state)
    {
        case LINE_BC_WAIT_QUIET:
            p_bc->white_um = p_bc->b_black ? 0u : (p_bc->white_um + step_um);
            if (p_bc->white_um >= BC_QUIET_UM)
            {
                p_bc->state    = (uint8_t)LINE_BC_ARMED;
                p_bc->white_um = 0u;
                pre_reset(p_bc);
                p_bc->q_lo     = 0u;
                p_bc->q_hi     = 0u;
                p_bc->q_wlo    = 0xFFFFu;
                p_bc->q_whi    = 0u;
            }
            break;

        case LINE_BC_ARMED:
            if (!p_bc->b_black)
            {
                /* white-paper levels, one BC_QUIET_UM window at a time */
                p_bc->q_wlo     = (norm < p_bc->q_wlo) ? norm : p_bc->q_wlo;
                p_bc->q_whi     = (norm > p_bc->q_whi) ? norm : p_bc->q_whi;
                p_bc->white_um += step_um;
                if (p_bc->white_um >= BC_QUIET_UM)
                {
                    p_bc->q_lo     = p_bc->q_wlo;
                    p_bc->q_hi     = p_bc->q_whi;
                    p_bc->q_wlo    = 0xFFFFu;
                    p_bc->q_whi    = 0u;
                    p_bc->white_um = 0u;
                }
            }
            if (p_bc->b_black)
            {
                uint16_t k;

                p_bc->state       = (uint8_t)LINE_BC_CAPTURING;
                p_bc->n           = 0u;
                p_bc->run_um      = step_um;
                p_bc->total_um    = step_um;
                p_bc->max_elem_um = 0u;
                p_bc->peak        = norm;
                /* the darkness record starts with what came just before,
                   oldest first (faint leading bars) */
                for (k = 0u; k < p_bc->pre_n; k++)
                {
                    p_bc->prof[k] = p_bc->pre[(uint16_t)((p_bc->pre_i + BC_PRE_BINS - p_bc->pre_n + k) % BC_PRE_BINS)];
                }
                p_bc->prof[p_bc->pre_n] = norm;
                p_bc->prof_n      = (uint16_t)(p_bc->pre_n + 1u);
                p_bc->bin_um      = step_um;
                pre_reset(p_bc);
            }
            else
            {
                pre_feed(p_bc, norm, step_um);
            }
            break;

        case LINE_BC_CAPTURING:
            p_bc->total_um += step_um;
            /* darkness profile, one bin per BC_PROF_BIN_UM of travel */
            p_bc->bin_um += step_um;
            while ((p_bc->bin_um >= BC_PROF_BIN_UM) && (p_bc->prof_n < BC_PROF_BINS))
            {
                p_bc->bin_um -= BC_PROF_BIN_UM;
                p_bc->prof[p_bc->prof_n] = norm;
                p_bc->prof_n++;
            }
            if ((p_bc->prof_n > 0u) && (norm > p_bc->prof[p_bc->prof_n - 1u]))
            {
                p_bc->prof[p_bc->prof_n - 1u] = norm;
            }
            if (norm > p_bc->peak)
            {
                p_bc->peak = norm;
            }
            if (p_bc->b_black == b_was_black)
            {
                p_bc->run_um += step_um;
            }
            else
            {
                /* run ended: store it as an element */
                if (p_bc->n >= BC_MAX_ELEMENTS)
                {
                    p_out->reason    = (uint8_t)CODE39_ERR_TOO_MANY;
                    p_out->elements  = p_bc->n;
                    p_out->length_um = p_bc->total_um;
                    start_cooldown(p_bc);
                    return BC_REJECT;
                }
                p_bc->widths[p_bc->n] = p_bc->run_um;
                p_bc->n++;
                if (p_bc->run_um > p_bc->max_elem_um)
                {
                    p_bc->max_elem_um = p_bc->run_um;
                }
                p_bc->run_um = step_um;
            }
            if (!p_bc->b_black)
            {
                uint32_t end_um = (p_bc->max_elem_um * BC_END_FACTOR_X10) / 10u;

                if (end_um < BC_END_MIN_UM)
                {
                    end_um = BC_END_MIN_UM;
                }
                if (p_bc->run_um >= end_um)
                {
                    return finish(p_bc, p_out);
                }
            }
            if (p_bc->total_um > BC_MAX_LEN_UM)
            {
                p_out->reason    = (uint8_t)CODE39_ERR_TOO_LONG;
                p_out->elements  = p_bc->n;
                p_out->length_um = p_bc->total_um;
                start_cooldown(p_bc);
                return BC_REJECT;
            }
            break;

        default:                        /* LINE_BC_COOLDOWN */
            p_bc->cooldown_um += step_um;
            if (p_bc->cooldown_um >= BC_COOLDOWN_UM)
            {
                p_bc->state    = (uint8_t)LINE_BC_WAIT_QUIET;
                p_bc->white_um = 0u;
            }
            break;
    }
    return BC_NONE;
}
