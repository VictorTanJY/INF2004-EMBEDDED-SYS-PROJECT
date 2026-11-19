/*
 *  line_barcode.c
 *  Subsystem 3 -- barcode capture.  See line_barcode.h.
 */

#include "line_barcode.h"

static void pre_reset (bc_capture_t * p_bc)
{
    p_bc->pre_n   = 0u;
    p_bc->pre_i   = 0u;
    p_bc->pre_cur = 0u;
    p_bc->pre_um  = 0u;
}

/* While armed: keep the darkness record of the last BC_PRE_BINS bins. */
static void pre_feed (bc_capture_t * p_bc, uint16_t norm, uint32_t step_um)
{
    p_bc->pre_cur = (norm > p_bc->pre_cur) ? norm : p_bc->pre_cur;
    p_bc->pre_um += step_um;
    while (p_bc->pre_um >= BC_PROF_BIN_UM)
    {
        p_bc->pre_um -= BC_PROF_BIN_UM;
        p_bc->pre[p_bc->pre_i] = p_bc->pre_cur;
        p_bc->pre_i            = (uint16_t)((p_bc->pre_i + 1u) % BC_PRE_BINS);
        if (p_bc->pre_n < BC_PRE_BINS)
        {
            p_bc->pre_n++;
        }
        p_bc->pre_cur = norm;
    }
}

void bc_init (bc_capture_t * p_bc)
{
    p_bc->state       = (uint8_t)LINE_BC_WAIT_QUIET;
    p_bc->b_black     = false;
    p_bc->run_um      = 0u;
    p_bc->white_um    = 0u;
    p_bc->total_um    = 0u;
    p_bc->max_elem_um = 0u;
    p_bc->cooldown_um = 0u;
    p_bc->n_elems     = 0u;
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

static void start_cooldown (bc_capture_t * p_bc)
{
    p_bc->state       = (uint8_t)LINE_BC_COOLDOWN;
    p_bc->cooldown_um = 0u;
}

void bc_prof_levels (uint16_t const * p_prof, uint16_t n_bins,
                     uint16_t * p_white, uint16_t * p_dark)
{
    uint16_t hist[64];
    uint32_t sum = 0u;
    uint16_t idx;
    bool     b_white = false;

    for (idx = 0u; idx < 64u; idx++)
    {
        hist[idx] = 0u;
    }
    for (idx = 0u; idx < n_bins; idx++)
    {
        uint16_t const bin = (uint16_t)(p_prof[idx] / 16u);

        hist[(bin < 64u) ? bin : 63u]++;
    }
    *p_white = 0u;
    *p_dark  = 1000u;
    for (idx = 0u; idx < 64u; idx++)
    {
        sum += hist[idx];
        if (!b_white
            && ((sum * 100u) >= ((uint32_t)n_bins * BC_PROF_WHITE_PCT)))
        {
            *p_white = (uint16_t)((idx * 16u) + 8u);
            b_white  = true;
        }
        if ((sum * 100u) >= ((uint32_t)n_bins * BC_PROF_DARK_PCT))
        {
            *p_dark = (uint16_t)((idx * 16u) + 8u);
            break;
        }
    }
}

/* Runs (in bins, bar first) to widths: runs shorter than
   BC_PROF_MIN_BINS merge into their neighbours (a speck inside a bar or a
   glint inside a space), keeping the bar / space alternation. */
static uint16_t runs_to_widths (uint16_t * p_runs, uint16_t n_runs,
                                uint32_t * p_widths, uint16_t max)
{
    uint16_t kdx = 0u;
    uint16_t idx;
    uint16_t out = 0u;

    while (kdx < n_runs)
    {
        if ((p_runs[kdx] < BC_PROF_MIN_BINS) && (kdx > 0u)
            && ((kdx + 1u) < n_runs))
        {
            p_runs[kdx - 1u] =
                (uint16_t)(p_runs[kdx - 1u] + p_runs[kdx] + p_runs[kdx + 1u]);
            for (idx = kdx; (idx + 2u) < n_runs; idx++)
            {
                p_runs[idx] = p_runs[idx + 2u];
            }
            n_runs = (uint16_t)(n_runs - 2u);
        }
        else
        {
            kdx++;
        }
    }
    for (kdx = 0u; (kdx < n_runs) && (out < max); kdx++)
    {
        p_widths[out] = (uint32_t)p_runs[kdx] * BC_PROF_BIN_UM;
        out++;
    }
    return out;
}

uint16_t bc_prof_elements (uint16_t const * p_prof, uint16_t n_bins,
                           uint16_t thr, uint16_t hyst, uint32_t * p_widths,
                           uint16_t max)
{
    static uint16_t runs[BC_PROF_BINS / 2u];
    uint16_t        n_runs  = 0u;
    uint16_t        idx     = 0u;
    uint16_t        run     = 0u;
    bool            b_black = true;

    /* skip to the first bar */
    while ((idx < n_bins) && (p_prof[idx] < (uint16_t)(thr + hyst)))
    {
        idx++;
    }
    for (; idx < n_bins; idx++)
    {
        bool const b_now = b_black ? (p_prof[idx] + hyst >= thr)
                                   : (p_prof[idx] >= (uint16_t)(thr + hyst));

        if ((b_now != b_black) && (run > 0u))
        {
            if (n_runs < (uint16_t)(sizeof(runs) / sizeof(runs[0])))
            {
                runs[n_runs] = run;
                n_runs++;
            }
            run     = 0u;
            b_black = b_now;
        }
        run++;
    }
    if (b_black && (run > 0u)
        && (n_runs < (uint16_t)(sizeof(runs) / sizeof(runs[0]))))
    {
        runs[n_runs] = run; /* a bar ran to the end: keep it  */
        n_runs++;
    }
    return runs_to_widths(runs, n_runs, p_widths, max);
}

uint16_t bc_prof_swing (uint16_t const * p_prof, uint16_t n_bins,
                        uint16_t delta, uint32_t * p_widths, uint16_t max)
{
    static uint16_t runs[BC_PROF_BINS / 2u];
    uint16_t        n_runs = 0u;
    uint16_t        idx;
    uint16_t        run = 0u;
    uint16_t        ext;
    bool            b_black    = false;
    bool            b_bar_seen = false;

    if (0u == n_bins)
    {
        return 0u;
    }
    ext = p_prof[0];
    for (idx = 0u; idx < n_bins; idx++)
    {
        uint16_t const level = p_prof[idx];
        bool           b_now = b_black;

        if (b_black)
        {
            ext = (level > ext) ? level : ext;
            if (((uint32_t)level + delta) < ext)
            {
                b_now = false;
                ext   = level;
            }
        }
        else
        {
            ext = (level < ext) ? level : ext;
            if ((uint32_t)level > ((uint32_t)ext + delta))
            {
                b_now = true;
                ext   = level;
            }
        }
        if (b_now != b_black)
        {
            if (b_bar_seen
                && (n_runs < (uint16_t)(sizeof(runs) / sizeof(runs[0]))))
            {
                runs[n_runs] = run;
                n_runs++;
            }
            b_bar_seen = b_bar_seen || b_now;
            run        = 0u;
            b_black    = b_now;
        }
        run++;
    }
    if (b_black && (run > 0u)
        && (n_runs < (uint16_t)(sizeof(runs) / sizeof(runs[0]))))
    {
        runs[n_runs] = run; /* a bar ran to the end: keep it  */
        n_runs++;
    }
    return runs_to_widths(runs, n_runs, p_widths, max);
}

/* Elements found again from the darkness profile at threshold k (1 mid,
   2 low for faint thin bars) against this code's own levels, or k 3 by
   swing sized to this code (BC_ADAPT_*); 0 if none or not useful. */
static uint16_t prof_read (bc_capture_t const * p_bc, uint8_t kdx,
                           uint32_t * p_w)
{
    uint16_t white;
    uint16_t dark;
    uint16_t span;
    uint16_t thr;

    if (0u == p_bc->prof_n)
    {
        return 0u;
    }
    bc_prof_levels(p_bc->prof, p_bc->prof_n, &white, &dark);
    if (dark <= white)
    {
        return 0u;
    }
    span = (uint16_t)(dark - white);
    if (kdx >= 3u)
    {
        uint32_t const pct =
            (3u == kdx) ? BC_ADAPT_PCT
                        : ((4u == kdx) ? BC_ADAPT_PCT2 : BC_ADAPT_PCT3);
        uint32_t const noise = (p_bc->q_hi > p_bc->q_lo)
                                   ? (uint32_t)(p_bc->q_hi - p_bc->q_lo)
                                   : 0u;
        uint32_t       delta = ((uint32_t)span * pct) / 100u;

        delta = (delta < (noise * BC_ADAPT_NOISE_X))
                    ? (noise * BC_ADAPT_NOISE_X)
                    : delta;
        delta = (delta < BC_ADAPT_MIN) ? BC_ADAPT_MIN : delta;
        if (delta >= (uint32_t)BC_DELTA)
        {
            return 0u; /* no smaller than the first reading */
        }
        return bc_prof_swing(p_bc->prof, p_bc->prof_n, (uint16_t)delta, p_w,
                             (uint16_t)BC_MAX_ELEMENTS);
    }
    thr = (uint16_t)(white
                     + ((1u == kdx) ? (span / 2u) : (span / BC_PROF_LOW_DIV)));
    return bc_prof_elements(p_bc->prof, p_bc->prof_n, thr,
                            (uint16_t)(span / 16u), p_w,
                            (uint16_t)BC_MAX_ELEMENTS);
}

/* 14 Nov: a speck before the start or after the stop adds a bar and a
   space at that end, so the read has 31 or 33 elements and decodes as
   nothing (two car reads of "*A*" on 14 Nov, both with a 1.4 mm bar
   outside the code).  An end bar under BC_SPECK_PCT % of the read's
   median inner bar is dropped with the space beside it; *p_first is
   where the rest starts, and the count of the rest is returned (n if
   nothing was dropped).  Most bars are narrow, so the median is one. */
static uint16_t trim_specks (uint32_t const * p_w, uint16_t n_elems,
                             uint16_t * p_first)
{
    static uint32_t sorted[(BC_MAX_ELEMENTS / 2u) + 1u];
    uint16_t        mid = 0u;
    uint16_t        idx;
    uint16_t        jdx;
    uint16_t        last = n_elems;
    uint32_t        med;

    *p_first = 0u;
    if ((n_elems < 11u) || (0u == (n_elems % 2u)))
    {
        return n_elems; /* a read starts and ends with a bar */
    }
    for (idx = 2u; (idx + 2u) < n_elems;
         idx += 2u) /* inner bars, insertion sort */
    {
        uint32_t const width = p_w[idx];

        for (jdx = mid; (jdx > 0u) && (sorted[jdx - 1u] > width); jdx--)
        {
            sorted[jdx] = sorted[jdx - 1u];
        }
        sorted[jdx] = width;
        mid++;
    }
    med = sorted[mid / 2u];
    if (((uint64_t)p_w[0] * 100u) < ((uint64_t)med * BC_SPECK_PCT))
    {
        *p_first = 2u;
    }
    if (((uint64_t)p_w[n_elems - 1u] * 100u) < ((uint64_t)med * BC_SPECK_PCT))
    {
        last = (uint16_t)(n_elems - 2u);
    }
    return (uint16_t)(last - *p_first);
}

/* Exact decode (with the end repair) of one reading; if it fails, again
   without an end speck.  *p_b_trim says the second one decoded.  The
   first result is returned on failure (NOT_BARCODE must stay itself). */
static code39_result_t decode_exact (uint32_t const * p_w, uint16_t n_elems,
                                     char * p_ch, bool * pb_rev, bool * pb_trim)
{
    code39_result_t result =
        code39_decode_one_repair(p_w, n_elems, BC_RATIO_MIN_X10, p_ch, pb_rev);

    *pb_trim = false;
    if ((result != CODE39_OK) && (result != CODE39_NOT_BARCODE))
    {
        uint16_t       first;
        uint16_t const n_read = trim_specks(p_w, n_elems, &first);

        if ((n_read != n_elems)
            && (CODE39_OK
                == code39_decode_one_repair(&p_w[first], n_read,
                                            BC_RATIO_MIN_X10, p_ch, pb_rev)))
        {
            *pb_trim = true;
            result   = CODE39_OK;
        }
    }
    return result;
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
static bc_result_t finish (bc_capture_t * p_bc, bc_out_t * p_out)
{
    static uint32_t read_widths[BC_MAX_ELEMENTS + 1u];
    code39_result_t result;
    uint8_t         kdx;
    bool            b_trim;

    p_out->elements = p_bc->n_elems;
    p_out->length_um =
        p_bc->total_um - p_bc->run_um; /* without the trailing white */
    p_out->peak_norm = p_bc->peak;
    p_bc->pass       = 0u;
    result           = decode_exact(p_bc->widths, p_bc->n_elems, &p_out->chr,
                                    &p_out->b_reversed, &b_trim);
    if (b_trim)
    {
        p_bc->pass = 5u;
    }
    for (kdx = 1u; (kdx <= 5u) && (result != CODE39_OK); kdx++)
    {
        uint16_t const n_read = prof_read(p_bc, kdx, read_widths);
        char           chr;
        bool           b_rev;

        if ((n_read > 0u)
            && (CODE39_OK
                == decode_exact(read_widths, n_read, &chr, &b_rev, &b_trim)))
        {
            p_out->chr        = chr;
            p_out->b_reversed = b_rev;
            p_out->elements   = n_read;
            p_bc->pass        = b_trim ? 5u : ((kdx > 3u) ? 3u : kdx);
            result            = CODE39_OK;
        }
    }
    if ((result != CODE39_OK) && (result != CODE39_NOT_BARCODE))
    {
        uint8_t  k_pick = BC_READINGS;
        uint16_t d_pick = 0xFFFFu;

        for (kdx = 0u; kdx < BC_READINGS; kdx++)
        {
            uint16_t       first;
            uint16_t const n_read = bc_reading(p_bc, kdx, read_widths);
            uint16_t const n_trim = trim_specks(read_widths, n_read, &first);
            uint16_t const diff   = (n_trim > 29u) ? (uint16_t)(n_trim - 29u)
                                                   : (uint16_t)(29u - n_trim);

            if ((n_trim >= 25u) && (n_trim <= 33u) && ((n_trim % 2u) != 0u)
                && (diff < d_pick))
            {
                d_pick = diff;
                k_pick = kdx;
            }
        }
        if (k_pick < BC_READINGS)
        {
            uint16_t       first;
            uint16_t const n_read = bc_reading(p_bc, k_pick, read_widths);
            uint16_t const n_trim = trim_specks(read_widths, n_read, &first);
            char           chr;
            bool           b_rev;

            if (CODE39_OK
                == code39_decode_one_best(&read_widths[first], n_trim, &chr,
                                          &b_rev))
            {
                p_out->chr        = chr;
                p_out->b_reversed = b_rev;
                p_out->elements   = n_trim;
                p_bc->pass        = (n_trim != n_read) ? 6u : 4u;
                result            = CODE39_OK;
            }
        }
    }
    if (CODE39_NOT_BARCODE == result)
    {
        /* A crossing line or a speck: not a barcode, not an error.  The
           white that ended it already counts as the next quiet zone. */
        p_bc->state = (uint8_t)LINE_BC_ARMED;
        return BC_NONE;
    }
    start_cooldown(p_bc);
    if (CODE39_OK == result)
    {
        return BC_VALID;
    }
    p_out->reason = (uint8_t)result;
    return BC_REJECT;
}

uint16_t bc_reading (bc_capture_t const * p_bc, uint8_t reading,
                     uint32_t * p_widths)
{
    uint16_t kdx;

    if (0u == reading)
    {
        for (kdx = 0u; (kdx < p_bc->n_elems) && (kdx <= BC_MAX_ELEMENTS); kdx++)
        {
            p_widths[kdx] = p_bc->widths[kdx];
        }
        return kdx;
    }
    return prof_read(p_bc, reading, p_widths);
}

bc_result_t bc_sample (bc_capture_t * p_bc, uint16_t norm, uint32_t step_um,
                       bc_out_t * p_out)
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
        p_bc->ext = (norm > p_bc->ext) ? norm : p_bc->ext; /* darkest so far */
        if (((uint32_t)norm + BC_DELTA) < p_bc->ext)
        {
            p_bc->b_black = false;
            p_bc->ext     = norm;
        }
    }
    else
    {
        p_bc->ext = (norm < p_bc->ext) ? norm : p_bc->ext; /* lightest so far */
        if ((norm >= BC_DELTA_MIN)
            && ((uint32_t)norm > ((uint32_t)p_bc->ext + BC_DELTA)))
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
                p_bc->q_lo  = 0u;
                p_bc->q_hi  = 0u;
                p_bc->q_wlo = 0xFFFFu;
                p_bc->q_whi = 0u;
            }
        break;

        case LINE_BC_ARMED:
            if (!p_bc->b_black)
            {
                /* white-paper levels, one BC_QUIET_UM window at a time */
                p_bc->q_wlo = (norm < p_bc->q_wlo) ? norm : p_bc->q_wlo;
                p_bc->q_whi = (norm > p_bc->q_whi) ? norm : p_bc->q_whi;
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
                uint16_t kdx;

                p_bc->state       = (uint8_t)LINE_BC_CAPTURING;
                p_bc->n_elems     = 0u;
                p_bc->run_um      = step_um;
                p_bc->total_um    = step_um;
                p_bc->max_elem_um = 0u;
                p_bc->peak        = norm;
                /* the darkness record starts with what came just before,
                   oldest first (faint leading bars) */
                for (kdx = 0u; kdx < p_bc->pre_n; kdx++)
                {
                    p_bc->prof[kdx] =
                        p_bc->pre[(uint16_t)((p_bc->pre_i + BC_PRE_BINS
                                              - p_bc->pre_n + kdx)
                                             % BC_PRE_BINS)];
                }
                p_bc->prof[p_bc->pre_n] = norm;
                p_bc->prof_n            = (uint16_t)(p_bc->pre_n + 1u);
                p_bc->bin_um            = step_um;
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
            while ((p_bc->bin_um >= BC_PROF_BIN_UM)
                   && (p_bc->prof_n < BC_PROF_BINS))
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
                if (p_bc->n_elems >= BC_MAX_ELEMENTS)
                {
                    p_out->reason    = (uint8_t)CODE39_ERR_TOO_MANY;
                    p_out->elements  = p_bc->n_elems;
                    p_out->length_um = p_bc->total_um;
                    start_cooldown(p_bc);
                    return BC_REJECT;
                }
                p_bc->widths[p_bc->n_elems] = p_bc->run_um;
                p_bc->n_elems++;
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
                p_out->elements  = p_bc->n_elems;
                p_out->length_um = p_bc->total_um;
                start_cooldown(p_bc);
                return BC_REJECT;
            }
        break;

        default: /* LINE_BC_COOLDOWN */
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

/*** end of file ***/
