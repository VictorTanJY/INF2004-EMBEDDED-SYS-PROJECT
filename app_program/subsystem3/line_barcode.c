/*
 *  line_barcode.c
 *  Subsystem 3 -- barcode capture.  See line_barcode.h.
 */

#include "line_barcode.h"

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

uint16_t bc_prof_elements(uint16_t const * p_prof, uint16_t n_bins, uint16_t thr, uint16_t hyst,
                          uint32_t * p_widths, uint16_t max)
{
    static uint16_t runs[BC_PROF_BINS / 2u];
    uint16_t nr = 0u;
    uint16_t i = 0u;
    uint16_t run = 0u;
    bool     b_black = true;
    uint16_t k;
    uint16_t out = 0u;

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
    /* merge runs shorter than BC_PROF_MIN_BINS into their neighbours
       (a speck inside a bar or a glint inside a space), keeping the
       bar / space alternation */
    k = 0u;
    while (k < nr)
    {
        if ((runs[k] < BC_PROF_MIN_BINS) && (k > 0u) && ((k + 1u) < nr))
        {
            runs[k - 1u] = (uint16_t)(runs[k - 1u] + runs[k] + runs[k + 1u]);
            for (i = k; (i + 2u) < nr; i++)
            {
                runs[i] = runs[i + 2u];
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
        p_widths[out] = (uint32_t)runs[k] * BC_PROF_BIN_UM;
        out++;
    }
    return out;
}

/* The trailing white run ended the code: decode what was captured.
   7 Nov: the swing reading first; if it does not decode, bars found
   again from the darkness profile against this code's own levels, at
   mid-way and then low for faint thin bars; each reading may also be
   repaired for a bar lost at either end. */
static bc_result_t finish(bc_capture_t * p_bc, bc_out_t * p_out)
{
    static uint32_t w2[BC_MAX_ELEMENTS + 1u];
    code39_result_t r;

    p_out->elements  = p_bc->n;
    p_out->length_um = p_bc->total_um - p_bc->run_um;   /* without the trailing white */
    p_out->peak_norm = p_bc->peak;
    p_bc->pass       = 0u;
    r = code39_decode_one_repair(p_bc->widths, p_bc->n, BC_RATIO_MIN_X10, &p_out->ch, &p_out->b_reversed);
    if ((r != CODE39_OK) && (p_bc->prof_n > 0u))
    {
        uint16_t white;
        uint16_t dark;
        uint8_t  k;

        bc_prof_levels(p_bc->prof, p_bc->prof_n, &white, &dark);
        for (k = 1u; (k <= 2u) && (dark > white); k++)
        {
            uint16_t const span = (uint16_t)(dark - white);
            uint16_t const thr  = (uint16_t)(white + ((k == 1u) ? (span / 2u) : (span / BC_PROF_LOW_DIV)));
            uint16_t const n2   = bc_prof_elements(p_bc->prof, p_bc->prof_n, thr, (uint16_t)(span / 16u),
                                                   w2, (uint16_t)BC_MAX_ELEMENTS);
            char           ch;
            bool           b_rev;

            if (code39_decode_one_repair(w2, n2, BC_RATIO_MIN_X10, &ch, &b_rev) == CODE39_OK)
            {
                p_out->ch         = ch;
                p_out->b_reversed = b_rev;
                p_out->elements   = n2;
                p_bc->pass        = k;
                r = CODE39_OK;
                break;
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
                p_bc->state = (uint8_t)LINE_BC_ARMED;
            }
            break;

        case LINE_BC_ARMED:
            if (p_bc->b_black)
            {
                p_bc->state       = (uint8_t)LINE_BC_CAPTURING;
                p_bc->n           = 0u;
                p_bc->run_um      = step_um;
                p_bc->total_um    = step_um;
                p_bc->max_elem_um = 0u;
                p_bc->peak        = norm;
                p_bc->prof_n      = 1u;
                p_bc->prof[0]     = norm;
                p_bc->bin_um      = step_um;
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

                if (end_um < BC_QUIET_UM)
                {
                    end_um = BC_QUIET_UM;
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
