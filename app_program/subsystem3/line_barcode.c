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
}

static void start_cooldown(bc_capture_t * p_bc)
{
    p_bc->state       = (uint8_t)LINE_BC_COOLDOWN;
    p_bc->cooldown_um = 0u;
}

/* The trailing white run ended the code: decode what was captured. */
static bc_result_t finish(bc_capture_t * p_bc, bc_out_t * p_out)
{
    code39_result_t r;

    p_out->elements  = p_bc->n;
    p_out->length_um = p_bc->total_um - p_bc->run_um;   /* without the trailing white */
    p_out->peak_norm = p_bc->peak;
    r = code39_decode_one(p_bc->widths, p_bc->n, BC_RATIO_MIN_X10, &p_out->ch, &p_out->b_reversed);
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

    /* Hysteresis: a level change needs a clear crossing. */
    if (p_bc->b_black && (norm < BC_BLACK_OFF))
    {
        p_bc->b_black = false;
    }
    else if (!p_bc->b_black && (norm > BC_BLACK_ON))
    {
        p_bc->b_black = true;
    }
    else
    {
        /* level unchanged */
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
            }
            break;

        case LINE_BC_CAPTURING:
            p_bc->total_um += step_um;
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
