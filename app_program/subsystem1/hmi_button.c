/*
 *  hmi_button.c
 *  Subsystem 1 -- button debounce (pure C, see hmi_button.h).
 *  Owner: Buddy 1 (Victor).
 */

#include "hmi_button.h"

void hmi_btn_init(hmi_btn_t * p_btn)
{
    p_btn->b_last_raw  = false;
    p_btn->b_stable    = false;
    p_btn->b_long_sent = false;
    p_btn->held_ms     = 0u;
}

hmi_btn_ev_t hmi_btn_update(hmi_btn_t * p_btn, bool b_pressed_raw,
                            uint16_t dt_ms, uint16_t long_ms)
{
    hmi_btn_ev_t ev       = HMI_BTN_EV_NONE;
    bool         b_agrees = (b_pressed_raw == p_btn->b_last_raw);

    p_btn->b_last_raw = b_pressed_raw;

    if (b_agrees && (b_pressed_raw != p_btn->b_stable))
    {
        p_btn->b_stable = b_pressed_raw;
        if (p_btn->b_stable)
        {
            p_btn->held_ms     = 0u;
            p_btn->b_long_sent = false;
            ev = HMI_BTN_EV_PRESS;
        }
        else if (!p_btn->b_long_sent)
        {
            ev = HMI_BTN_EV_SHORT;
        }
        else
        {
            /* release after a long press: already reported */
        }
    }
    else if (p_btn->b_stable)
    {
        if (p_btn->held_ms < (uint16_t)(0xFFFFu - dt_ms))
        {
            p_btn->held_ms = (uint16_t)(p_btn->held_ms + dt_ms);
        }
        if (!p_btn->b_long_sent && (p_btn->held_ms >= long_ms))
        {
            p_btn->b_long_sent = true;
            ev = HMI_BTN_EV_LONG;
        }
    }
    else
    {
        /* released and stable: nothing to do */
    }
    return ev;
}
