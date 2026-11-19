/** @file hmi_button.c
 *
 *  @brief Subsystem 1 -- button debounce (pure C, see hmi_button.h).
 *  Owner: Buddy 1 (Victor).
 */

#include "hmi_button.h"

void hmi_button_init (hmi_button_t * p_btn)
{
    p_btn->b_last_raw  = false;
    p_btn->b_stable    = false;
    p_btn->b_long_sent = false;
    p_btn->held_ms     = 0u;
}

hmi_button_ev_t hmi_button_update (hmi_button_t * p_btn, bool b_pressed_raw,
                                   uint16_t dt_ms, uint16_t long_ms)
{
    hmi_button_ev_t event    = HMI_BTN_EV_NONE;
    bool            b_agrees = (b_pressed_raw == p_btn->b_last_raw);

    p_btn->b_last_raw = b_pressed_raw;

    if (b_agrees && (b_pressed_raw != p_btn->b_stable))
    {
        p_btn->b_stable = b_pressed_raw;
        if (p_btn->b_stable)
        {
            p_btn->held_ms     = 0u;
            p_btn->b_long_sent = false;
            event              = HMI_BTN_EV_PRESS;
        }
        else if (!p_btn->b_long_sent)
        {
            event = HMI_BTN_EV_SHORT;
        }
        else
        {
            /* release after a long press: already reported */
        }
    }
    else if (p_btn->b_stable)
    {
        /* Cast (uint16_t)(0xFFFFu - dt_ms): the value is 0..65535. */
        if (p_btn->held_ms < (uint16_t)(0xFFFFu - dt_ms))
        {
            /* Cast (uint16_t)(p_btn->held_ms + dt_ms): held_ms < 0xFFFF - dt_ms
               (tested above). */
            p_btn->held_ms = (uint16_t)(p_btn->held_ms + dt_ms);
        }
        if (!p_btn->b_long_sent && (p_btn->held_ms >= long_ms))
        {
            p_btn->b_long_sent = true;
            event              = HMI_BTN_EV_LONG;
        }
    }
    else
    {
        /* released and stable: nothing to do */
    }
    return event;
}

/*** end of file ***/
