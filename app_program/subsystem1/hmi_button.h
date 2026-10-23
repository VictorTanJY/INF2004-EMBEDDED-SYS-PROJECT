/*
 *  hmi_button.h
 *  Subsystem 1 -- button debounce and press classification (pure C).
 *  Owner: Buddy 1 (Victor).
 *
 *  hmi_btn_update() is called every HMI_PERIOD_MS with the raw pin level.
 *  A change is accepted only when two consecutive samples agree, so a
 *  bounce shorter than one period (20 ms) is ignored and an accepted
 *  edge is reported 20-40 ms after it happens.
 */

#ifndef HMI_BUTTON_H
#define HMI_BUTTON_H

#include <stdint.h>
#include <stdbool.h>

typedef enum
{
    HMI_BTN_EV_NONE  = 0,
    HMI_BTN_EV_PRESS = 1,       /* debounced press edge                       */
    HMI_BTN_EV_SHORT = 2,       /* released before the long-press time        */
    HMI_BTN_EV_LONG  = 3        /* held for the long-press time (sent once)   */
} hmi_btn_ev_t;

typedef struct
{
    bool     b_last_raw;        /* previous raw sample (true = pressed)       */
    bool     b_stable;          /* debounced level                            */
    bool     b_long_sent;
    uint16_t held_ms;           /* time held since the debounced press        */
} hmi_btn_t;

void         hmi_btn_init(hmi_btn_t * p_btn);
hmi_btn_ev_t hmi_btn_update(hmi_btn_t * p_btn, bool b_pressed_raw,
                            uint16_t dt_ms, uint16_t long_ms);

#endif /* HMI_BUTTON_H */
