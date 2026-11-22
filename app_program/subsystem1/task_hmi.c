/** @file task_hmi.c
 *
 *  @brief Subsystem 1 -- HmiTask: the two Robo Pico buttons (FR16).
 *  Owner: Buddy 1 (Victor).
 *
 *  Priority 11, core 0, periodic 20 ms (tk_dly_tsk): a human timescale,
 *  so period drift does not matter.  HmiTask only reports button events;
 *  MissionTask (subsystem 5) owns the state machine and decides what each
 *  event means in the current state.
 *
 *  GP20 BTN_START  press               -> MISSION_EV_BTN_START_STOP
 *  GP21 BTN_MODE   short press         -> MISSION_EV_BTN_CALIBRATE
 *  held >= 2 s         -> MISSION_EV_BTN_RESET (fault reset)
 *
 *  Every event is also posted to mbf_telem so it appears on MQTT.
 */

#include "task_hmi.h"
#include "subsystem1.h"
#include "hmi_button.h"
#include "../subsystem5/subsystem5.h"

static void hmi_pin_init(uint32_t pin);
static bool hmi_pressed(uint32_t pin);
static void hmi_report(mission_types_ev_t event);

EXPORT void task_hmi_thread (INT stacd, void * p_exinf)
{
    hmi_button_t btn_start;
    hmi_button_t btn_mode;

    (void)stacd;
    (void)p_exinf;

    hmi_pin_init(S1_PIN_BTN_START);
    hmi_pin_init(S1_PIN_BTN_MODE);
    hmi_button_init(&btn_start);
    hmi_button_init(&btn_mode);

    for (;;)
    {
        hmi_button_ev_t event;

        (void)tk_dly_tsk(HMI_PERIOD_MS);

        event = hmi_button_update(&btn_start, hmi_pressed(S1_PIN_BTN_START),
                                  HMI_PERIOD_MS, HMI_LONG_PRESS_MS);

        if (HMI_BTN_EV_PRESS == event)
        {
            hmi_report(MISSION_EV_BTN_START_STOP);
        }

        event = hmi_button_update(&btn_mode, hmi_pressed(S1_PIN_BTN_MODE),
                                  HMI_PERIOD_MS, HMI_LONG_PRESS_MS);

        if (HMI_BTN_EV_SHORT == event)
        {
            hmi_report(MISSION_EV_BTN_CALIBRATE);
        }
        else if (HMI_BTN_EV_LONG == event)
        {
            hmi_report(MISSION_EV_BTN_RESET);
        }
        else
        {
            /* PRESS on BTN_MODE: wait for release or long press */
        }
    }
}

/** Configure a button pin as an input with the internal pull-up. */
static void hmi_pin_init (uint32_t pin)
{
    (void)gpio_set_pin(pin, GPIO_MODE_IN);
    clr_w(app_regs_pad(pin), APP_REGS_PAD_PDE);
    set_w(app_regs_pad(pin), APP_REGS_PAD_PUE);
}

/** True if the button on pin is pressed now (raw level, not debounced). */
static bool hmi_pressed (uint32_t pin)
{
    return (S1_BTN_PRESSED_LEVEL == gpio_get_val(pin));
}

/** Post a button event to MissionTask; a full mbf_mission is logged, not
    waited on. */
static void hmi_report (mission_types_ev_t event)
{
    /* Both sends use TMO_POL; a full mbf_mission is reported, not waited on. */
    if (subsystem5_mission_post(event, APP_SRC_COMMS, 0u, 0) != E_OK)
    {
        (void)subsystem1_telem_post(COMMS_KIND_LOG, APP_SRC_COMMS, event, -1);
    }

    (void)subsystem1_telem_post(COMMS_KIND_EVENT, APP_SRC_COMMS, event, 0);
}

/*** end of file ***/
