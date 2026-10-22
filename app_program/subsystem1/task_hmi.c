/*
 *  task_hmi.c
 *  Subsystem 1 -- HmiTask: the two Robo Pico buttons (FR16).
 *  Owner: Buddy 1 (Victor).
 *
 *  Priority 11, core 0, periodic 20 ms (tk_dly_tsk): a human timescale,
 *  so period drift does not matter.  HmiTask only reports button events;
 *  MissionTask (subsystem 5) owns the state machine and decides what each
 *  event means in the current state.
 *
 *    GP20 BTN_START  press               -> MISSION_EV_BTN_START_STOP
 *    GP21 BTN_MODE   short press         -> MISSION_EV_BTN_CALIBRATE
 *                    held >= 2 s         -> MISSION_EV_BTN_RESET (fault reset)
 *
 *  Every event is also posted to mbf_telem so it appears on MQTT.
 */

#include "subsystem1.h"
#include "hmi_button.h"
#include "../subsystem5/subsystem5.h"

static void hmi_pin_init(UW pin)
{
    (void)gpio_set_pin(pin, GPIO_MODE_IN);
    clr_w(GPIO(pin), GPIO_PDE);
    set_w(GPIO(pin), GPIO_PUE);
}

static bool hmi_pressed(UW pin)
{
    return (gpio_get_val(pin) == S1_BTN_PRESSED_LEVEL);
}

static void hmi_report(mission_ev_t ev)
{
    /* Both sends use TMO_POL; a full mbf_mission is reported, not waited on. */
    if (mission_post((uint8_t)ev, (uint8_t)APP_SRC_COMMS, 0u, 0) != E_OK)
    {
        (void)comms_telem_post((uint8_t)COMMS_KIND_LOG, (uint8_t)APP_SRC_COMMS,
                               (uint16_t)ev, -1);
    }
    (void)comms_telem_post((uint8_t)COMMS_KIND_EVENT, (uint8_t)APP_SRC_COMMS,
                           (uint16_t)ev, 0);
}

EXPORT void hmi_task(INT stacd, void * p_exinf)
{
    hmi_btn_t btn_start;
    hmi_btn_t btn_mode;

    (void)stacd;
    (void)p_exinf;

    hmi_pin_init(S1_PIN_BTN_START);
    hmi_pin_init(S1_PIN_BTN_MODE);
    hmi_btn_init(&btn_start);
    hmi_btn_init(&btn_mode);

    for (;;)
    {
        hmi_btn_ev_t ev;

        (void)tk_dly_tsk(HMI_PERIOD_MS);

        ev = hmi_btn_update(&btn_start, hmi_pressed(S1_PIN_BTN_START),
                            (uint16_t)HMI_PERIOD_MS, (uint16_t)HMI_LONG_PRESS_MS);
        if (ev == HMI_BTN_EV_PRESS)
        {
            hmi_report(MISSION_EV_BTN_START_STOP);
        }

        ev = hmi_btn_update(&btn_mode, hmi_pressed(S1_PIN_BTN_MODE),
                            (uint16_t)HMI_PERIOD_MS, (uint16_t)HMI_LONG_PRESS_MS);
        if (ev == HMI_BTN_EV_SHORT)
        {
            hmi_report(MISSION_EV_BTN_CALIBRATE);
        }
        else if (ev == HMI_BTN_EV_LONG)
        {
            hmi_report(MISSION_EV_BTN_RESET);
        }
        else
        {
            /* PRESS on BTN_MODE: wait for release or long press */
        }
    }
}
