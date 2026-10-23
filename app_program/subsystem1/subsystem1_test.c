/*
 *  subsystem1_test.c
 *  On-target tests for subsystem 1.  Built only with: make TEST=1
 *  Record results in testing/subsystem1/TEST_LOG.md.
 *
 *  The same protocol and debounce logic is also tested on a PC, with
 *  many more cases: testing/subsystem1/host/ (make -C testing/subsystem1/host).
 */

#include "subsystem1.h"
#include "hmi_button.h"
#include "../test_report.h"

#if APP_TEST

static bool s1_str_eq(char const * p_a, char const * p_b)
{
    while ((*p_a != '\0') && (*p_a == *p_b))
    {
        p_a++;
        p_b++;
    }
    return (*p_a == *p_b);
}

static bool s1_parse(char const * p_text, comms_cmd_t * p_cmd, comms_err_t * p_err)
{
    uint16_t n = 0u;

    while (p_text[n] != '\0')
    {
        n++;
    }
    *p_err = comms_parse_cmd((uint8_t const *)p_text, n, p_cmd);
    return (*p_err == COMMS_ERR_NONE);
}

EXPORT void subsystem1_test(void)
{
    static char    buf[320];
    T_RMBF         rmbf;
    comms_cmd_t    cmd;
    comms_err_t    err;
    comms_telem_t  t = {0};
    hmi_btn_t      btn;
    uint16_t       len;
    uint32_t       i;
    bool           b_long = false;
    bool           b_short = false;

    TEST_INIT();

    /* S1-U01: mbf_telem exists and accepts a record without blocking. */
    TEST_CHECK("S1-U01",
               (tk_ref_mbf(g_mbf_telem, &rmbf) == E_OK) &&
               comms_telem_post((uint8_t)COMMS_KIND_LOG, (uint8_t)APP_SRC_COMMS, 1u, 42),
               "mbf_telem accepts a record with TMO_POL");

    /* S1-U02: worst-case telemetry fits the TX buffer (and the 1 KB MQTT ring). */
    t.seq = 4294967295u;  t.t_ms = 4294967295u;  t.state = 5u;
    t.v_l_mm_s = -32768;  t.v_r_mm_s = -32768;
    t.enc_l = -2147483647 - 1;  t.enc_r = -2147483647 - 1;  t.dist_mm = -2147483647 - 1;
    t.line_err = -1000;  t.b_line_lost = true;  t.barcode = 'Z';
    t.pitch_cdeg = -32768;  t.hump_max_mm = 65535u;
    t.obst_d_mm = 65534u;  t.b_obst_valid = true;
    t.obst_w_mm = 65535u;  t.clr_l_mm = 65535u;  t.clr_r_mm = 65535u;
    len = comms_build_telemetry(&t, buf, (uint16_t)sizeof(buf));
    TEST_CHECK("S1-U02", len > 0u, "worst-case telemetry fits the 320 B TX buffer");

    /* S1-U03: valid commands parse; invalid ones are rejected with a reason. */
    TEST_CHECK("S1-U03",
               s1_parse("{\"id\":7,\"cmd\":\"start\"}", &cmd, &err) &&
               (cmd.id == 7u) && (cmd.cmd == (uint8_t)COMMS_CMD_START) &&
               !s1_parse("{\"id\":8,\"cmd\":\"set\",\"param\":\"stop_cm\",\"value\":99}", &cmd, &err) &&
               (err == COMMS_ERR_RANGE) &&
               !s1_parse("{\"cmd\":\"stop\"}", &cmd, &err) && (err == COMMS_ERR_NO_ID),
               "command parser accepts valid and rejects invalid input");

    /* S1-U04: START is refused outside IDLE (FR14), STOP is not. */
    (void)s1_parse("{\"id\":9,\"cmd\":\"start\"}", &cmd, &err);
    TEST_CHECK("S1-U04",
               (comms_check_cmd(&cmd, false) == COMMS_ERR_BUSY) &&
               (comms_check_cmd(&cmd, true) == COMMS_ERR_NONE),
               "start accepted only in IDLE");

    /* S1-U05: ack format. */
    len = comms_build_ack(&cmd, COMMS_ERR_BUSY, buf, (uint16_t)sizeof(buf));
    TEST_CHECK("S1-U05", (len > 0u) && s1_str_eq(buf, "{\"id\":9,\"ok\":false,\"err\":\"busy\"}"),
               "ack JSON");

    /* S1-U06: a 10 ms bounce is ignored; a 2 s hold gives one LONG, no SHORT. */
    hmi_btn_init(&btn);
    (void)hmi_btn_update(&btn, true, 20u, 2000u);
    TEST_CHECK("S1-U06a", hmi_btn_update(&btn, false, 20u, 2000u) == HMI_BTN_EV_NONE,
               "single-sample glitch ignored");
    hmi_btn_init(&btn);
    for (i = 0u; i < 120u; i++)
    {
        hmi_btn_ev_t ev = hmi_btn_update(&btn, true, 20u, 2000u);

        b_long = b_long || (ev == HMI_BTN_EV_LONG);
    }
    (void)hmi_btn_update(&btn, false, 20u, 2000u);
    b_short = (hmi_btn_update(&btn, false, 20u, 2000u) == HMI_BTN_EV_SHORT);
    TEST_CHECK("S1-U06b", b_long && !b_short, "long press reported once, no short");

    /* S1-U07: a delayed QoS 1 redelivery is recognised; a reused id is not. */
    {
        comms_dedup_t dd;
        comms_cmd_t   again;

        comms_dedup_init(&dd);
        (void)s1_parse("{\"id\":11,\"cmd\":\"start\"}", &cmd, &err);
        comms_dedup_add(&dd, &cmd, 100u);
        (void)s1_parse("{\"id\":11,\"cmd\":\"reset\"}", &again, &err);
        TEST_CHECK("S1-U07",
                   (comms_dedup_check(&dd, &cmd, 5000u) == COMMS_DEDUP_REPEAT) &&
                   (comms_dedup_check(&dd, &again, 5000u) == COMMS_DEDUP_CONFLICT),
                   "duplicate command ids are suppressed within the window");
    }

    TEST_SUMMARY("subsystem1");
}

#endif /* APP_TEST */
