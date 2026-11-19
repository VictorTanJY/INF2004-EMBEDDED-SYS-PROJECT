/*
 *  subsystem1_test_run.c
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

static bool s1_bytes_eq (uint8_t const * p_a, uint8_t const * p_b,
                         uint16_t n_bytes)
{
    uint16_t idx;

    for (idx = 0u; idx < n_bytes; idx++)
    {
        if (p_a[idx] != p_b[idx])
        {
            return false;
        }
    }
    return true;
}

static bool s1_parse (uint8_t const * p_frame, uint16_t n_bytes,
                      comms_proto_cmd_t * p_cmd, comms_proto_err_t * p_err)
{
    *p_err = comms_proto_parse_cmd(p_frame, n_bytes, p_cmd);
    return (COMMS_ERR_NONE == *p_err);
}

/* Command frames (comms_proto.h): header 0x25 = type 5, version 1. */
static uint8_t const g_start_7[3]  = {0x25u, 0x07u, 0x01u}; /* START, id 7  */
static uint8_t const g_start_9[3]  = {0x25u, 0x09u, 0x01u}; /* START, id 9  */
static uint8_t const g_start_11[3] = {0x25u, 0x0Bu, 0x01u}; /* START, id 11 */
static uint8_t const g_reset_11[3] = {0x25u, 0x0Bu, 0x04u}; /* RESET, id 11 */
static uint8_t const g_set_99[5]   = {0x25u, 0x08u, 0x95u, 0x31u,
                                      0x40u}; /* SET stop_cm 99, id 8 */
static uint8_t const g_no_id[1]    = {0x25u}; /* header only  */
static uint8_t const g_ack_busy[3] = {0x24u, 0x13u, 0x0Cu}; /* id 9, err busy */

EXPORT void subsystem1_test_run (void)
{
    static uint8_t      buf[COMMS_FRAME_MAX];
    T_RMBF              rmbf;
    comms_proto_cmd_t   cmd;
    comms_proto_err_t   err;
    comms_proto_telem_t telem = {0};
    hmi_button_t        btn;
    uint16_t            len;
    uint32_t            idx;
    bool                b_long  = false;
    bool                b_short = false;

    TEST_INIT();

    /* S1-U01: mbf_telem exists and accepts a record without blocking. */
    TEST_CHECK("S1-U01",
               (E_OK == tk_ref_mbf(g_mbf_telem, &rmbf))
                   && subsystem1_telem_post(COMMS_KIND_LOG,
                                            (uint8_t)APP_SRC_COMMS, 1u, 42),
               "mbf_telem accepts a record with TMO_POL");

    /* S1-U02: worst-case telemetry is one 27-byte binary frame. */
    telem.seq          = 4294967295u;
    telem.t_ms         = 4294967295u;
    telem.state        = 5u;
    telem.v_l_mm_s     = -32768;
    telem.v_r_mm_s     = -32768;
    telem.enc_l        = -2147483647 - 1;
    telem.enc_r        = -2147483647 - 1;
    telem.dist_mm      = -2147483647 - 1;
    telem.line_err     = -1000;
    telem.b_line_lost  = true;
    telem.barcode      = 'Z';
    telem.pitch_cdeg   = -32768;
    telem.roll_cdeg    = -32768;
    telem.hump_max_mm  = 65535u;
    telem.obst_d_mm    = 65534u;
    telem.b_obst_valid = true;
    telem.obst_w_mm    = 65535u;
    telem.clr_l_mm     = 65535u;
    telem.clr_r_mm     = 65535u;
    len = comms_proto_build_telemetry(&telem, buf, (uint16_t)sizeof(buf));
    TEST_CHECK("S1-U02", 27u == len,
               "worst-case telemetry is a 27-byte binary frame");

    /* S1-U03: valid commands parse; invalid ones are rejected with a reason. */
    TEST_CHECK(
        "S1-U03",
        s1_parse(g_start_7, 3u, &cmd, &err) && (7u == cmd.cmd_id)
            && (COMMS_CMD_START == cmd.cmd)
            && !s1_parse(g_set_99, 5u, &cmd, &err) && (COMMS_ERR_RANGE == err)
            && !s1_parse(g_no_id, 1u, &cmd, &err) && (COMMS_ERR_NO_ID == err),
        "binary command parser accepts valid and rejects invalid frames");

    /* S1-U04: START is refused outside IDLE (FR14), STOP is not. */
    (void)s1_parse(g_start_9, 3u, &cmd, &err);
    TEST_CHECK("S1-U04",
               (COMMS_ERR_BUSY == comms_proto_check_cmd(&cmd, false))
                   && (COMMS_ERR_NONE == comms_proto_check_cmd(&cmd, true)),
               "start accepted only in IDLE");

    /* S1-U05: ack frame. */
    len =
        comms_proto_build_ack(&cmd, COMMS_ERR_BUSY, buf, (uint16_t)sizeof(buf));
    TEST_CHECK("S1-U05", (3u == len) && s1_bytes_eq(buf, g_ack_busy, 3u),
               "ack frame: 3 bytes, id 9, err busy");

    /* S1-U06: a 10 ms bounce is ignored; a 2 s hold gives one LONG, no SHORT.
     */
    hmi_button_init(&btn);
    (void)hmi_button_update(&btn, true, 20u, 2000u);
    TEST_CHECK("S1-U06a",
               HMI_BTN_EV_NONE == hmi_button_update(&btn, false, 20u, 2000u),
               "single-sample glitch ignored");
    hmi_button_init(&btn);
    for (idx = 0u; idx < 120u; idx++)
    {
        hmi_button_ev_t event = hmi_button_update(&btn, true, 20u, 2000u);

        b_long = b_long || (HMI_BTN_EV_LONG == event);
    }
    (void)hmi_button_update(&btn, false, 20u, 2000u);
    b_short = (HMI_BTN_EV_SHORT == hmi_button_update(&btn, false, 20u, 2000u));
    TEST_CHECK("S1-U06b", b_long && !b_short,
               "long press reported once, no short");

    /* S1-U07: a delayed QoS 1 redelivery is recognised; a reused id is not. */
    {
        comms_proto_dedup_t dedup;
        comms_proto_cmd_t   again;

        comms_proto_dedup_init(&dedup);
        (void)s1_parse(g_start_11, 3u, &cmd, &err);
        comms_proto_dedup_add(&dedup, &cmd, 100u);
        (void)s1_parse(g_reset_11, 3u, &again, &err);
        TEST_CHECK(
            "S1-U07",
            (COMMS_DEDUP_REPEAT == comms_proto_dedup_check(&dedup, &cmd, 5000u))
                && (COMMS_DEDUP_CONFLICT
                    == comms_proto_dedup_check(&dedup, &again, 5000u)),
            "duplicate command ids are suppressed within the window");
    }

    TEST_SUMMARY("subsystem1");
}

#endif /* APP_TEST */

/*** end of file ***/
