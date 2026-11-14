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

static bool s1_bytes_eq(uint8_t const * p_a, uint8_t const * p_b, uint16_t n)
{
    uint16_t i;

    for (i = 0u; i < n; i++)
    {
        if (p_a[i] != p_b[i])
        {
            return false;
        }
    }
    return true;
}

static bool s1_parse(uint8_t const * p_frame, uint16_t n, comms_cmd_t * p_cmd, comms_err_t * p_err)
{
    *p_err = comms_parse_cmd(p_frame, n, p_cmd);
    return (*p_err == COMMS_ERR_NONE);
}

/* Command frames (comms_proto.h): header 0x25 = type 5, version 1. */
static uint8_t const k_start_7[3]  = { 0x25u, 0x07u, 0x01u };              /* START, id 7  */
static uint8_t const k_start_9[3]  = { 0x25u, 0x09u, 0x01u };              /* START, id 9  */
static uint8_t const k_start_11[3] = { 0x25u, 0x0Bu, 0x01u };              /* START, id 11 */
static uint8_t const k_reset_11[3] = { 0x25u, 0x0Bu, 0x04u };              /* RESET, id 11 */
static uint8_t const k_set_99[5]   = { 0x25u, 0x08u, 0x95u, 0x31u, 0x40u };/* SET stop_cm 99, id 8 */
static uint8_t const k_no_id[1]    = { 0x25u };                            /* header only  */
static uint8_t const k_ack_busy[3] = { 0x24u, 0x13u, 0x0Cu };              /* id 9, err busy */

EXPORT void subsystem1_test(void)
{
    static uint8_t buf[COMMS_FRAME_MAX];
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

    /* S1-U02: worst-case telemetry is one 27-byte binary frame. */
    t.seq = 4294967295u;  t.t_ms = 4294967295u;  t.state = 5u;
    t.v_l_mm_s = -32768;  t.v_r_mm_s = -32768;
    t.enc_l = -2147483647 - 1;  t.enc_r = -2147483647 - 1;  t.dist_mm = -2147483647 - 1;
    t.line_err = -1000;  t.b_line_lost = true;  t.barcode = 'Z';
    t.pitch_cdeg = -32768;  t.roll_cdeg = -32768;  t.hump_max_mm = 65535u;
    t.obst_d_mm = 65534u;  t.b_obst_valid = true;
    t.obst_w_mm = 65535u;  t.clr_l_mm = 65535u;  t.clr_r_mm = 65535u;
    len = comms_build_telemetry(&t, buf, (uint16_t)sizeof(buf));
    TEST_CHECK("S1-U02", len == 27u, "worst-case telemetry is a 27-byte binary frame");

    /* S1-U03: valid commands parse; invalid ones are rejected with a reason. */
    TEST_CHECK("S1-U03",
               s1_parse(k_start_7, 3u, &cmd, &err) &&
               (cmd.id == 7u) && (cmd.cmd == (uint8_t)COMMS_CMD_START) &&
               !s1_parse(k_set_99, 5u, &cmd, &err) && (err == COMMS_ERR_RANGE) &&
               !s1_parse(k_no_id, 1u, &cmd, &err) && (err == COMMS_ERR_NO_ID),
               "binary command parser accepts valid and rejects invalid frames");

    /* S1-U04: START is refused outside IDLE (FR14), STOP is not. */
    (void)s1_parse(k_start_9, 3u, &cmd, &err);
    TEST_CHECK("S1-U04",
               (comms_check_cmd(&cmd, false) == COMMS_ERR_BUSY) &&
               (comms_check_cmd(&cmd, true) == COMMS_ERR_NONE),
               "start accepted only in IDLE");

    /* S1-U05: ack frame. */
    len = comms_build_ack(&cmd, COMMS_ERR_BUSY, buf, (uint16_t)sizeof(buf));
    TEST_CHECK("S1-U05", (len == 3u) && s1_bytes_eq(buf, k_ack_busy, 3u),
               "ack frame: 3 bytes, id 9, err busy");

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
        (void)s1_parse(k_start_11, 3u, &cmd, &err);
        comms_dedup_add(&dd, &cmd, 100u);
        (void)s1_parse(k_reset_11, 3u, &again, &err);
        TEST_CHECK("S1-U07",
                   (comms_dedup_check(&dd, &cmd, 5000u) == COMMS_DEDUP_REPEAT) &&
                   (comms_dedup_check(&dd, &again, 5000u) == COMMS_DEDUP_CONFLICT),
                   "duplicate command ids are suppressed within the window");
    }

    TEST_SUMMARY("subsystem1");
}

#endif /* APP_TEST */
