/*
 *  test_s1.c
 *  Host unit tests for subsystem 1's pure modules: comms_proto.c (JSON
 *  writer, builders, command parser, offline ring) and hmi_button.c.
 *  Build and run:  make -C testing/subsystem1/host
 *  Test IDs S1-H.. are listed in testing/subsystem1/TEST_LOG.md.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "comms_proto.h"
#include "hmi_button.h"

static int g_pass;
static int g_fail;

#define CHECK(id, cond) \
    do { if (cond) { g_pass++; } else { g_fail++; \
        printf("FAIL %s  (%s:%d)  %s\n", id, __FILE__, __LINE__, #cond); } } while (0)

static comms_err_t parse(char const * p_text, comms_cmd_t * p_cmd)
{
    return comms_parse_cmd((uint8_t const *)p_text, (uint16_t)strlen(p_text), p_cmd);
}

/* ------------------------------------------------------------------ */
static void test_writer(void)
{
    char buf[64];
    jw_t jw;

    jw_init(&jw, buf, sizeof(buf));
    jw_int(&jw, 0);  jw_raw(&jw, ",");
    jw_int(&jw, -2147483647 - 1);  jw_raw(&jw, ",");
    jw_uint(&jw, 4294967295u);  jw_raw(&jw, ",");
    jw_fixed1(&jw, -5);  jw_raw(&jw, ",");
    jw_fixed1(&jw, 123);  jw_raw(&jw, ",");
    jw_str(&jw, "a\"b\\c\n");
    CHECK("S1-H01", jw_finish(&jw) > 0u);
    CHECK("S1-H01", strcmp(buf, "0,-2147483648,4294967295,-0.5,12.3,\"a\\\"b\\\\c\\u000a\"") == 0);

    /* Never writes past cap, for every cap; overflow returns 0. */
    {
        comms_telem_t t;
        uint16_t      full;
        uint16_t      cap;
        char          big[400];

        memset(&t, 0, sizeof(t));
        t.obst_d_mm = COMMS_D_NONE;
        full = comms_build_telemetry(&t, big, sizeof(big));
        for (cap = 0u; cap <= (uint16_t)(full + 2u); cap++)
        {
            char     area[420];
            uint16_t len;
            uint16_t i;
            int      ok = 1;

            memset(area, 0x5A, sizeof(area));
            len = comms_build_telemetry(&t, area, cap);
            for (i = cap; i < sizeof(area); i++)
            {
                if ((unsigned char)area[i] != 0x5Au) { ok = 0; }
            }
            CHECK("S1-H02", ok);
            CHECK("S1-H02", (cap > full) ? (len == full) : (len == 0u));
        }
    }
}

static void test_telemetry(void)
{
    char          buf[384];
    comms_telem_t t;
    uint16_t      len;

    memset(&t, 0, sizeof(t));
    t.seq = 12u;  t.t_ms = 3400u;  t.state = 3u;
    t.v_l_mm_s = 250;  t.v_r_mm_s = -10;
    t.enc_l = 1000;  t.enc_r = 990;  t.dist_mm = 555;
    t.line_err = -120;  t.b_line_lost = false;  t.barcode = 'A';
    t.pitch_cdeg = -1234;  t.hump_max_mm = 25u;
    t.obst_d_mm = 384u;  t.b_obst_valid = false;
    len = comms_build_telemetry(&t, buf, sizeof(buf));
    CHECK("S1-H03", len == strlen(buf));
    CHECK("S1-H03", strcmp(buf,
        "{\"seq\":12,\"t_ms\":3400,\"state\":\"LINE_FOLLOW\",\"v_mm_s\":[250,-10],"
        "\"enc\":[1000,990],\"dist_mm\":555,\"line\":{\"err\":-120,\"lost\":false},"
        "\"barcode\":\"A\",\"pitch_deg\":-12.3,\"hump_max_mm\":25,"
        "\"obstacle\":{\"d_cm\":38,\"w_cm\":null,\"w_mm\":null,\"clr_l_cm\":null,\"clr_r_cm\":null},"
        "\"nav\":null}") == 0);

    /* No obstacle, no barcode. */
    t.obst_d_mm = COMMS_D_NONE;  t.barcode = '\0';  t.pitch_cdeg = 5;
    (void)comms_build_telemetry(&t, buf, sizeof(buf));
    CHECK("S1-H04", strstr(buf, "\"barcode\":null") != NULL);
    CHECK("S1-H04", strstr(buf, "\"d_cm\":null") != NULL);
    CHECK("S1-H04", strstr(buf, "\"pitch_deg\":0.1") != NULL);
    t.b_obst_valid = true;  t.obst_d_mm = 190u;  t.obst_w_mm = 147u;  t.nav = 'F';
    (void)comms_build_telemetry(&t, buf, sizeof(buf));
    CHECK("S1-H04", strstr(buf, "\"w_cm\":15,\"w_mm\":147") != NULL);
    CHECK("S1-H04", strstr(buf, "\"nav\":\"F\"") != NULL);
    t.b_obst_valid = false;  t.nav = '\0';

    /* Worst case stays below the 384 B buffer and the 1 KB MQTT ring. */
    t.seq = 4294967295u;  t.t_ms = 4294967295u;  t.state = 5u;
    t.v_l_mm_s = -32768;  t.v_r_mm_s = -32768;
    t.enc_l = -2147483647 - 1;  t.enc_r = -2147483647 - 1;  t.dist_mm = -2147483647 - 1;
    t.line_err = -32768;  t.b_line_lost = true;  t.barcode = 'Z';
    t.pitch_cdeg = -32768;  t.hump_max_mm = 65535u;  t.obst_d_mm = 65534u;
    t.b_obst_valid = true;  t.obst_w_mm = 65535u;  t.clr_l_mm = 65535u;  t.clr_r_mm = 65535u;
    t.nav = '%';
    len = comms_build_telemetry(&t, buf, sizeof(buf));
    printf("info: worst-case telemetry payload = %u B (+%u B topic)\n",
           (unsigned)len, (unsigned)strlen("picocar/is23/telemetry"));
    CHECK("S1-H05", (len > 0u) && (len < 384u));   /* COMMS_TX_BUF_SIZE */
}

static void test_event_status_ack(void)
{
    char        buf[200];
    comms_rec_t rec;
    comms_cmd_t cmd;

    rec.kind = COMMS_KIND_EVENT;  rec.src = 3u;  rec.code = 8u;  rec.data = 'B';  rec.t_ms = 77u;
    (void)comms_build_event(&rec, 5u, buf, sizeof(buf));
    CHECK("S1-H06", strcmp(buf, "{\"seq\":5,\"t_ms\":77,\"type\":\"BARCODE\",\"data\":\"B\",\"src\":3}") == 0);

    rec.kind = COMMS_KIND_STATE;  rec.code = 10u;  rec.data = 0;
    (void)comms_build_event(&rec, 6u, buf, sizeof(buf));
    CHECK("S1-H06", strstr(buf, "\"type\":\"STATE\",\"data\":\"FAULT\"") != NULL);

    rec.kind = COMMS_KIND_FAULT;  rec.code = 0u;  rec.data = 0x14;
    (void)comms_build_event(&rec, 7u, buf, sizeof(buf));
    CHECK("S1-H06", strstr(buf, "\"type\":\"FAULT\",\"data\":20") != NULL);

    rec.kind = COMMS_KIND_EVENT;  rec.code = 999u;
    (void)comms_build_event(&rec, 8u, buf, sizeof(buf));
    CHECK("S1-H06", strstr(buf, "\"UNKNOWN\"") != NULL);

    (void)comms_build_status(true, 61u, -58, 3u, NULL, buf, sizeof(buf));
    CHECK("S1-H07", strcmp(buf, "{\"online\":true,\"uptime_s\":61,\"rssi\":-58,\"drops\":3,\"map\":null}") == 0);
    {
        char const map[4] = { 'Q', 'F', 'Z', 'W' };

        (void)comms_build_status(true, 1u, -60, 0u, map, buf, sizeof(buf));
        CHECK("S1-H07", strstr(buf, "\"map\":{\"left\":\"Q\",\"right\":\"F\",\"straight\":\"Z\",\"uturn\":\"W\"}}") != NULL);
    }

    (void)parse("{\"id\":42,\"cmd\":\"stop\"}", &cmd);
    (void)comms_build_ack(&cmd, COMMS_ERR_NONE, buf, sizeof(buf));
    CHECK("S1-H08", strcmp(buf, "{\"id\":42,\"ok\":true}") == 0);
    (void)parse("{\"cmd\":\"stop\"}", &cmd);
    (void)comms_build_ack(&cmd, COMMS_ERR_NO_ID, buf, sizeof(buf));
    CHECK("S1-H08", strcmp(buf, "{\"id\":null,\"ok\":false,\"err\":\"no_id\"}") == 0);
}

static void test_parser(void)
{
    comms_cmd_t cmd;

    CHECK("S1-H09", parse("{\"id\":1,\"cmd\":\"start\"}", &cmd) == COMMS_ERR_NONE && cmd.cmd == COMMS_CMD_START);
    CHECK("S1-H09", parse(" { \"cmd\" : \"stop\" , \"id\" : 2 } ", &cmd) == COMMS_ERR_NONE && cmd.cmd == COMMS_CMD_STOP && cmd.id == 2u);
    CHECK("S1-H09", parse("{\"id\":3,\"cmd\":\"estop\"}", &cmd) == COMMS_ERR_NONE && cmd.cmd == COMMS_CMD_ESTOP);
    CHECK("S1-H09", parse("{\"id\":4,\"cmd\":\"reset\",\"x\":[1]}", &cmd) == COMMS_ERR_BAD_JSON);
    CHECK("S1-H09", parse("{\"id\":4,\"cmd\":\"reset\",\"note\":\"hi \\\"there\\\"\"}", &cmd) == COMMS_ERR_NONE && cmd.cmd == COMMS_CMD_RESET);
    CHECK("S1-H09", parse("{\"id\":5,\"cmd\":\"set\",\"param\":\"stop_cm\",\"value\":25}", &cmd) == COMMS_ERR_NONE
                    && cmd.cmd == COMMS_CMD_SET && cmd.param_id == 3u && cmd.value == 25);

    /* Boundaries of every whitelisted parameter. */
    {
        static char const * const names[] = { "speed_mm_s", "slow_cm", "stop_cm", "lost_mm", "junction_mm" };
        unsigned i;

        for (i = 0u; i < 5u; i++)
        {
            comms_param_t const * p = comms_param_by_name(names[i], (uint16_t)strlen(names[i]));
            char text[120];

            CHECK("S1-H10", p != NULL && comms_param_by_id(p->id) == p);
            snprintf(text, sizeof(text), "{\"id\":9,\"cmd\":\"set\",\"param\":\"%s\",\"value\":%ld}", names[i], (long)p->min);
            CHECK("S1-H10", parse(text, &cmd) == COMMS_ERR_NONE);
            snprintf(text, sizeof(text), "{\"id\":9,\"cmd\":\"set\",\"param\":\"%s\",\"value\":%ld}", names[i], (long)p->max);
            CHECK("S1-H10", parse(text, &cmd) == COMMS_ERR_NONE);
            snprintf(text, sizeof(text), "{\"id\":9,\"cmd\":\"set\",\"param\":\"%s\",\"value\":%ld}", names[i], (long)p->min - 1);
            CHECK("S1-H10", parse(text, &cmd) == COMMS_ERR_RANGE);
            snprintf(text, sizeof(text), "{\"id\":9,\"cmd\":\"set\",\"param\":\"%s\",\"value\":%ld}", names[i], (long)p->max + 1);
            CHECK("S1-H10", parse(text, &cmd) == COMMS_ERR_RANGE);
        }
    }

    /* Rejections, each with its reason. */
    CHECK("S1-H11", parse("", &cmd) == COMMS_ERR_BAD_JSON);
    CHECK("S1-H11", parse("start", &cmd) == COMMS_ERR_BAD_JSON);
    CHECK("S1-H11", parse("{\"id\":1,\"cmd\":\"start\"", &cmd) == COMMS_ERR_BAD_JSON);
    CHECK("S1-H11", parse("{\"id\":1 \"cmd\":\"start\"}", &cmd) == COMMS_ERR_BAD_JSON);
    CHECK("S1-H11", parse("{\"id\":1,\"cmd\":\"start\"} x", &cmd) == COMMS_ERR_BAD_JSON);
    CHECK("S1-H11", parse("{\"id\":{},\"cmd\":\"start\"}", &cmd) == COMMS_ERR_BAD_JSON);
    CHECK("S1-H11", parse("{\"cmd\":\"start\"}", &cmd) == COMMS_ERR_NO_ID);
    CHECK("S1-H11", parse("{\"id\":-1,\"cmd\":\"start\"}", &cmd) == COMMS_ERR_NO_ID);
    CHECK("S1-H11", parse("{\"id\":\"1\",\"cmd\":\"start\"}", &cmd) == COMMS_ERR_NO_ID);
    CHECK("S1-H11", parse("{\"id\":99999999999,\"cmd\":\"start\"}", &cmd) == COMMS_ERR_NO_ID);
    CHECK("S1-H11", parse("{\"id\":1.5,\"cmd\":\"start\"}", &cmd) == COMMS_ERR_NO_ID);
    CHECK("S1-H11", parse("{\"id\":1}", &cmd) == COMMS_ERR_BAD_CMD);
    CHECK("S1-H11", parse("{\"id\":1,\"cmd\":\"fly\"}", &cmd) == COMMS_ERR_BAD_CMD);
    CHECK("S1-H11", parse("{\"id\":1,\"cmd\":\"starts\"}", &cmd) == COMMS_ERR_BAD_CMD);
    CHECK("S1-H11", parse("{\"id\":1,\"cmd\":\"set\",\"value\":20}", &cmd) == COMMS_ERR_BAD_PARAM);
    CHECK("S1-H11", parse("{\"id\":1,\"cmd\":\"set\",\"param\":\"kp\",\"value\":1}", &cmd) == COMMS_ERR_BAD_PARAM);
    CHECK("S1-H11", parse("{\"id\":1,\"cmd\":\"set\",\"param\":\"stop_cm\"}", &cmd) == COMMS_ERR_RANGE);
    CHECK("S1-H11", parse("{\"id\":1,\"cmd\":\"set\",\"param\":\"stop_cm\",\"value\":20.5}", &cmd) == COMMS_ERR_RANGE);
    CHECK("S1-H11", parse("{\"id\":1,\"cmd\":\"set\",\"param\":\"stop_cm\",\"value\":\"20\"}", &cmd) == COMMS_ERR_RANGE);

    /* A trailing NUL from a C-string publisher is accepted. */
    {
        static char const with_nul[] = "{\"id\":1,\"cmd\":\"stop\"}";

        CHECK("S1-H12", comms_parse_cmd((uint8_t const *)with_nul, (uint16_t)sizeof(with_nul), &cmd) == COMMS_ERR_NONE);
    }

    /* State rule (FR14). */
    (void)parse("{\"id\":1,\"cmd\":\"set\",\"param\":\"stop_cm\",\"value\":20}", &cmd);
    CHECK("S1-H13", comms_check_cmd(&cmd, false) == COMMS_ERR_BUSY);
    CHECK("S1-H13", comms_check_cmd(&cmd, true) == COMMS_ERR_NONE);
    (void)parse("{\"id\":1,\"cmd\":\"estop\"}", &cmd);
    CHECK("S1-H13", comms_check_cmd(&cmd, false) == COMMS_ERR_NONE);

    /* Robustness: random bytes and every truncation never crash (ASan/UBSan). */
    {
        char const * good = "{\"id\":5,\"cmd\":\"set\",\"param\":\"stop_cm\",\"value\":25}";
        uint8_t      junk[128];
        unsigned     n;
        unsigned     k;

        for (n = 0u; n <= strlen(good); n++)
        {
            (void)comms_parse_cmd((uint8_t const *)good, (uint16_t)n, &cmd);
        }
        srand(1234);
        for (k = 0u; k < 20000u; k++)
        {
            unsigned len = (unsigned)rand() % sizeof(junk);

            for (n = 0u; n < len; n++)
            {
                static char const alphabet[] = "{}[]\":,0123456789-.e \\abtnrsu";

                junk[n] = (k & 1u) ? (uint8_t)rand()
                                   : (uint8_t)alphabet[(unsigned)rand() % (sizeof(alphabet) - 1u)];
            }
            (void)comms_parse_cmd(junk, (uint16_t)len, &cmd);
        }
        CHECK("S1-H14", 1);
    }
}

static void test_ring(void)
{
    comms_ring_t ring;
    comms_rec_t  rec;
    uint32_t     seq;
    uint32_t     i;
    int          overwrote = 0;

    comms_ring_init(&ring);
    CHECK("S1-H15", !comms_ring_peek(&ring, &rec, &seq));
    memset(&rec, 0, sizeof(rec));
    for (i = 1u; i <= 20u; i++)
    {
        rec.data = (int32_t)i;
        overwrote += comms_ring_push(&ring, &rec, i) ? 1 : 0;
    }
    CHECK("S1-H15", overwrote == 4 && ring.count == COMMS_RING_LEN);
    for (i = 5u; i <= 20u; i++)
    {
        CHECK("S1-H15", comms_ring_peek(&ring, &rec, &seq) && seq == i && rec.data == (int32_t)i);
        comms_ring_pop(&ring);
    }
    CHECK("S1-H15", ring.count == 0u && !comms_ring_peek(&ring, &rec, &seq));
    comms_ring_pop(&ring);  /* pop on empty is harmless */
    CHECK("S1-H15", ring.count == 0u);
}

static int run_button(char const * p_pattern, uint16_t long_ms, char * p_events)
{
    /* One char per 20 ms sample: '1' pressed, '0' released.
       Output: P press, S short, L long. */
    hmi_btn_t btn;
    int       n = 0;

    hmi_btn_init(&btn);
    for (; *p_pattern != '\0'; p_pattern++)
    {
        hmi_btn_ev_t ev = hmi_btn_update(&btn, *p_pattern == '1', 20u, long_ms);

        if (ev == HMI_BTN_EV_PRESS) { p_events[n++] = 'P'; }
        if (ev == HMI_BTN_EV_SHORT) { p_events[n++] = 'S'; }
        if (ev == HMI_BTN_EV_LONG)  { p_events[n++] = 'L'; }
    }
    p_events[n] = '\0';
    return n;
}

static void test_button(void)
{
    char ev[32];
    char pattern[200];

    (void)run_button("0101010000", 2000u, ev);           /* bounce only          */
    CHECK("S1-H16", strcmp(ev, "") == 0);
    (void)run_button("0110000", 2000u, ev);              /* 40 ms press          */
    CHECK("S1-H16", strcmp(ev, "PS") == 0);
    (void)run_button("01111101111100", 2000u, ev);       /* 1-sample drop inside */
    CHECK("S1-H16", strcmp(ev, "PS") == 0);

    memset(pattern, '1', 150);                           /* 3 s hold             */
    pattern[0] = '0';
    strcpy(&pattern[150], "000");
    (void)run_button(pattern, 2000u, ev);
    CHECK("S1-H17", strcmp(ev, "PL") == 0);              /* one LONG, no SHORT   */

    memset(pattern, '1', 99);                            /* release at 1.96 s    */
    pattern[0] = '0';
    strcpy(&pattern[99], "00");
    (void)run_button(pattern, 2000u, ev);
    CHECK("S1-H17", strcmp(ev, "PS") == 0);
}

static void test_dedup(void)
{
    comms_dedup_t dd;
    comms_cmd_t   a, b, stop;
    unsigned      i;

    comms_dedup_init(&dd);
    (void)parse("{\"id\":7,\"cmd\":\"start\"}", &a);
    (void)parse("{\"id\":7,\"cmd\":\"reset\"}", &b);
    (void)parse("{\"id\":9,\"cmd\":\"stop\"}", &stop);

    CHECK("S1-H18", comms_dedup_check(&dd, &a, 1000u) == COMMS_DEDUP_NEW);
    comms_dedup_add(&dd, &a, 1000u);
    /* delayed redelivery after other commands, inside the window */
    for (i = 0u; i < 5u; i++)
    {
        comms_cmd_t other;
        char text[64];

        snprintf(text, sizeof(text), "{\"id\":%u,\"cmd\":\"start\"}", 100u + i);
        (void)parse(text, &other);
        comms_dedup_add(&dd, &other, 2000u + i);
    }
    CHECK("S1-H18", comms_dedup_check(&dd, &a, 30000u) == COMMS_DEDUP_REPEAT);
    CHECK("S1-H18", comms_dedup_check(&dd, &b, 30000u) == COMMS_DEDUP_CONFLICT);
    CHECK("S1-H18", comms_err_name(COMMS_ERR_REUSED)[0] == 'i');
    /* outside the window the id may be used again */
    CHECK("S1-H18", comms_dedup_check(&dd, &a, 1000u + COMMS_DEDUP_WINDOW_MS) == COMMS_DEDUP_NEW);
    /* the window works across the 32-bit millisecond wrap */
    comms_dedup_init(&dd);
    comms_dedup_add(&dd, &a, 0xFFFFFF00u);
    CHECK("S1-H18", comms_dedup_check(&dd, &a, 0x00000100u) == COMMS_DEDUP_REPEAT);
    /* only the last COMMS_DEDUP_LEN commands are kept */
    comms_dedup_init(&dd);
    comms_dedup_add(&dd, &a, 0u);
    for (i = 0u; i < COMMS_DEDUP_LEN; i++)
    {
        comms_cmd_t other;
        char text[64];

        snprintf(text, sizeof(text), "{\"id\":%u,\"cmd\":\"start\"}", 200u + i);
        (void)parse(text, &other);
        comms_dedup_add(&dd, &other, 10u + i);
    }
    CHECK("S1-H18", comms_dedup_check(&dd, &a, 100u) == COMMS_DEDUP_NEW);
    CHECK("S1-H18", dd.count == COMMS_DEDUP_LEN);
    /* STOP and ESTOP are always safe to repeat */
    CHECK("S1-H18", comms_cmd_repeat_safe(stop.cmd) && !comms_cmd_repeat_safe(a.cmd));
}

static void test_map(void)
{
    comms_cmd_t cmd;

    /* S1-H19: the barcode map arrives over MQTT (not hard-coded) */
    CHECK("S1-H19", parse("{\"id\":5,\"cmd\":\"map\",\"left\":\"A\",\"right\":\"F\","
                          "\"straight\":\"Z\",\"uturn\":\"W\"}", &cmd) == COMMS_ERR_NONE);
    CHECK("S1-H19", cmd.cmd == COMMS_CMD_MAP &&
                    (uint32_t)cmd.value == ((uint32_t)'A' | ((uint32_t)'F' << 8) | ((uint32_t)'Z' << 16) | ((uint32_t)'W' << 24)));
    CHECK("S1-H19", parse("{\"id\":5,\"cmd\":\"map\",\"uturn\":\"w\",\"straight\":\"z\","
                          "\"right\":\"f\",\"left\":\"a\"}", &cmd) == COMMS_ERR_NONE &&
                    (uint32_t)cmd.value == ((uint32_t)'A' | ((uint32_t)'F' << 8) | ((uint32_t)'Z' << 16) | ((uint32_t)'W' << 24)));
    CHECK("S1-H19", parse("{\"id\":5,\"cmd\":\"map\",\"left\":\"A\",\"right\":\"F\",\"straight\":\"Z\"}", &cmd)
                    == COMMS_ERR_BAD_MAP);                          /* U-turn missing */
    CHECK("S1-H19", parse("{\"id\":5,\"cmd\":\"map\",\"left\":\"A\",\"right\":\"A\",\"straight\":\"Z\",\"uturn\":\"W\"}", &cmd)
                    == COMMS_ERR_BAD_MAP);                          /* used twice */
    CHECK("S1-H19", parse("{\"id\":5,\"cmd\":\"map\",\"left\":\"*\",\"right\":\"F\",\"straight\":\"Z\",\"uturn\":\"W\"}", &cmd)
                    == COMMS_ERR_BAD_MAP);                          /* start/stop char */
    CHECK("S1-H19", parse("{\"id\":5,\"cmd\":\"map\",\"left\":\"AB\",\"right\":\"F\",\"straight\":\"Z\",\"uturn\":\"W\"}", &cmd)
                    == COMMS_ERR_BAD_MAP);
    CHECK("S1-H19", parse("{\"id\":5,\"cmd\":\"map\",\"left\":1,\"right\":\"F\",\"straight\":\"Z\",\"uturn\":\"W\"}", &cmd)
                    == COMMS_ERR_BAD_MAP);
    (void)parse("{\"id\":5,\"cmd\":\"map\",\"left\":\"A\",\"right\":\"F\",\"straight\":\"Z\",\"uturn\":\"W\"}", &cmd);
    CHECK("S1-H19", comms_check_cmd(&cmd, false) == COMMS_ERR_BUSY && comms_check_cmd(&cmd, true) == COMMS_ERR_NONE);
    CHECK("S1-H19", strcmp(comms_err_name(COMMS_ERR_BAD_MAP), "bad_map") == 0);
    CHECK("S1-H19", comms_map_char_ok('Z') && comms_map_char_ok('7') && !comms_map_char_ok('*') && !comms_map_char_ok('a'));

#if APP_BENCH
    CHECK("S1-H20", parse("{\"id\":7,\"cmd\":\"set\",\"param\":\"bench_turn\",\"value\":-900}", &cmd) == COMMS_ERR_NONE &&
                    cmd.param_id == 102u && cmd.value == -900);
    CHECK("S1-H20", parse("{\"id\":7,\"cmd\":\"set\",\"param\":\"bench_servo\",\"value\":181}", &cmd) == COMMS_ERR_RANGE);
#else
    CHECK("S1-H20", parse("{\"id\":7,\"cmd\":\"set\",\"param\":\"bench_turn\",\"value\":-900}", &cmd) == COMMS_ERR_BAD_PARAM);
#endif
    /* S1-H20: START needs a calibration; event names include the new events */
    (void)parse("{\"id\":6,\"cmd\":\"start\"}", &cmd);
    CHECK("S1-H20", comms_check_ready(&cmd, false) == COMMS_ERR_NOT_CAL);
    CHECK("S1-H20", comms_check_ready(&cmd, true) == COMMS_ERR_NONE);
    CHECK("S1-H20", strcmp(comms_err_name(COMMS_ERR_NOT_CAL), "not_calibrated") == 0);
    CHECK("S1-H20", strcmp(comms_event_name(27u), "REMOTE_MAP") == 0 && strcmp(comms_event_name(28u), "SAFETY_OK") == 0);
    CHECK("S1-H20", strcmp(comms_event_name(29u), "UNKNOWN") == 0);
}

int main(void)
{
    test_writer();
    test_telemetry();
    test_event_status_ack();
    test_parser();
    test_ring();
    test_button();
    test_dedup();
    test_map();
    printf("host tests: %d passed, %d failed\n", g_pass, g_fail);
    return (g_fail == 0) ? 0 : 1;
}
