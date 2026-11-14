/*
 *  test_s1.c
 *  Host unit tests for subsystem 1's pure modules: comms_proto.c (binary
 *  frame builders, command parser, packed telemetry record, offline ring)
 *  and hmi_button.c.
 *  Build and run:  make -C testing/subsystem1/host
 *  Test IDs S1-H.. are listed in testing/subsystem1/TEST_LOG.md.
 *
 *  Frames are checked byte for byte against golden vectors.  The same
 *  vectors are decoded by the laptop codec in test_codec.py (S1-H21), so
 *  the C encoder and the Python decoder are checked against each other.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "comms_proto.h"
#include "hmi_button.h"
#include "app_pack.h"
#include "golden.h"

static int g_pass;
static int g_fail;

#define CHECK(id, cond) \
    do { if (cond) { g_pass++; } else { g_fail++; \
        printf("FAIL %s  (%s:%d)  %s\n", id, __FILE__, __LINE__, #cond); } } while (0)

/* Build a command frame the way the laptop does (picocar_codec.py). */
static uint16_t cmd_frame(uint8_t * p_buf, uint8_t id, uint8_t cmd, int index, int32_t value,
                          char const * p_map)
{
    pk_writer_t w;
    int         k;

    pk_writer_init(&w, p_buf, 8u);
    pk_put(&w, COMMS_FT_CMD, 5u);
    pk_put(&w, COMMS_FRAME_VERSION, 3u);
    pk_put(&w, id, 8u);
    pk_put(&w, cmd, 3u);
    if (cmd == COMMS_CMD_SET)
    {
        pk_put(&w, (uint32_t)index, 4u);
        pk_put(&w, (uint32_t)(value + 32768) & 0xFFFFu, 16u);
    }
    if (cmd == COMMS_CMD_MAP)
    {
        for (k = 0; k < 4; k++)
        {
            pk_put(&w, comms_symbol_index(p_map[k]), 6u);
        }
    }
    return pk_writer_len(&w);
}

static comms_err_t parse(uint8_t id, uint8_t cmd, int index, int32_t value, char const * p_map,
                         comms_cmd_t * p_cmd)
{
    uint8_t  buf[8];
    uint16_t n = cmd_frame(buf, id, cmd, index, value, p_map);

    return comms_parse_cmd(buf, n, p_cmd);
}

static int param_index(char const * p_name)
{
    static char const * const names[] = { "speed_mm_s", "slow_cm", "stop_cm", "lost_mm", "junction_mm",
                                          "bench_move", "bench_turn", "bench_speed", "bench_servo" };
    int i;

    for (i = 0; i < 9; i++)
    {
        if (strcmp(names[i], p_name) == 0)
        {
            return i;
        }
    }
    return 15;
}

/* Skip n bits (any number). */
static void skip(pk_reader_t * p_r, unsigned n)
{
    while (n > 0u)
    {
        unsigned const k = (n > 32u) ? 32u : n;

        (void)pk_take(p_r, (uint8_t)k);
        n -= k;
    }
}

static int hex_eq(uint8_t const * p_buf, uint16_t n, char const * p_hex)
{
    char     text[2 * COMMS_FRAME_MAX + 1];
    uint16_t i;

    for (i = 0u; i < n; i++)
    {
        snprintf(&text[2u * i], 3u, "%02x", p_buf[i]);
    }
    text[2u * n] = '\0';
    if (strcmp(text, p_hex) != 0)
    {
        printf("  got %s\n  expected %s\n", text, p_hex);
        return 0;
    }
    return 1;
}

/* ------------------------------------------------------------------ */
static void test_writer(void)
{
    uint8_t     buf[8];
    pk_writer_t w;
    pk_reader_t r;

    /* LSB first: 3 bits 0b101, 7 bits 0x7F, 1 bit 0, 5 bits 0b10011 */
    pk_writer_init(&w, buf, sizeof(buf));
    pk_put(&w, 5u, 3u);
    pk_put(&w, 0x7Fu, 7u);
    pk_put(&w, 0u, 1u);
    pk_put(&w, 0x13u, 5u);
    CHECK("S1-H01", pk_writer_len(&w) == 2u && hex_eq(buf, 2u, "fd9b"));
    pk_reader_init(&r, buf, 2u);
    CHECK("S1-H01", pk_take(&r, 3u) == 5u && pk_take(&r, 7u) == 0x7Fu && pk_take(&r, 1u) == 0u &&
                    pk_take(&r, 5u) == 0x13u && !r.b_short);
    CHECK("S1-H01", pk_take(&r, 1u) == 0u && r.b_short);

    /* Never writes past cap, for every cap; overflow returns 0. */
    {
        comms_telem_t t;
        uint16_t      full;
        uint16_t      cap;
        uint8_t       big[64];

        memset(&t, 0, sizeof(t));
        t.obst_d_mm = COMMS_D_NONE;
        full = comms_build_telemetry(&t, big, sizeof(big));
        for (cap = 0u; cap <= (uint16_t)(full + 2u); cap++)
        {
            uint8_t  area[80];
            uint16_t len;
            uint16_t i;
            int      ok = 1;

            memset(area, 0x5A, sizeof(area));
            len = comms_build_telemetry(&t, area, cap);
            for (i = cap; i < sizeof(area); i++)
            {
                if (area[i] != 0x5Au) { ok = 0; }
            }
            CHECK("S1-H02", ok);
            CHECK("S1-H02", (cap >= full) ? (len == full) : (len == 0u));
        }
    }
}

static void test_telemetry(void)
{
    uint8_t       buf[COMMS_FRAME_MAX];
    comms_telem_t t;
    uint16_t      len;
    pk_reader_t   r;

    /* S1-H03: golden frame (decoded by test_codec.py) */
    memset(&t, 0, sizeof(t));
    t.seq = 12u;  t.t_ms = 3400u;  t.state = 3u;
    t.v_l_mm_s = 250;  t.v_r_mm_s = -10;
    t.enc_l = 1000;  t.enc_r = 990;  t.dist_mm = 555;
    t.line_err = -120;  t.b_line_lost = false;  t.barcode = 'A';
    t.pitch_cdeg = -1234;  t.roll_cdeg = 875;  t.hump_max_mm = 25u;
    t.obst_d_mm = 384u;  t.b_obst_valid = false;
    len = comms_build_telemetry(&t, buf, sizeof(buf));
    CHECK("S1-H03", len == 27u);
    CHECK("S1-H03", hex_eq(buf, len, GOLD_TELEMETRY));
    pk_reader_init(&r, buf, len);
    CHECK("S1-H03", pk_take(&r, 5u) == COMMS_FT_TELEMETRY && pk_take(&r, 3u) == COMMS_FRAME_VERSION);
    CHECK("S1-H03", pk_take(&r, 8u) == 12u && pk_take(&r, 24u) == 340u && pk_take(&r, 4u) == 3u);
    CHECK("S1-H03", pk_unoff_s(pk_take(&r, 10u), 10u) == 250 && pk_unoff_s(pk_take(&r, 10u), 10u) == -10);
    CHECK("S1-H03", pk_take(&r, 16u) == 1000u && pk_take(&r, 16u) == 990u && pk_take(&r, 16u) == 555u);
    CHECK("S1-H03", pk_unoff_s(pk_take(&r, 11u), 11u) == -120 && pk_take(&r, 1u) == 0u && pk_take(&r, 10u) == 0u);
    CHECK("S1-H03", comms_symbol_char((uint8_t)pk_take(&r, 6u)) == 'A' && pk_take(&r, 6u) == 0u);
    CHECK("S1-H03", pk_unoff_s(pk_take(&r, 11u), 11u) == -123 && pk_unoff_s(pk_take(&r, 12u), 12u) == 88);
    CHECK("S1-H03", pk_take(&r, 8u) == 25u && pk_take(&r, 9u) == 38u && pk_take(&r, 10u) == 1023u);
    CHECK("S1-H03", pk_take(&r, 7u) == 127u && pk_take(&r, 7u) == 127u && !r.b_short && r.bits == 210u);

    /* S1-H04: none codes, rounding, a valid profile */
    t.obst_d_mm = COMMS_D_NONE;  t.barcode = '\0';  t.pitch_cdeg = 5;  t.roll_cdeg = -1049;
    t.b_obst_valid = true;  t.obst_w_mm = 147u;  t.clr_l_mm = 204u;  t.clr_r_mm = 5000u;  t.nav = 'F';
    len = comms_build_telemetry(&t, buf, sizeof(buf));
    pk_reader_init(&r, buf, len);
    skip(&r, 8u + 8u + 24u + 4u + 10u + 10u + 16u + 16u + 16u + 11u + 1u + 10u);
    CHECK("S1-H04", pk_take(&r, 6u) == 0u && comms_symbol_char((uint8_t)pk_take(&r, 6u)) == 'F');
    CHECK("S1-H04", pk_unoff_s(pk_take(&r, 11u), 11u) == 1 && pk_unoff_s(pk_take(&r, 12u), 12u) == -105);
    (void)pk_take(&r, 8u);
    CHECK("S1-H04", pk_take(&r, 9u) == 511u && pk_take(&r, 10u) == 147u);
    CHECK("S1-H04", pk_take(&r, 7u) == 20u && pk_take(&r, 7u) == 126u);    /* 500 cm caps at 126 */

    /* S1-H05: worst case: every field saturates, still 27 bytes */
    t.seq = 4294967295u;  t.t_ms = 4294967295u;  t.state = 15u;
    t.v_l_mm_s = -32768;  t.v_r_mm_s = 32767;
    t.enc_l = -2147483647 - 1;  t.enc_r = -2147483647 - 1;  t.dist_mm = -2147483647 - 1;
    t.line_err = -32768;  t.b_line_lost = true;  t.barcode = 'Z';  t.line_bc = 65535u;
    t.pitch_cdeg = -32768;  t.roll_cdeg = 32767;  t.hump_max_mm = 65535u;  t.obst_d_mm = 65534u;
    t.b_obst_valid = true;  t.obst_w_mm = 65535u;  t.clr_l_mm = 65535u;  t.clr_r_mm = 65535u;
    t.nav = '%';
    len = comms_build_telemetry(&t, buf, sizeof(buf));
    pk_reader_init(&r, buf, len);
    skip(&r, 8u + 8u + 24u + 4u);
    CHECK("S1-H05", len == 27u);
    CHECK("S1-H05", pk_unoff_s(pk_take(&r, 10u), 10u) == -512 && pk_unoff_s(pk_take(&r, 10u), 10u) == 511);
    skip(&r, 48u);
    CHECK("S1-H05", pk_unoff_s(pk_take(&r, 11u), 11u) == -1024 && pk_take(&r, 1u) == 1u && pk_take(&r, 10u) == 1023u);
    (void)pk_take(&r, 12u);
    CHECK("S1-H05", pk_unoff_s(pk_take(&r, 11u), 11u) == -1024 && pk_unoff_s(pk_take(&r, 12u), 12u) == 2047);
    CHECK("S1-H05", pk_take(&r, 8u) == 255u && pk_take(&r, 9u) == 510u && pk_take(&r, 10u) == 1022u);
    printf("info: telemetry frame = %u B (+%u B topic); the JSON it replaces was 297 B typical, 340 B worst case\n",
           (unsigned)len, (unsigned)strlen("picocar/is23/telemetry"));
}

static void test_event_status_ack(void)
{
    uint8_t       buf[COMMS_FRAME_MAX];
    comms_event_t ev;
    comms_rec_t   rec;
    comms_cmd_t   cmd;
    uint16_t      len;

    /* S1-H06: events, the data width follows the value */
    ev.kind = COMMS_KIND_EVENT;  ev.src = 3u;  ev.code = 8u;  ev.data = 'B';  ev.t_ms = 770u;
    len = comms_build_event(&ev, 5u, buf, sizeof(buf));
    CHECK("S1-H06", len == 8u && hex_eq(buf, len, GOLD_EVENT_BARCODE));
    ev.kind = COMMS_KIND_STATE;  ev.code = 10u;  ev.data = 0;
    CHECK("S1-H06", comms_build_event(&ev, 6u, buf, sizeof(buf)) == 7u);
    ev.kind = COMMS_KIND_FAULT;  ev.code = 0u;  ev.data = 0x0214;
    CHECK("S1-H06", comms_build_event(&ev, 7u, buf, sizeof(buf)) == 9u);
    ev.kind = COMMS_KIND_LOG;  ev.code = 81u;  ev.data = 0x12345678;
    len = comms_build_event(&ev, 8u, buf, sizeof(buf));
    CHECK("S1-H06", len == 11u && hex_eq(buf, len, GOLD_EVENT_LOG));
    ev.data = -1;
    CHECK("S1-H06", comms_build_event(&ev, 9u, buf, sizeof(buf)) == 8u);
    ev.code = 999u;                                     /* saturates at 127 */
    len = comms_build_event(&ev, 9u, buf, sizeof(buf));
    {
        pk_reader_t r;

        pk_reader_init(&r, buf, len);
        skip(&r, 8u + 8u + 24u + 2u + 3u);
        CHECK("S1-H06", pk_take(&r, 7u) == 127u);
    }

    /* S1-H06: the packed mbf_telem record (8 B) and its time */
    CHECK("S1-H06", sizeof(comms_rec_t) == 8u);
    comms_rec_pack(&rec, COMMS_KIND_LOG, 4u, 79u, -123456, 123456u);
    comms_rec_unpack(&rec, 125000u, &ev);
    CHECK("S1-H06", ev.kind == COMMS_KIND_LOG && ev.src == 4u && ev.code == 79u &&
                    ev.data == -123456 && ev.t_ms == 123450u);
    comms_rec_pack(&rec, COMMS_KIND_EVENT, 9u, 300u, 7, 10485755u);  /* src, code saturate */
    comms_rec_unpack(&rec, 10485755u + 2000u, &ev);    /* across the 20-bit (2.9 h) wrap */
    CHECK("S1-H06", ev.kind == COMMS_KIND_EVENT && ev.src == 7u && ev.code == 127u && ev.t_ms == 10485750u);

    /* S1-H07: status, with and without the map; the Last Will */
    len = comms_build_status(61u, -58, 3u, NULL, buf, sizeof(buf));
    CHECK("S1-H07", len == 6u && hex_eq(buf, len, GOLD_STATUS));
    {
        char const map[4] = { 'Q', 'F', 'Z', 'W' };

        len = comms_build_status(1u, -60, 0u, map, buf, sizeof(buf));
        CHECK("S1-H07", len == 9u && hex_eq(buf, len, GOLD_STATUS_MAP));
    }
    len = comms_build_status(200000u, -300, 9999u, NULL, buf, sizeof(buf));   /* saturates */
    {
        pk_reader_t r;

        pk_reader_init(&r, buf, len);
        (void)pk_take(&r, 8u);
        CHECK("S1-H07", pk_take(&r, 17u) == 131071u && pk_take(&r, 7u) == 127u && pk_take(&r, 8u) == 255u);
    }
    CHECK("S1-H07", strlen(COMMS_WILL_MSG) == 1u && (uint8_t)COMMS_WILL_MSG[0] == 0x23u);

    /* S1-H08: acks */
    (void)parse(42u, COMMS_CMD_STOP, 0, 0, NULL, &cmd);
    len = comms_build_ack(&cmd, COMMS_ERR_NONE, buf, sizeof(buf));
    CHECK("S1-H08", len == 3u && hex_eq(buf, len, GOLD_ACK_OK));
    cmd.b_has_id = false;
    len = comms_build_ack(&cmd, COMMS_ERR_NO_ID, buf, sizeof(buf));
    CHECK("S1-H08", len == 3u && hex_eq(buf, len, GOLD_ACK_NO_ID));
}

static void test_parser(void)
{
    comms_cmd_t cmd;
    uint8_t     f[8];
    uint16_t    n;

    CHECK("S1-H09", parse(1u, COMMS_CMD_START, 0, 0, NULL, &cmd) == COMMS_ERR_NONE && cmd.cmd == COMMS_CMD_START && cmd.id == 1u);
    CHECK("S1-H09", parse(255u, COMMS_CMD_STOP, 0, 0, NULL, &cmd) == COMMS_ERR_NONE && cmd.cmd == COMMS_CMD_STOP && cmd.id == 255u);
    CHECK("S1-H09", parse(4u, COMMS_CMD_RESET, 0, 0, NULL, &cmd) == COMMS_ERR_NONE && cmd.cmd == COMMS_CMD_RESET);
    CHECK("S1-H09", parse(3u, 3u, 0, 0, NULL, &cmd) == COMMS_ERR_BAD_CMD);      /* 3: E-stop, removed 14 Nov */
    CHECK("S1-H09", parse(3u, 7u, 0, 0, NULL, &cmd) == COMMS_ERR_BAD_CMD);
    CHECK("S1-H09", parse(5u, COMMS_CMD_SET, param_index("stop_cm"), 25, NULL, &cmd) == COMMS_ERR_NONE
                    && cmd.cmd == COMMS_CMD_SET && cmd.param_id == 3u && cmd.value == 25);
    n = cmd_frame(f, 9u, COMMS_CMD_SET, param_index("speed_mm_s"), 250, NULL);
    CHECK("S1-H09", n == 5u && hex_eq(f, n, GOLD_CMD_SET));
    n = cmd_frame(f, 7u, COMMS_CMD_START, 0, 0, NULL);
    CHECK("S1-H09", n == 3u && hex_eq(f, n, GOLD_CMD_START));

    /* S1-H10: boundaries of every whitelisted parameter */
    {
        static char const * const names[] = { "speed_mm_s", "slow_cm", "stop_cm", "lost_mm", "junction_mm" };
        unsigned i;

        for (i = 0u; i < 5u; i++)
        {
            comms_param_t const * p = comms_param_by_index((uint8_t)i);

            CHECK("S1-H10", p != NULL && strcmp(p->p_name, names[i]) == 0 && comms_param_by_id(p->id) == p);
            CHECK("S1-H10", parse(9u, COMMS_CMD_SET, (int)i, p->min, NULL, &cmd) == COMMS_ERR_NONE && cmd.value == p->min);
            CHECK("S1-H10", parse(9u, COMMS_CMD_SET, (int)i, p->max, NULL, &cmd) == COMMS_ERR_NONE && cmd.value == p->max);
            CHECK("S1-H10", parse(9u, COMMS_CMD_SET, (int)i, p->min - 1, NULL, &cmd) == COMMS_ERR_RANGE);
            CHECK("S1-H10", parse(9u, COMMS_CMD_SET, (int)i, p->max + 1, NULL, &cmd) == COMMS_ERR_RANGE);
        }
        CHECK("S1-H10", parse(9u, COMMS_CMD_SET, 0, -32768, NULL, &cmd) == COMMS_ERR_RANGE && cmd.value == -32768);
    }

    /* S1-H11: rejections, each with its reason */
    CHECK("S1-H11", comms_parse_cmd(f, 0u, &cmd) == COMMS_ERR_BAD_FRAME);
    n = cmd_frame(f, 1u, COMMS_CMD_START, 0, 0, NULL);
    f[0] = 0x21u;                                                   /* telemetry type */
    CHECK("S1-H11", comms_parse_cmd(f, n, &cmd) == COMMS_ERR_BAD_FRAME);
    f[0] = 0x45u;                                                   /* version 2 */
    CHECK("S1-H11", comms_parse_cmd(f, n, &cmd) == COMMS_ERR_BAD_FRAME);
    f[0] = 0x25u;
    CHECK("S1-H11", comms_parse_cmd(f, 1u, &cmd) == COMMS_ERR_NO_ID && !cmd.b_has_id);
    CHECK("S1-H11", comms_parse_cmd(f, 2u, &cmd) == COMMS_ERR_BAD_FRAME && cmd.b_has_id && cmd.id == 1u);
    CHECK("S1-H11", comms_parse_cmd(f, 4u, &cmd) == COMMS_ERR_BAD_FRAME);       /* START is 3 B */
    n = cmd_frame(f, 1u, COMMS_CMD_SET, 2, 20, NULL);
    CHECK("S1-H11", comms_parse_cmd(f, (uint16_t)(n - 1u), &cmd) == COMMS_ERR_BAD_FRAME);
    CHECK("S1-H11", parse(1u, COMMS_CMD_SET, 15, 20, NULL, &cmd) == COMMS_ERR_BAD_PARAM);
#if !APP_BENCH
    CHECK("S1-H11", parse(1u, COMMS_CMD_SET, 5, 20, NULL, &cmd) == COMMS_ERR_BAD_PARAM);
#endif
    CHECK("S1-H11", parse(1u, COMMS_CMD_SET, 2, 99, NULL, &cmd) == COMMS_ERR_RANGE);

    /* S1-H12: the length is exact: one byte more or less is a bad frame */
    n = cmd_frame(f, 1u, COMMS_CMD_STOP, 0, 0, NULL);
    f[n] = 0u;
    CHECK("S1-H12", comms_parse_cmd(f, (uint16_t)(n + 1u), &cmd) == COMMS_ERR_BAD_FRAME);
    CHECK("S1-H12", comms_parse_cmd(f, n, &cmd) == COMMS_ERR_NONE);

    /* S1-H13: state rule (FR14) */
    (void)parse(1u, COMMS_CMD_SET, 2, 20, NULL, &cmd);
    CHECK("S1-H13", comms_check_cmd(&cmd, false) == COMMS_ERR_BUSY);
    CHECK("S1-H13", comms_check_cmd(&cmd, true) == COMMS_ERR_NONE);
    (void)parse(1u, COMMS_CMD_STOP, 0, 0, NULL, &cmd);
    CHECK("S1-H13", comms_check_cmd(&cmd, false) == COMMS_ERR_NONE);

    /* S1-H14: random bytes and every truncation never crash (ASan/UBSan) */
    {
        uint8_t  junk[64];
        unsigned k;
        unsigned m;

        n = cmd_frame(f, 5u, COMMS_CMD_MAP, 0, 0, "AFZW");
        for (m = 0u; m <= n; m++)
        {
            (void)comms_parse_cmd(f, (uint16_t)m, &cmd);
        }
        srand(1234);
        for (k = 0u; k < 20000u; k++)
        {
            unsigned len = (unsigned)rand() % sizeof(junk);

            for (m = 0u; m < len; m++)
            {
                junk[m] = (uint8_t)rand();
            }
            if ((k & 1u) && (len > 0u))
            {
                junk[0] = 0x25u;                /* a valid header, junk after it */
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
    uint8_t      seq;
    uint32_t     i;
    int          overwrote = 0;

    comms_ring_init(&ring);
    CHECK("S1-H15", !comms_ring_peek(&ring, &rec, &seq));
    memset(&rec, 0, sizeof(rec));
    for (i = 1u; i <= 20u; i++)
    {
        rec.data = (int32_t)i;
        overwrote += comms_ring_push(&ring, &rec, (uint8_t)(i + 250u)) ? 1 : 0;   /* seq wraps */
    }
    CHECK("S1-H15", overwrote == 4 && ring.count == COMMS_RING_LEN);
    for (i = 5u; i <= 20u; i++)
    {
        CHECK("S1-H15", comms_ring_peek(&ring, &rec, &seq) && seq == (uint8_t)(i + 250u) && rec.data == (int32_t)i);
        comms_ring_pop(&ring);
    }
    CHECK("S1-H15", ring.count == 0u && !comms_ring_peek(&ring, &rec, &seq));
    comms_ring_pop(&ring);  /* pop on empty is harmless */
    CHECK("S1-H15", ring.count == 0u);
    CHECK("S1-H15", sizeof(ring) <= (COMMS_RING_LEN * 9u) + 4u);
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
    (void)parse(7u, COMMS_CMD_START, 0, 0, NULL, &a);
    (void)parse(7u, COMMS_CMD_RESET, 0, 0, NULL, &b);
    (void)parse(9u, COMMS_CMD_STOP, 0, 0, NULL, &stop);

    CHECK("S1-H18", comms_dedup_check(&dd, &a, 1000u) == COMMS_DEDUP_NEW);
    comms_dedup_add(&dd, &a, 1000u);
    /* delayed redelivery after other commands, inside the window */
    for (i = 0u; i < 5u; i++)
    {
        comms_cmd_t other;

        (void)parse((uint8_t)(100u + i), COMMS_CMD_START, 0, 0, NULL, &other);
        comms_dedup_add(&dd, &other, 2000u + i);
    }
    CHECK("S1-H18", comms_dedup_check(&dd, &a, 30000u) == COMMS_DEDUP_REPEAT);
    CHECK("S1-H18", comms_dedup_check(&dd, &b, 30000u) == COMMS_DEDUP_CONFLICT);
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

        (void)parse((uint8_t)(200u + i), COMMS_CMD_START, 0, 0, NULL, &other);
        comms_dedup_add(&dd, &other, 10u + i);
    }
    CHECK("S1-H18", comms_dedup_check(&dd, &a, 100u) == COMMS_DEDUP_NEW);
    CHECK("S1-H18", dd.count == COMMS_DEDUP_LEN);
    /* STOP is always safe to repeat */
    CHECK("S1-H18", comms_cmd_repeat_safe(stop.cmd) && !comms_cmd_repeat_safe(a.cmd));
}

static void test_map(void)
{
    comms_cmd_t cmd;
    uint8_t     f[8];
    uint16_t    n;
    uint32_t const afzw = (uint32_t)'A' | ((uint32_t)'F' << 8) | ((uint32_t)'Z' << 16) | ((uint32_t)'W' << 24);

    /* S1-H19: the barcode map arrives over MQTT (not hard-coded) */
    n = cmd_frame(f, 5u, COMMS_CMD_MAP, 0, 0, "AFZW");
    CHECK("S1-H19", n == 6u && hex_eq(f, n, GOLD_CMD_MAP));
    CHECK("S1-H19", comms_parse_cmd(f, n, &cmd) == COMMS_ERR_NONE && cmd.cmd == COMMS_CMD_MAP &&
                    (uint32_t)cmd.value == afzw);
    CHECK("S1-H19", parse(5u, COMMS_CMD_MAP, 0, 0, "AAZW", &cmd) == COMMS_ERR_BAD_MAP);   /* used twice */
    f[2] = (uint8_t)(f[2] & 0x07u);             /* left symbol 0 (none) */
    f[3] = (uint8_t)(f[3] & 0xFEu);
    CHECK("S1-H19", comms_parse_cmd(f, n, &cmd) == COMMS_ERR_BAD_MAP);
    n = cmd_frame(f, 5u, COMMS_CMD_MAP, 0, 0, "AFZW");
    f[2] = (uint8_t)(f[2] | 0xF8u);             /* left symbol 63: not a symbol */
    f[3] = (uint8_t)(f[3] | 0x01u);
    CHECK("S1-H19", comms_parse_cmd(f, n, &cmd) == COMMS_ERR_BAD_MAP);
    (void)parse(5u, COMMS_CMD_MAP, 0, 0, "AFZW", &cmd);
    CHECK("S1-H19", comms_check_cmd(&cmd, false) == COMMS_ERR_BUSY && comms_check_cmd(&cmd, true) == COMMS_ERR_NONE);
    CHECK("S1-H19", comms_map_char_ok('Z') && comms_map_char_ok('7') && !comms_map_char_ok('*') && !comms_map_char_ok('a'));
    CHECK("S1-H19", comms_symbol_index('0') == 1u && comms_symbol_index('%') == 43u && comms_symbol_index('*') == 0u &&
                    comms_symbol_char(43u) == '%' && comms_symbol_char(44u) == '\0');

#if APP_BENCH
    CHECK("S1-H20", parse(7u, COMMS_CMD_SET, param_index("bench_turn"), -900, NULL, &cmd) == COMMS_ERR_NONE &&
                    cmd.param_id == 102u && cmd.value == -900);
    CHECK("S1-H20", parse(7u, COMMS_CMD_SET, param_index("bench_servo"), 181, NULL, &cmd) == COMMS_ERR_RANGE);
#else
    CHECK("S1-H20", parse(7u, COMMS_CMD_SET, param_index("bench_turn"), -900, NULL, &cmd) == COMMS_ERR_BAD_PARAM);
#endif
    /* S1-H20: START needs a calibration */
    (void)parse(6u, COMMS_CMD_START, 0, 0, NULL, &cmd);
    CHECK("S1-H20", comms_check_ready(&cmd, false) == COMMS_ERR_NOT_CAL);
    CHECK("S1-H20", comms_check_ready(&cmd, true) == COMMS_ERR_NONE);
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
