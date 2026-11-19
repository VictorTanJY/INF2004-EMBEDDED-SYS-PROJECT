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
static uint16_t cmd_frame(uint8_t * p_buf, uint8_t ident, uint8_t cmd, int index, int32_t value,
                          char const * p_map)
{
    pk_writer_t writer;
    int         kdx;

    pk_writer_init(&writer, p_buf, 8u);
    pk_put(&writer, COMMS_FT_CMD, 5u);
    pk_put(&writer, COMMS_FRAME_VERSION, 3u);
    pk_put(&writer, ident, 8u);
    pk_put(&writer, cmd, 3u);
    if (cmd == COMMS_CMD_SET)
    {
        pk_put(&writer, (uint32_t)index, 4u);
        pk_put(&writer, (uint32_t)(value + 32768) & 0xFFFFu, 16u);
    }
    if (cmd == COMMS_CMD_MAP)
    {
        for (kdx = 0; kdx < 4; kdx++)
        {
            pk_put(&writer, comms_symbol_index(p_map[kdx]), 6u);
        }
    }
    return pk_writer_len(&writer);
}

static comms_err_t parse(uint8_t ident, uint8_t cmd, int index, int32_t value, char const * p_map,
                         comms_cmd_t * p_cmd)
{
    uint8_t  buf[8];
    uint16_t count = cmd_frame(buf, ident, cmd, index, value, p_map);

    return comms_parse_cmd(buf, count, p_cmd);
}

static int param_index(char const * p_name)
{
    static char const * const p_names[] = { "speed_mm_s", "slow_cm", "stop_cm", "lost_mm", "junction_mm",
                                          "bench_move", "bench_turn", "bench_speed" };
    int idx;

    for (idx = 0; idx < 9; idx++)
    {
        if (strcmp(p_names[idx], p_name) == 0)
        {
            return idx;
        }
    }
    return 15;
}

/* Skip n bits (any number). */
static void skip(pk_reader_t * p_r, unsigned count)
{
    while (count > 0u)
    {
        unsigned const kdx = (count > 32u) ? 32u : count;

        (void)pk_take(p_r, (uint8_t)kdx);
        count -= kdx;
    }
}

static int hex_eq(uint8_t const * p_buf, uint16_t count, char const * p_hex)
{
    char     text[2 * COMMS_FRAME_MAX + 1];
    uint16_t idx;

    for (idx = 0u; idx < count; idx++)
    {
        snprintf(&text[2u * idx], 3u, "%02x", p_buf[idx]);
    }
    text[2u * count] = '\0';
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
    pk_writer_t writer;
    pk_reader_t reader;

    /* LSB first: 3 bits 0b101, 7 bits 0x7F, 1 bit 0, 5 bits 0b10011 */
    pk_writer_init(&writer, buf, sizeof(buf));
    pk_put(&writer, 5u, 3u);
    pk_put(&writer, 0x7Fu, 7u);
    pk_put(&writer, 0u, 1u);
    pk_put(&writer, 0x13u, 5u);
    CHECK("S1-H01", pk_writer_len(&writer) == 2u && hex_eq(buf, 2u, "fd9b"));
    pk_reader_init(&reader, buf, 2u);
    CHECK("S1-H01", pk_take(&reader, 3u) == 5u && pk_take(&reader, 7u) == 0x7Fu && pk_take(&reader, 1u) == 0u &&
                    pk_take(&reader, 5u) == 0x13u && !reader.b_short);
    CHECK("S1-H01", pk_take(&reader, 1u) == 0u && reader.b_short);

    /* Never writes past cap, for every cap; overflow returns 0. */
    {
        comms_telem_t telem;
        uint16_t      full;
        uint16_t      cap;
        uint8_t       big[64];

        memset(&telem, 0, sizeof(telem));
        telem.obst_d_mm = COMMS_D_NONE;
        full = comms_build_telemetry(&telem, big, sizeof(big));
        for (cap = 0u; cap <= (uint16_t)(full + 2u); cap++)
        {
            uint8_t  area[80];
            uint16_t len;
            uint16_t idx;
            int      b_ok = 1;

            memset(area, 0x5A, sizeof(area));
            len = comms_build_telemetry(&telem, area, cap);
            for (idx = cap; idx < sizeof(area); idx++)
            {
                if (area[idx] != 0x5Au) { b_ok = 0; }
            }
            CHECK("S1-H02", b_ok);
            CHECK("S1-H02", (cap >= full) ? (len == full) : (len == 0u));
        }
    }
}

static void test_telemetry(void)
{
    uint8_t       buf[COMMS_FRAME_MAX];
    comms_telem_t telem;
    uint16_t      len;
    pk_reader_t   reader;

    /* S1-H03: golden frame (decoded by test_codec.py) */
    memset(&telem, 0, sizeof(telem));
    telem.seq = 12u;  telem.t_ms = 3400u;  telem.state = 3u;
    telem.v_l_mm_s = 250;  telem.v_r_mm_s = -10;
    telem.enc_l = 1000;  telem.enc_r = 990;  telem.dist_mm = 555;
    telem.line_err = -120;  telem.b_line_lost = false;  telem.barcode = 'A';
    telem.pitch_cdeg = -1234;  telem.roll_cdeg = 875;  telem.hump_max_mm = 25u;
    telem.obst_d_mm = 384u;  telem.b_obst_valid = false;
    len = comms_build_telemetry(&telem, buf, sizeof(buf));
    CHECK("S1-H03", len == 27u);
    CHECK("S1-H03", hex_eq(buf, len, GOLD_TELEMETRY));
    pk_reader_init(&reader, buf, len);
    CHECK("S1-H03", pk_take(&reader, 5u) == COMMS_FT_TELEMETRY && pk_take(&reader, 3u) == COMMS_FRAME_VERSION);
    CHECK("S1-H03", pk_take(&reader, 8u) == 12u && pk_take(&reader, 24u) == 340u && pk_take(&reader, 4u) == 3u);
    CHECK("S1-H03", pk_unoff_s(pk_take(&reader, 10u), 10u) == 250 && pk_unoff_s(pk_take(&reader, 10u), 10u) == -10);
    CHECK("S1-H03", pk_take(&reader, 16u) == 1000u && pk_take(&reader, 16u) == 990u && pk_take(&reader, 16u) == 555u);
    CHECK("S1-H03", pk_unoff_s(pk_take(&reader, 11u), 11u) == -120 && pk_take(&reader, 1u) == 0u && pk_take(&reader, 10u) == 0u);
    CHECK("S1-H03", comms_symbol_char((uint8_t)pk_take(&reader, 6u)) == 'A' && pk_take(&reader, 6u) == 0u);
    CHECK("S1-H03", pk_unoff_s(pk_take(&reader, 11u), 11u) == -123 && pk_unoff_s(pk_take(&reader, 12u), 12u) == 88);
    CHECK("S1-H03", pk_take(&reader, 8u) == 25u && pk_take(&reader, 9u) == 38u && pk_take(&reader, 10u) == 1023u);
    CHECK("S1-H03", pk_take(&reader, 7u) == 127u && pk_take(&reader, 7u) == 127u && !reader.b_short && reader.bits == 210u);

    /* S1-H04: none codes, rounding, a valid profile */
    telem.obst_d_mm = COMMS_D_NONE;  telem.barcode = '\0';  telem.pitch_cdeg = 5;  telem.roll_cdeg = -1049;
    telem.b_obst_valid = true;  telem.obst_w_mm = 147u;  telem.clr_l_mm = 204u;  telem.clr_r_mm = 5000u;  telem.nav = 'F';
    len = comms_build_telemetry(&telem, buf, sizeof(buf));
    pk_reader_init(&reader, buf, len);
    skip(&reader, 8u + 8u + 24u + 4u + 10u + 10u + 16u + 16u + 16u + 11u + 1u + 10u);
    CHECK("S1-H04", pk_take(&reader, 6u) == 0u && comms_symbol_char((uint8_t)pk_take(&reader, 6u)) == 'F');
    CHECK("S1-H04", pk_unoff_s(pk_take(&reader, 11u), 11u) == 1 && pk_unoff_s(pk_take(&reader, 12u), 12u) == -105);
    (void)pk_take(&reader, 8u);
    CHECK("S1-H04", pk_take(&reader, 9u) == 511u && pk_take(&reader, 10u) == 147u);
    CHECK("S1-H04", pk_take(&reader, 7u) == 20u && pk_take(&reader, 7u) == 126u);    /* 500 cm caps at 126 */

    /* S1-H05: worst case: every field saturates, still 27 bytes */
    telem.seq = 4294967295u;  telem.t_ms = 4294967295u;  telem.state = 15u;
    telem.v_l_mm_s = -32768;  telem.v_r_mm_s = 32767;
    telem.enc_l = -2147483647 - 1;  telem.enc_r = -2147483647 - 1;  telem.dist_mm = -2147483647 - 1;
    telem.line_err = -32768;  telem.b_line_lost = true;  telem.barcode = 'Z';  telem.line_bc = 65535u;
    telem.pitch_cdeg = -32768;  telem.roll_cdeg = 32767;  telem.hump_max_mm = 65535u;  telem.obst_d_mm = 65534u;
    telem.b_obst_valid = true;  telem.obst_w_mm = 65535u;  telem.clr_l_mm = 65535u;  telem.clr_r_mm = 65535u;
    telem.nav = '%';
    len = comms_build_telemetry(&telem, buf, sizeof(buf));
    pk_reader_init(&reader, buf, len);
    skip(&reader, 8u + 8u + 24u + 4u);
    CHECK("S1-H05", len == 27u);
    CHECK("S1-H05", pk_unoff_s(pk_take(&reader, 10u), 10u) == -512 && pk_unoff_s(pk_take(&reader, 10u), 10u) == 511);
    skip(&reader, 48u);
    CHECK("S1-H05", pk_unoff_s(pk_take(&reader, 11u), 11u) == -1024 && pk_take(&reader, 1u) == 1u && pk_take(&reader, 10u) == 1023u);
    (void)pk_take(&reader, 12u);
    CHECK("S1-H05", pk_unoff_s(pk_take(&reader, 11u), 11u) == -1024 && pk_unoff_s(pk_take(&reader, 12u), 12u) == 2047);
    CHECK("S1-H05", pk_take(&reader, 8u) == 255u && pk_take(&reader, 9u) == 510u && pk_take(&reader, 10u) == 1022u);
    printf("info: telemetry frame = %u B (+%u B topic); the JSON it replaces was 297 B typical, 340 B worst case\n",
           (unsigned)len, (unsigned)strlen("picocar/is23/telemetry"));
}

static void test_event_status_ack(void)
{
    uint8_t       buf[COMMS_FRAME_MAX];
    comms_event_t event;
    comms_rec_t   rec;
    comms_cmd_t   cmd;
    uint16_t      len;

    /* S1-H06: events, the data width follows the value */
    event.kind = COMMS_KIND_EVENT;  event.src = 3u;  event.code = 8u;  event.data = 'B';  event.t_ms = 770u;
    len = comms_build_event(&event, 5u, buf, sizeof(buf));
    CHECK("S1-H06", len == 8u && hex_eq(buf, len, GOLD_EVENT_BARCODE));
    event.kind = COMMS_KIND_STATE;  event.code = 10u;  event.data = 0;
    CHECK("S1-H06", comms_build_event(&event, 6u, buf, sizeof(buf)) == 7u);
    event.kind = COMMS_KIND_FAULT;  event.code = 0u;  event.data = 0x0214;
    CHECK("S1-H06", comms_build_event(&event, 7u, buf, sizeof(buf)) == 9u);
    event.kind = COMMS_KIND_LOG;  event.code = 81u;  event.data = 0x12345678;
    len = comms_build_event(&event, 8u, buf, sizeof(buf));
    CHECK("S1-H06", len == 11u && hex_eq(buf, len, GOLD_EVENT_LOG));
    event.data = -1;
    CHECK("S1-H06", comms_build_event(&event, 9u, buf, sizeof(buf)) == 8u);
    event.code = 999u;                                     /* saturates at 127 */
    len = comms_build_event(&event, 9u, buf, sizeof(buf));
    {
        pk_reader_t reader;

        pk_reader_init(&reader, buf, len);
        skip(&reader, 8u + 8u + 24u + 2u + 3u);
        CHECK("S1-H06", pk_take(&reader, 7u) == 127u);
    }

    /* S1-H06: the packed mbf_telem record (8 B) and its time */
    CHECK("S1-H06", sizeof(comms_rec_t) == 8u);
    comms_rec_pack(&rec, COMMS_KIND_LOG, 4u, 79u, -123456, 123456u);
    comms_rec_unpack(&rec, 125000u, &event);
    CHECK("S1-H06", event.kind == COMMS_KIND_LOG && event.src == 4u && event.code == 79u &&
                    event.data == -123456 && event.t_ms == 123450u);
    comms_rec_pack(&rec, COMMS_KIND_EVENT, 9u, 300u, 7, 10485755u);  /* src, code saturate */
    comms_rec_unpack(&rec, 10485755u + 2000u, &event);    /* across the 20-bit (2.9 h) wrap */
    CHECK("S1-H06", event.kind == COMMS_KIND_EVENT && event.src == 7u && event.code == 127u && event.t_ms == 10485750u);

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
        pk_reader_t reader;

        pk_reader_init(&reader, buf, len);
        (void)pk_take(&reader, 8u);
        CHECK("S1-H07", pk_take(&reader, 17u) == 131071u && pk_take(&reader, 7u) == 127u && pk_take(&reader, 8u) == 255u);
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
    uint8_t     frame[8];
    uint16_t    count;

    CHECK("S1-H09", parse(1u, COMMS_CMD_START, 0, 0, NULL, &cmd) == COMMS_ERR_NONE && cmd.cmd == COMMS_CMD_START && cmd.cmd_id == 1u);
    CHECK("S1-H09", parse(255u, COMMS_CMD_STOP, 0, 0, NULL, &cmd) == COMMS_ERR_NONE && cmd.cmd == COMMS_CMD_STOP && cmd.cmd_id == 255u);
    CHECK("S1-H09", parse(4u, COMMS_CMD_RESET, 0, 0, NULL, &cmd) == COMMS_ERR_NONE && cmd.cmd == COMMS_CMD_RESET);
    CHECK("S1-H09", parse(3u, 3u, 0, 0, NULL, &cmd) == COMMS_ERR_BAD_CMD);      /* 3: E-stop, removed 14 Nov */
    CHECK("S1-H09", parse(3u, 7u, 0, 0, NULL, &cmd) == COMMS_ERR_BAD_CMD);
    CHECK("S1-H09", parse(5u, COMMS_CMD_SET, param_index("stop_cm"), 25, NULL, &cmd) == COMMS_ERR_NONE
                    && cmd.cmd == COMMS_CMD_SET && cmd.param_id == 3u && cmd.value == 25);
    count = cmd_frame(frame, 9u, COMMS_CMD_SET, param_index("speed_mm_s"), 250, NULL);
    CHECK("S1-H09", count == 5u && hex_eq(frame, count, GOLD_CMD_SET));
    count = cmd_frame(frame, 7u, COMMS_CMD_START, 0, 0, NULL);
    CHECK("S1-H09", count == 3u && hex_eq(frame, count, GOLD_CMD_START));

    /* S1-H10: boundaries of every whitelisted parameter */
    {
        static char const * const p_names[] = { "speed_mm_s", "slow_cm", "stop_cm", "lost_mm", "junction_mm" };
        unsigned idx;

        for (idx = 0u; idx < 5u; idx++)
        {
            comms_param_t const * p_param = comms_param_by_index((uint8_t)idx);

            CHECK("S1-H10", p_param != NULL && strcmp(p_param->p_name, p_names[idx]) == 0 && comms_param_by_id(p_param->param_id) == p_param);
            CHECK("S1-H10", parse(9u, COMMS_CMD_SET, (int)idx, p_param->min, NULL, &cmd) == COMMS_ERR_NONE && cmd.value == p_param->min);
            CHECK("S1-H10", parse(9u, COMMS_CMD_SET, (int)idx, p_param->max, NULL, &cmd) == COMMS_ERR_NONE && cmd.value == p_param->max);
            CHECK("S1-H10", parse(9u, COMMS_CMD_SET, (int)idx, p_param->min - 1, NULL, &cmd) == COMMS_ERR_RANGE);
            CHECK("S1-H10", parse(9u, COMMS_CMD_SET, (int)idx, p_param->max + 1, NULL, &cmd) == COMMS_ERR_RANGE);
        }
        CHECK("S1-H10", parse(9u, COMMS_CMD_SET, 0, -32768, NULL, &cmd) == COMMS_ERR_RANGE && cmd.value == -32768);
    }

    /* S1-H11: rejections, each with its reason */
    CHECK("S1-H11", comms_parse_cmd(frame, 0u, &cmd) == COMMS_ERR_BAD_FRAME);
    count = cmd_frame(frame, 1u, COMMS_CMD_START, 0, 0, NULL);
    frame[0] = 0x21u;                                                   /* telemetry type */
    CHECK("S1-H11", comms_parse_cmd(frame, count, &cmd) == COMMS_ERR_BAD_FRAME);
    frame[0] = 0x45u;                                                   /* version 2 */
    CHECK("S1-H11", comms_parse_cmd(frame, count, &cmd) == COMMS_ERR_BAD_FRAME);
    frame[0] = 0x25u;
    CHECK("S1-H11", comms_parse_cmd(frame, 1u, &cmd) == COMMS_ERR_NO_ID && !cmd.b_has_id);
    CHECK("S1-H11", comms_parse_cmd(frame, 2u, &cmd) == COMMS_ERR_BAD_FRAME && cmd.b_has_id && cmd.cmd_id == 1u);
    CHECK("S1-H11", comms_parse_cmd(frame, 4u, &cmd) == COMMS_ERR_BAD_FRAME);       /* START is 3 B */
    count = cmd_frame(frame, 1u, COMMS_CMD_SET, 2, 20, NULL);
    CHECK("S1-H11", comms_parse_cmd(frame, (uint16_t)(count - 1u), &cmd) == COMMS_ERR_BAD_FRAME);
    CHECK("S1-H11", parse(1u, COMMS_CMD_SET, 15, 20, NULL, &cmd) == COMMS_ERR_BAD_PARAM);
#if !APP_BENCH
    CHECK("S1-H11", parse(1u, COMMS_CMD_SET, 5, 20, NULL, &cmd) == COMMS_ERR_BAD_PARAM);
#endif
    CHECK("S1-H11", parse(1u, COMMS_CMD_SET, 2, 99, NULL, &cmd) == COMMS_ERR_RANGE);

    /* S1-H12: the length is exact: one byte more or less is a bad frame */
    count = cmd_frame(frame, 1u, COMMS_CMD_STOP, 0, 0, NULL);
    frame[count] = 0u;
    CHECK("S1-H12", comms_parse_cmd(frame, (uint16_t)(count + 1u), &cmd) == COMMS_ERR_BAD_FRAME);
    CHECK("S1-H12", comms_parse_cmd(frame, count, &cmd) == COMMS_ERR_NONE);

    /* S1-H13: state rule (FR14) */
    (void)parse(1u, COMMS_CMD_SET, 2, 20, NULL, &cmd);
    CHECK("S1-H13", comms_check_cmd(&cmd, false) == COMMS_ERR_BUSY);
    CHECK("S1-H13", comms_check_cmd(&cmd, true) == COMMS_ERR_NONE);
    (void)parse(1u, COMMS_CMD_STOP, 0, 0, NULL, &cmd);
    CHECK("S1-H13", comms_check_cmd(&cmd, false) == COMMS_ERR_NONE);

    /* S1-H14: random bytes and every truncation never crash (ASan/UBSan) */
    {
        uint8_t  junk[64];
        unsigned kdx;
        unsigned mid;

        count = cmd_frame(frame, 5u, COMMS_CMD_MAP, 0, 0, "AFZW");
        for (mid = 0u; mid <= count; mid++)
        {
            (void)comms_parse_cmd(frame, (uint16_t)mid, &cmd);
        }
        srand(1234);
        for (kdx = 0u; kdx < 20000u; kdx++)
        {
            unsigned len = (unsigned)rand() % sizeof(junk);

            for (mid = 0u; mid < len; mid++)
            {
                junk[mid] = (uint8_t)rand();
            }
            if ((kdx & 1u) && (len > 0u))
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
    uint32_t     idx;
    int          overwrote = 0;

    comms_ring_init(&ring);
    CHECK("S1-H15", !comms_ring_peek(&ring, &rec, &seq));
    memset(&rec, 0, sizeof(rec));
    for (idx = 1u; idx <= 20u; idx++)
    {
        rec.data = (int32_t)idx;
        overwrote += comms_ring_push(&ring, &rec, (uint8_t)(idx + 250u)) ? 1 : 0;   /* seq wraps */
    }
    CHECK("S1-H15", overwrote == 4 && ring.count == COMMS_RING_LEN);
    for (idx = 5u; idx <= 20u; idx++)
    {
        CHECK("S1-H15", comms_ring_peek(&ring, &rec, &seq) && seq == (uint8_t)(idx + 250u) && rec.data == (int32_t)idx);
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
    int       count = 0;

    hmi_btn_init(&btn);
    for (; *p_pattern != '\0'; p_pattern++)
    {
        hmi_btn_ev_t event = hmi_btn_update(&btn, *p_pattern == '1', 20u, long_ms);

        if (event == HMI_BTN_EV_PRESS) { p_events[count++] = 'P'; }
        if (event == HMI_BTN_EV_SHORT) { p_events[count++] = 'S'; }
        if (event == HMI_BTN_EV_LONG)  { p_events[count++] = 'L'; }
    }
    p_events[count] = '\0';
    return count;
}

static void test_button(void)
{
    char ev_name[32];
    char pattern[200];

    (void)run_button("0101010000", 2000u, ev_name);           /* bounce only          */
    CHECK("S1-H16", strcmp(ev_name, "") == 0);
    (void)run_button("0110000", 2000u, ev_name);              /* 40 ms press          */
    CHECK("S1-H16", strcmp(ev_name, "PS") == 0);
    (void)run_button("01111101111100", 2000u, ev_name);       /* 1-sample drop inside */
    CHECK("S1-H16", strcmp(ev_name, "PS") == 0);

    memset(pattern, '1', 150);                           /* 3 s hold             */
    pattern[0] = '0';
    strcpy(&pattern[150], "000");
    (void)run_button(pattern, 2000u, ev_name);
    CHECK("S1-H17", strcmp(ev_name, "PL") == 0);              /* one LONG, no SHORT   */

    memset(pattern, '1', 99);                            /* release at 1.96 s    */
    pattern[0] = '0';
    strcpy(&pattern[99], "00");
    (void)run_button(pattern, 2000u, ev_name);
    CHECK("S1-H17", strcmp(ev_name, "PS") == 0);
}

static void test_dedup(void)
{
    comms_dedup_t dedup;
    comms_cmd_t   cmd_a, cmd_b, stop;
    unsigned      idx;

    comms_dedup_init(&dedup);
    (void)parse(7u, COMMS_CMD_START, 0, 0, NULL, &cmd_a);
    (void)parse(7u, COMMS_CMD_RESET, 0, 0, NULL, &cmd_b);
    (void)parse(9u, COMMS_CMD_STOP, 0, 0, NULL, &stop);

    CHECK("S1-H18", comms_dedup_check(&dedup, &cmd_a, 1000u) == COMMS_DEDUP_NEW);
    comms_dedup_add(&dedup, &cmd_a, 1000u);
    /* delayed redelivery after other commands, inside the window */
    for (idx = 0u; idx < 5u; idx++)
    {
        comms_cmd_t other;

        (void)parse((uint8_t)(100u + idx), COMMS_CMD_START, 0, 0, NULL, &other);
        comms_dedup_add(&dedup, &other, 2000u + idx);
    }
    CHECK("S1-H18", comms_dedup_check(&dedup, &cmd_a, 30000u) == COMMS_DEDUP_REPEAT);
    CHECK("S1-H18", comms_dedup_check(&dedup, &cmd_b, 30000u) == COMMS_DEDUP_CONFLICT);
    /* outside the window the id may be used again */
    CHECK("S1-H18", comms_dedup_check(&dedup, &cmd_a, 1000u + COMMS_DEDUP_WINDOW_MS) == COMMS_DEDUP_NEW);
    /* the window works across the 32-bit millisecond wrap */
    comms_dedup_init(&dedup);
    comms_dedup_add(&dedup, &cmd_a, 0xFFFFFF00u);
    CHECK("S1-H18", comms_dedup_check(&dedup, &cmd_a, 0x00000100u) == COMMS_DEDUP_REPEAT);
    /* only the last COMMS_DEDUP_LEN commands are kept */
    comms_dedup_init(&dedup);
    comms_dedup_add(&dedup, &cmd_a, 0u);
    for (idx = 0u; idx < COMMS_DEDUP_LEN; idx++)
    {
        comms_cmd_t other;

        (void)parse((uint8_t)(200u + idx), COMMS_CMD_START, 0, 0, NULL, &other);
        comms_dedup_add(&dedup, &other, 10u + idx);
    }
    CHECK("S1-H18", comms_dedup_check(&dedup, &cmd_a, 100u) == COMMS_DEDUP_NEW);
    CHECK("S1-H18", dedup.count == COMMS_DEDUP_LEN);
    /* STOP is always safe to repeat */
    CHECK("S1-H18", comms_cmd_repeat_safe(stop.cmd) && !comms_cmd_repeat_safe(cmd_a.cmd));
}

static void test_map(void)
{
    comms_cmd_t cmd;
    uint8_t     frame[8];
    uint16_t    count;
    uint32_t const afzw = (uint32_t)'A' | ((uint32_t)'F' << 8) | ((uint32_t)'Z' << 16) | ((uint32_t)'W' << 24);

    /* S1-H19: the barcode map arrives over MQTT (not hard-coded) */
    count = cmd_frame(frame, 5u, COMMS_CMD_MAP, 0, 0, "AFZW");
    CHECK("S1-H19", count == 6u && hex_eq(frame, count, GOLD_CMD_MAP));
    CHECK("S1-H19", comms_parse_cmd(frame, count, &cmd) == COMMS_ERR_NONE && cmd.cmd == COMMS_CMD_MAP &&
                    (uint32_t)cmd.value == afzw);
    CHECK("S1-H19", parse(5u, COMMS_CMD_MAP, 0, 0, "AAZW", &cmd) == COMMS_ERR_BAD_MAP);   /* used twice */
    frame[2] = (uint8_t)(frame[2] & 0x07u);             /* left symbol 0 (none) */
    frame[3] = (uint8_t)(frame[3] & 0xFEu);
    CHECK("S1-H19", comms_parse_cmd(frame, count, &cmd) == COMMS_ERR_BAD_MAP);
    count = cmd_frame(frame, 5u, COMMS_CMD_MAP, 0, 0, "AFZW");
    frame[2] = (uint8_t)(frame[2] | 0xF8u);             /* left symbol 63: not a symbol */
    frame[3] = (uint8_t)(frame[3] | 0x01u);
    CHECK("S1-H19", comms_parse_cmd(frame, count, &cmd) == COMMS_ERR_BAD_MAP);
    (void)parse(5u, COMMS_CMD_MAP, 0, 0, "AFZW", &cmd);
    CHECK("S1-H19", comms_check_cmd(&cmd, false) == COMMS_ERR_BUSY && comms_check_cmd(&cmd, true) == COMMS_ERR_NONE);
    CHECK("S1-H19", comms_map_char_ok('Z') && comms_map_char_ok('7') && !comms_map_char_ok('*') && !comms_map_char_ok('a'));
    CHECK("S1-H19", comms_symbol_index('0') == 1u && comms_symbol_index('%') == 43u && comms_symbol_index('*') == 0u &&
                    comms_symbol_char(43u) == '%' && comms_symbol_char(44u) == '\0');

#if APP_BENCH
    CHECK("S1-H20", parse(7u, COMMS_CMD_SET, param_index("bench_turn"), -900, NULL, &cmd) == COMMS_ERR_NONE &&
                    cmd.param_id == 102u && cmd.value == -900);
    CHECK("S1-H20", parse(7u, COMMS_CMD_SET, param_index("bench_turn"), 3601, NULL, &cmd) == COMMS_ERR_RANGE);
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
