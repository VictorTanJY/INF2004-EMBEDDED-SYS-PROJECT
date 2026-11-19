/*
 *  comms_proto.c
 *  Subsystem 1 -- MQTT message formats.  Owner: Buddy 1 (Victor).
 *
 *  Pure C, no kernel calls (see comms_proto.h for the frame layouts).
 *  Built into the firmware and into testing/subsystem1/host/ for the host
 *  unit tests.  Every field is written with app_pack.h: masks and shifts,
 *  only as many bits as its valid range needs (Change Requirement 1).
 */

#include <stddef.h>
#include "comms_proto.h"
#include "../app_pack.h"

typedef uint8_t
    comms_check_symbols_t[(PK_SYM_COUNT == COMMS_SYMBOL_COUNT) ? 1 : -1];

/* Whitelist for "set" (PROPOSED ranges; owners confirm, TEAM.md IF-11). */
static comms_proto_param_t const g_params[COMMS_PARAM_COUNT] = {
    {"speed_mm_s", 1u, 150, 350}, /* cruise speed, NFR11 (Buddy 2/3)       */
    {"slow_cm", 2u, 25, 60},      /* slow-down range, FR9, default 40 (B5) */
    {"stop_cm", 3u, 15, 40},      /* stop range, FR9/NFR6, default 20 (B5) */
    {"lost_mm", 4u, 55, 150},     /* line-lost length, FR2, default 80 (B3) */
    {"junction_mm", 5u, 10, 40}   /* junction length, FR2, default 12 (B3) */
#if APP_BENCH
    ,
    {"bench_move", 101u, -2000, 2000}, /* app_config.h: bring-up only */
    {"bench_turn", 102u, -3600, 3600},
    {"bench_speed", 103u, 50, 450}
#endif
};


#define NONE_D_CM   511u  /* telemetry d: nothing in range      */
#define NONE_W_MM   1023u /* telemetry w: no profile            */
#define NONE_CLR_CM 127u  /* telemetry clr_l / clr_r: no profile */
#define T10_BITS    24u   /* frame time, 10 ms units (46.6 h)   */

/* ---------------------------------------------------------------------
 *  Helpers
 * --------------------------------------------------------------------- */
uint8_t comms_proto_symbol_index (char chr)
{
    return app_pack_sym_index(chr);
}

char comms_proto_symbol_char (uint8_t index)
{
    return app_pack_sym_char(index);
}

static void put_header (app_pack_writer_t * p_w, comms_proto_frame_type_t type)
{
    app_pack_put(p_w, (uint32_t)type, 5u);
    app_pack_put(p_w, COMMS_FRAME_VERSION, 3u);
}

/* centi-degrees -> tenths of a degree, rounded half away from zero. */
static int32_t cdeg_to_tenths (int16_t cdeg)
{
    int32_t value = (int32_t)cdeg;

    return (value >= 0) ? ((value + 5) / 10) : -((5 - value) / 10);
}

/* mm -> cm, rounded; none_code if absent or too large for the field. */
static uint32_t mm_to_cm_field (uint16_t value_mm, bool b_valid,
                                uint32_t none_code)
{
    uint32_t value_cm;

    if (!b_valid || (COMMS_D_NONE == value_mm))
    {
        return none_code;
    }
    value_cm = ((uint32_t)value_mm + 5u) / 10u;
    return (value_cm >= none_code) ? (none_code - 1u) : value_cm;
}

/* ---------------------------------------------------------------------
 *  Telemetry record (mbf_telem)
 * --------------------------------------------------------------------- */
void comms_proto_rec_pack (comms_proto_rec_t * p_rec, uint8_t kind, uint8_t src,
                           uint16_t code, int32_t data, uint32_t now_ms)
{
    uint32_t       word     = 0u;
    uint32_t const kind_idx = ((kind >= 1u) && (kind <= 4u))
                                  ? (uint32_t)(kind - 1u)
                                  : 3u; /* unknown -> LOG */

    word =
        app_pack_set(word, COMMS_REC_KIND_SHIFT, COMMS_REC_KIND_BITS, kind_idx);
    word       = app_pack_set(word, COMMS_REC_SRC_SHIFT, COMMS_REC_SRC_BITS,
                              app_pack_sat_u(src, COMMS_REC_SRC_BITS));
    word       = app_pack_set(word, COMMS_REC_CODE_SHIFT, COMMS_REC_CODE_BITS,
                              app_pack_sat_u(code, COMMS_REC_CODE_BITS));
    word       = app_pack_set(word, COMMS_REC_T_SHIFT, COMMS_REC_T_BITS,
                              (now_ms / 10u) & app_pack_mask(COMMS_REC_T_BITS));
    p_rec->hdr = word;
    p_rec->data = data;
}

void comms_proto_rec_unpack (comms_proto_rec_t const * p_rec, uint32_t now_ms,
                             comms_proto_event_t * p_ev)
{
    uint32_t const now10 = now_ms / 10u;
    uint32_t const t10 =
        app_pack_get(p_rec->hdr, COMMS_REC_T_SHIFT, COMMS_REC_T_BITS);
    uint32_t const age10 = (now10 - t10) & app_pack_mask(COMMS_REC_T_BITS);

    p_ev->kind = (uint8_t)(app_pack_get(p_rec->hdr, COMMS_REC_KIND_SHIFT,
                                        COMMS_REC_KIND_BITS)
                           + 1u);
    p_ev->src  = (uint8_t)app_pack_get(p_rec->hdr, COMMS_REC_SRC_SHIFT,
                                       COMMS_REC_SRC_BITS);
    p_ev->code = (uint16_t)app_pack_get(p_rec->hdr, COMMS_REC_CODE_SHIFT,
                                        COMMS_REC_CODE_BITS);
    p_ev->data = p_rec->data;
    p_ev->t_ms = (now10 - age10) * 10u;
}

/* ---------------------------------------------------------------------
 *  Builders
 * --------------------------------------------------------------------- */
uint16_t comms_proto_build_telemetry (comms_proto_telem_t const * p_t,
                                      uint8_t * p_buf, uint16_t cap)
{
    app_pack_writer_t writer;
    bool const        b_prof = p_t->b_obst_valid;

    app_pack_writer_init(&writer, p_buf, cap);
    put_header(&writer, COMMS_FT_TELEMETRY);
    app_pack_put(&writer, p_t->seq, 8u);
    app_pack_put(&writer, p_t->t_ms / 10u, T10_BITS);
    app_pack_put(&writer, app_pack_sat_u(p_t->state, COMMS_STATE_BITS),
                 COMMS_STATE_BITS);
    app_pack_put(&writer, app_pack_off_s(p_t->v_l_mm_s, 10u), 10u);
    app_pack_put(&writer, app_pack_off_s(p_t->v_r_mm_s, 10u), 10u);
    app_pack_put(&writer, (uint32_t)p_t->enc_l, 16u);
    app_pack_put(&writer, (uint32_t)p_t->enc_r, 16u);
    app_pack_put(&writer, (uint32_t)p_t->dist_mm, 16u);
    app_pack_put(&writer, app_pack_off_s(p_t->line_err, 11u), 11u);
    app_pack_put(&writer, p_t->b_line_lost ? 1u : 0u, 1u);
    app_pack_put(&writer, app_pack_sat_u(p_t->line_bc, 10u), 10u);
    app_pack_put(&writer, comms_proto_symbol_index(p_t->barcode), 6u);
    app_pack_put(&writer, comms_proto_symbol_index(p_t->nav), 6u);
    app_pack_put(&writer, app_pack_off_s(cdeg_to_tenths(p_t->pitch_cdeg), 11u),
                 11u);
    app_pack_put(&writer, app_pack_off_s(cdeg_to_tenths(p_t->roll_cdeg), 12u),
                 12u);
    app_pack_put(&writer, app_pack_sat_u(p_t->hump_max_mm, 8u), 8u);
    app_pack_put(&writer, mm_to_cm_field(p_t->obst_d_mm, true, NONE_D_CM), 9u);
    app_pack_put(&writer,
                 b_prof ? ((p_t->obst_w_mm >= NONE_W_MM) ? (NONE_W_MM - 1u)
                                                         : p_t->obst_w_mm)
                        : NONE_W_MM,
                 10u);
    app_pack_put(&writer, mm_to_cm_field(p_t->clr_l_mm, b_prof, NONE_CLR_CM),
                 7u);
    app_pack_put(&writer, mm_to_cm_field(p_t->clr_r_mm, b_prof, NONE_CLR_CM),
                 7u);
    return app_pack_writer_len(&writer);
}

uint16_t comms_proto_build_event (comms_proto_event_t const * p_ev,
                                  uint32_t seq, uint8_t * p_buf, uint16_t cap)
{
    app_pack_writer_t writer;
    int32_t const     data = p_ev->data;
    uint8_t           dlen;
    uint8_t           width;

    if (0 == data)
    {
        dlen  = 0u;
        width = 0u;
    }
    else if ((data >= -128) && (data <= 127))
    {
        dlen  = 1u;
        width = 8u;
    }
    else if ((data >= -32768) && (data <= 32767))
    {
        dlen  = 2u;
        width = 16u;
    }
    else
    {
        dlen  = 3u;
        width = 32u;
    }

    app_pack_writer_init(&writer, p_buf, cap);
    put_header(&writer, COMMS_FT_EVENT);
    app_pack_put(&writer, seq, 8u);
    app_pack_put(&writer, p_ev->t_ms / 10u, T10_BITS);
    app_pack_put(&writer,
                 ((p_ev->kind >= 1u) && (p_ev->kind <= 4u))
                     ? (uint32_t)(p_ev->kind - 1u)
                     : 3u,
                 2u);
    app_pack_put(&writer, app_pack_sat_u(p_ev->src, COMMS_SRC_BITS),
                 COMMS_SRC_BITS);
    app_pack_put(&writer, app_pack_sat_u(p_ev->code, COMMS_REC_CODE_BITS),
                 COMMS_REC_CODE_BITS);
    app_pack_put(&writer, dlen, 2u);
    if (width > 0u)
    {
        app_pack_put(&writer, (uint32_t)data,
                     width); /* two's complement, low bits */
    }
    return app_pack_writer_len(&writer);
}

uint16_t comms_proto_build_status (uint32_t uptime_s, int32_t rssi,
                                   uint32_t drops, char const * p_map,
                                   uint8_t * p_buf, uint16_t cap)
{
    app_pack_writer_t writer;
    uint32_t          neg_rssi = (rssi < 0) ? (uint32_t)(-(int64_t)rssi) : 0u;
    bool const        b_map    = (p_map != NULL) && (p_map[0] != '\0');
    uint8_t           kdx;

    app_pack_writer_init(&writer, p_buf, cap);
    put_header(&writer, COMMS_FT_STATUS);
    app_pack_put(&writer, app_pack_sat_u(uptime_s, 17u), 17u);
    app_pack_put(&writer, app_pack_sat_u(neg_rssi, 7u), 7u);
    app_pack_put(&writer, app_pack_sat_u(drops, 8u), 8u);
    app_pack_put(&writer, b_map ? 1u : 0u, 1u);
    if (b_map)
    {
        for (kdx = 0u; kdx < 4u; kdx++)
        {
            app_pack_put(&writer, comms_proto_symbol_index(p_map[kdx]), 6u);
        }
    }
    return app_pack_writer_len(&writer);
}

uint16_t comms_proto_build_ack (comms_proto_cmd_t const * p_cmd,
                                comms_proto_err_t err, uint8_t * p_buf,
                                uint16_t cap)
{
    app_pack_writer_t writer;

    app_pack_writer_init(&writer, p_buf, cap);
    put_header(&writer, COMMS_FT_ACK);
    app_pack_put(&writer, p_cmd->b_has_id ? 1u : 0u, 1u);
    app_pack_put(&writer, p_cmd->b_has_id ? p_cmd->cmd_id : 0u, COMMS_ID_BITS);
    app_pack_put(&writer, app_pack_sat_u((uint32_t)err, COMMS_ERR_BITS),
                 COMMS_ERR_BITS);
    return app_pack_writer_len(&writer);
}

/* ---------------------------------------------------------------------
 *  Command parser
 * --------------------------------------------------------------------- */
comms_proto_param_t const * comms_proto_param_by_index (uint8_t index)
{
    return (index < COMMS_PARAM_COUNT) ? &g_params[index] : NULL;
}

comms_proto_param_t const * comms_proto_param_by_id (uint8_t param_id)
{
    uint8_t idx;

    for (idx = 0u; idx < COMMS_PARAM_COUNT; idx++)
    {
        if (g_params[idx].param_id == param_id)
        {
            return &g_params[idx];
        }
    }
    return NULL;
}

comms_proto_err_t comms_proto_parse_cmd (uint8_t const * p_data, uint16_t len,
                                         comms_proto_cmd_t * p_cmd)
{
    app_pack_reader_t reader;
    uint32_t          type;
    uint32_t          version;

    p_cmd->cmd_id   = 0u;
    p_cmd->b_has_id = false;
    p_cmd->cmd      = (uint8_t)COMMS_CMD_NONE;
    p_cmd->param_id = 0u;
    p_cmd->value    = 0;

    app_pack_reader_init(&reader, p_data, len);
    type    = app_pack_take(&reader, 5u);
    version = app_pack_take(&reader, 3u);
    if (reader.b_short || (type != (uint32_t)COMMS_FT_CMD)
        || (version != COMMS_FRAME_VERSION))
    {
        return COMMS_ERR_BAD_FRAME;
    }
    p_cmd->cmd_id = (uint8_t)app_pack_take(&reader, COMMS_ID_BITS);
    if (reader.b_short)
    {
        return COMMS_ERR_NO_ID;
    }
    p_cmd->b_has_id = true;
    p_cmd->cmd      = (uint8_t)app_pack_take(&reader, COMMS_CMD_BITS);
    if (reader.b_short)
    {
        return COMMS_ERR_BAD_FRAME;
    }

    switch (p_cmd->cmd)
    {
        case COMMS_CMD_START:
        case COMMS_CMD_STOP:
        case COMMS_CMD_RESET:
            if (len != 3u)
            {
                return COMMS_ERR_BAD_FRAME;
            }
        break;

        case COMMS_CMD_SET:
        {
            comms_proto_param_t const * p_param;
            uint32_t                    index;

            if (len != 5u)
            {
                return COMMS_ERR_BAD_FRAME;
            }
            index        = app_pack_take(&reader, COMMS_PARAM_BITS);
            p_cmd->value = app_pack_unoff_s(
                app_pack_take(&reader, COMMS_VALUE_BITS), COMMS_VALUE_BITS);
            p_param = comms_proto_param_by_index((uint8_t)index);
            if (NULL == p_param)
            {
                return COMMS_ERR_BAD_PARAM;
            }
            p_cmd->param_id = p_param->param_id;
            if ((p_cmd->value < p_param->min) || (p_cmd->value > p_param->max))
            {
                return COMMS_ERR_RANGE;
            }
        }
        break;

        case COMMS_CMD_MAP:
        {
            char     chars[4];
            uint32_t packed = 0u;
            uint8_t  kdx;
            uint8_t  jdx;

            if (len != 6u)
            {
                return COMMS_ERR_BAD_FRAME;
            }
            for (kdx = 0u; kdx < 4u; kdx++)
            {
                chars[kdx] = comms_proto_symbol_char(
                    (uint8_t)app_pack_take(&reader, 6u));
                if ('\0' == chars[kdx])
                {
                    return COMMS_ERR_BAD_MAP;
                }
                for (jdx = 0u; jdx < kdx; jdx++)
                {
                    if (chars[jdx] == chars[kdx])
                    {
                        return COMMS_ERR_BAD_MAP;
                    }
                }
                packed |= ((uint32_t)(uint8_t)chars[kdx]) << (8u * kdx);
            }
            p_cmd->value = (int32_t)packed;
        }
        break;

        default:
            return COMMS_ERR_BAD_CMD;
    }
    return COMMS_ERR_NONE;
}

bool comms_proto_map_char_ok (char chr)
{
    return (app_pack_sym_index(chr) != 0u);
}

comms_proto_err_t comms_proto_check_ready (comms_proto_cmd_t const * p_cmd,
                                           bool b_calibrated)
{
    if (((uint8_t)COMMS_CMD_START == p_cmd->cmd) && !b_calibrated)
    {
        return COMMS_ERR_NOT_CAL;
    }
    return COMMS_ERR_NONE;
}

comms_proto_err_t comms_proto_check_cmd (comms_proto_cmd_t const * p_cmd,
                                         bool                      b_idle)
{
    if ((((uint8_t)COMMS_CMD_START == p_cmd->cmd)
         || ((uint8_t)COMMS_CMD_SET == p_cmd->cmd)
         || ((uint8_t)COMMS_CMD_MAP == p_cmd->cmd))
        && !b_idle)
    {
        return COMMS_ERR_BUSY;
    }
    return COMMS_ERR_NONE;
}

/* ---------------------------------------------------------------------
 *  Duplicate suppression
 * --------------------------------------------------------------------- */
void comms_proto_dedup_init (comms_proto_dedup_t * p_dd)
{
    p_dd->next  = 0u;
    p_dd->count = 0u;
}

bool comms_proto_cmd_repeat_safe (uint8_t cmd)
{
    return ((uint8_t)COMMS_CMD_STOP == cmd);
}

comms_proto_dedup_result_t
comms_proto_dedup_check (comms_proto_dedup_t const * p_dd,
                         comms_proto_cmd_t const * p_cmd, uint32_t now_ms)
{
    uint8_t idx;

    for (idx = 0u; idx < p_dd->count; idx++)
    {
        comms_proto_dedup_entry_t const * p_e = &p_dd->entries[idx];

        /* Unsigned difference: correct across the 32-bit ms wrap. */
        if ((p_e->cmd_id == p_cmd->cmd_id)
            && ((uint32_t)(now_ms - p_e->t_ms) < COMMS_DEDUP_WINDOW_MS))
        {
            if ((p_e->cmd == p_cmd->cmd) && (p_e->param_id == p_cmd->param_id)
                && (p_e->value == p_cmd->value))
            {
                return COMMS_DEDUP_REPEAT;
            }
            return COMMS_DEDUP_CONFLICT;
        }
    }
    return COMMS_DEDUP_NEW;
}

void comms_proto_dedup_add (comms_proto_dedup_t *     p_dd,
                            comms_proto_cmd_t const * p_cmd, uint32_t now_ms)
{
    comms_proto_dedup_entry_t * p_e = &p_dd->entries[p_dd->next];

    p_e->cmd_id   = p_cmd->cmd_id;
    p_e->t_ms     = now_ms;
    p_e->cmd      = p_cmd->cmd;
    p_e->param_id = p_cmd->param_id;
    p_e->value    = p_cmd->value;
    p_dd->next    = (uint8_t)((p_dd->next + 1u) % COMMS_DEDUP_LEN);
    if (p_dd->count < COMMS_DEDUP_LEN)
    {
        p_dd->count++;
    }
}

/* ---------------------------------------------------------------------
 *  Offline event ring
 * --------------------------------------------------------------------- */
void comms_proto_ring_init (comms_proto_ring_t * p_ring)
{
    p_ring->head  = 0u;
    p_ring->count = 0u;
}

bool comms_proto_ring_push (comms_proto_ring_t *      p_ring,
                            comms_proto_rec_t const * p_rec, uint8_t seq)
{
    bool    b_overwrote = false;
    uint8_t idx;

    if (COMMS_RING_LEN == p_ring->count)
    {
        p_ring->head =
            (uint8_t)((p_ring->head + 1u) % COMMS_RING_LEN); /* drop oldest */
        p_ring->count--;
        b_overwrote = true;
    }
    idx = (uint8_t)((p_ring->head + p_ring->count) % COMMS_RING_LEN);
    p_ring->rec[idx] = *p_rec;
    p_ring->seq[idx] = seq;
    p_ring->count++;
    return b_overwrote;
}

bool comms_proto_ring_peek (comms_proto_ring_t const * p_ring,
                            comms_proto_rec_t * p_rec, uint8_t * p_seq)
{
    if (0u == p_ring->count)
    {
        return false;
    }
    *p_rec = p_ring->rec[p_ring->head];
    *p_seq = p_ring->seq[p_ring->head];
    return true;
}

void comms_proto_ring_pop (comms_proto_ring_t * p_ring)
{
    if (p_ring->count > 0u)
    {
        p_ring->head = (uint8_t)((p_ring->head + 1u) % COMMS_RING_LEN);
        p_ring->count--;
    }
}

/*** end of file ***/
