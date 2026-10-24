/*
 *  comms_proto.c
 *  Subsystem 1 -- MQTT message formats.  Owner: Buddy 1 (Victor).
 *
 *  Pure C, no kernel calls (see comms_proto.h).  Built into the firmware
 *  and into testing/subsystem1/host/ for the host unit tests.
 */

#include <stddef.h>
#include "comms_proto.h"

/* ---------------------------------------------------------------------
 *  Name tables.  Index = mission_state_t / mission_ev_t (subsystem5.h);
 *  comms_service.c checks the counts at compile time.
 * --------------------------------------------------------------------- */
static char const * const k_state_names[COMMS_STATE_NAME_COUNT] =
{
    "BOOT", "IDLE", "CALIBRATING", "LINE_FOLLOW", "EXECUTE_NAV",
    "OBSTACLE_HANDLING", "AVOIDING", "LINE_RECOVERY", "LINE_SEARCH",
    "MISSION_COMPLETE", "FAULT"
};

static char const * const k_event_names[COMMS_EVENT_NAME_COUNT] =
{
    "NONE", "BTN_START_STOP", "BTN_CALIBRATE", "BTN_RESET",
    "REMOTE_START", "REMOTE_STOP", "REMOTE_RESET", "REMOTE_SET",
    "BARCODE", "JUNCTION", "LINE_LOST", "LINE_FOUND",
    "MOTION_DONE", "MOTION_FAILED", "OBSTACLE", "SLOW_DOWN",
    "PROFILE_DONE", "CONTINUE", "BYPASS_DONE", "RECOVERY_DONE",
    "RECOVERY_FAILED", "BLOCKED_TIMEOUT", "HUMP_START", "HUMP_END",
    "MOTION_CLASS", "FAULT", "CAL_DONE", "REMOTE_MAP", "SAFETY_OK"
};

#define COMMS_EV_BARCODE    8u      /* MISSION_EV_BARCODE */

static char const * const k_err_names[COMMS_ERR_COUNT] =
{
    "", "bad_json", "no_id", "bad_cmd", "bad_param", "range", "busy", "full", "id_reused",
    "not_calibrated", "bad_map"
};

static char const k_map_chars[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ-. $/+%";

/* Whitelist for "set" (PROPOSED ranges; owners confirm, TEAM.md IF-11). */
static comms_param_t const k_params[COMMS_PARAM_COUNT] =
{
    { "speed_mm_s",  1u, 150, 350 },  /* cruise speed, NFR11 (Buddy 2/3)       */
    { "slow_cm",     2u,  25,  60 },  /* slow-down range, FR9, default 40 (B5) */
    { "stop_cm",     3u,  15,  40 },  /* stop range, FR9/NFR6, default 20 (B5) */
    { "lost_mm",     4u,  55, 150 },  /* line-lost length, FR2, default 80 (B3) */
    { "junction_mm", 5u,  10,  40 }   /* junction length, FR2, default 12 (B3) */
#if APP_BENCH
    ,
    { "bench_move",  101u, -2000, 2000 },     /* app_config.h: bring-up only */
    { "bench_turn",  102u, -3600, 3600 },
    { "bench_speed", 103u,    50,  450 },
    { "bench_servo", 104u,     0,  180 }
#endif
};

char const * comms_state_name(uint8_t state)
{
    return (state < COMMS_STATE_NAME_COUNT) ? k_state_names[state] : "UNKNOWN";
}

char const * comms_event_name(uint16_t event)
{
    return (event < COMMS_EVENT_NAME_COUNT) ? k_event_names[event] : "UNKNOWN";
}

char const * comms_err_name(comms_err_t err)
{
    return ((uint32_t)err < (uint32_t)COMMS_ERR_COUNT) ? k_err_names[err] : "unknown";
}

/* ---------------------------------------------------------------------
 *  JSON writer
 * --------------------------------------------------------------------- */
static void jw_putc(jw_t * p_jw, char c)
{
    /* Keep one byte for the terminator written by jw_finish(). */
    if ((uint32_t)p_jw->len + 1u < (uint32_t)p_jw->cap)
    {
        p_jw->p_buf[p_jw->len] = c;
        p_jw->len++;
    }
    else
    {
        p_jw->b_overflow = true;
    }
}

void jw_init(jw_t * p_jw, char * p_buf, uint16_t cap)
{
    p_jw->p_buf      = p_buf;
    p_jw->cap        = cap;
    p_jw->len        = 0u;
    p_jw->b_overflow = (cap == 0u);
}

void jw_raw(jw_t * p_jw, char const * p_text)
{
    while (*p_text != '\0')
    {
        jw_putc(p_jw, *p_text);
        p_text++;
    }
}

void jw_uint(jw_t * p_jw, uint32_t value)
{
    char    digits[10];
    uint8_t n = 0u;

    do
    {
        digits[n] = (char)('0' + (char)(value % 10u));
        n++;
        value /= 10u;
    } while (value != 0u);

    while (n > 0u)
    {
        n--;
        jw_putc(p_jw, digits[n]);
    }
}

void jw_int(jw_t * p_jw, int32_t value)
{
    if (value < 0)
    {
        jw_putc(p_jw, '-');
        jw_uint(p_jw, (uint32_t)0u - (uint32_t)value);   /* safe for INT32_MIN */
    }
    else
    {
        jw_uint(p_jw, (uint32_t)value);
    }
}

void jw_fixed1(jw_t * p_jw, int32_t tenths)
{
    uint32_t mag;

    if (tenths < 0)
    {
        jw_putc(p_jw, '-');
        mag = (uint32_t)0u - (uint32_t)tenths;
    }
    else
    {
        mag = (uint32_t)tenths;
    }
    jw_uint(p_jw, mag / 10u);
    jw_putc(p_jw, '.');
    jw_putc(p_jw, (char)('0' + (char)(mag % 10u)));
}

void jw_str(jw_t * p_jw, char const * p_text)
{
    static char const k_hex[] = "0123456789abcdef";

    jw_putc(p_jw, '"');
    while (*p_text != '\0')
    {
        uint8_t c = (uint8_t)*p_text;

        if ((c == (uint8_t)'"') || (c == (uint8_t)'\\'))
        {
            jw_putc(p_jw, '\\');
            jw_putc(p_jw, (char)c);
        }
        else if (c < 0x20u)
        {
            jw_raw(p_jw, "\\u00");
            jw_putc(p_jw, k_hex[c >> 4]);
            jw_putc(p_jw, k_hex[c & 0x0Fu]);
        }
        else
        {
            jw_putc(p_jw, (char)c);
        }
        p_text++;
    }
    jw_putc(p_jw, '"');
}

uint16_t jw_finish(jw_t * p_jw)
{
    if (p_jw->b_overflow)
    {
        if (p_jw->cap > 0u)
        {
            p_jw->p_buf[0] = '\0';
        }
        return 0u;
    }
    p_jw->p_buf[p_jw->len] = '\0';
    return p_jw->len;
}

static void jw_bool(jw_t * p_jw, bool b_value)
{
    jw_raw(p_jw, b_value ? "true" : "false");
}

/* mm -> cm rounded, or null. */
static void jw_cm_or_null(jw_t * p_jw, uint16_t mm, bool b_valid)
{
    if (b_valid && (mm != COMMS_D_NONE))
    {
        jw_uint(p_jw, ((uint32_t)mm + 5u) / 10u);
    }
    else
    {
        jw_raw(p_jw, "null");
    }
}

static int32_t cdeg_to_tenths(int16_t cdeg)
{
    int32_t c = (int32_t)cdeg;

    return (c >= 0) ? ((c + 5) / 10) : -((5 - c) / 10);
}

/* ---------------------------------------------------------------------
 *  Builders
 * --------------------------------------------------------------------- */
uint16_t comms_build_telemetry(comms_telem_t const * p_t, char * p_buf, uint16_t cap)
{
    jw_t jw;
    char code[2];

    jw_init(&jw, p_buf, cap);
    jw_raw(&jw, "{\"seq\":");          jw_uint(&jw, p_t->seq);
    jw_raw(&jw, ",\"t_ms\":");         jw_uint(&jw, p_t->t_ms);
    jw_raw(&jw, ",\"state\":");        jw_str(&jw, comms_state_name(p_t->state));
    jw_raw(&jw, ",\"v_mm_s\":[");      jw_int(&jw, p_t->v_l_mm_s);
    jw_raw(&jw, ",");                  jw_int(&jw, p_t->v_r_mm_s);
    jw_raw(&jw, "],\"enc\":[");        jw_int(&jw, p_t->enc_l);
    jw_raw(&jw, ",");                  jw_int(&jw, p_t->enc_r);
    jw_raw(&jw, "],\"dist_mm\":");     jw_int(&jw, p_t->dist_mm);
    jw_raw(&jw, ",\"line\":{\"err\":"); jw_int(&jw, p_t->line_err);
    jw_raw(&jw, ",\"lost\":");         jw_bool(&jw, p_t->b_line_lost);
    jw_raw(&jw, "},\"barcode\":");
    if (p_t->barcode != '\0')
    {
        code[0] = p_t->barcode;
        code[1] = '\0';
        jw_str(&jw, code);
    }
    else
    {
        jw_raw(&jw, "null");
    }
    jw_raw(&jw, ",\"pitch_deg\":");    jw_fixed1(&jw, cdeg_to_tenths(p_t->pitch_cdeg));
    jw_raw(&jw, ",\"hump_max_mm\":");  jw_uint(&jw, p_t->hump_max_mm);
    jw_raw(&jw, ",\"obstacle\":{\"d_cm\":");
    jw_cm_or_null(&jw, p_t->obst_d_mm, true);
    jw_raw(&jw, ",\"w_cm\":");         jw_cm_or_null(&jw, p_t->obst_w_mm, p_t->b_obst_valid);
    jw_raw(&jw, ",\"w_mm\":");
    if (p_t->b_obst_valid)
    {
        jw_uint(&jw, p_t->obst_w_mm);   /* width to the mm: graded on accuracy */
    }
    else
    {
        jw_raw(&jw, "null");
    }
    jw_raw(&jw, ",\"clr_l_cm\":");     jw_cm_or_null(&jw, p_t->clr_l_mm, p_t->b_obst_valid);
    jw_raw(&jw, ",\"clr_r_cm\":");     jw_cm_or_null(&jw, p_t->clr_r_mm, p_t->b_obst_valid);
    jw_raw(&jw, "},\"nav\":");
    if (p_t->nav != '\0')
    {
        code[0] = p_t->nav;
        code[1] = '\0';
        jw_str(&jw, code);
    }
    else
    {
        jw_raw(&jw, "null");
    }
    jw_raw(&jw, "}");
    return jw_finish(&jw);
}

uint16_t comms_build_event(comms_rec_t const * p_rec, uint32_t seq, char * p_buf, uint16_t cap)
{
    jw_t jw;
    char code[2];

    jw_init(&jw, p_buf, cap);
    jw_raw(&jw, "{\"seq\":");   jw_uint(&jw, seq);
    jw_raw(&jw, ",\"t_ms\":");  jw_uint(&jw, p_rec->t_ms);
    jw_raw(&jw, ",\"type\":");

    switch (p_rec->kind)
    {
        case COMMS_KIND_EVENT:
            jw_str(&jw, comms_event_name(p_rec->code));
            jw_raw(&jw, ",\"data\":");
            if ((p_rec->code == COMMS_EV_BARCODE) &&
                (p_rec->data >= 0x20) && (p_rec->data < 0x7F))
            {
                code[0] = (char)p_rec->data;
                code[1] = '\0';
                jw_str(&jw, code);
            }
            else
            {
                jw_int(&jw, p_rec->data);
            }
            break;

        case COMMS_KIND_FAULT:
            jw_str(&jw, "FAULT");
            jw_raw(&jw, ",\"data\":");
            jw_int(&jw, p_rec->data);
            break;

        case COMMS_KIND_STATE:
            jw_str(&jw, "STATE");
            jw_raw(&jw, ",\"data\":");
            jw_str(&jw, comms_state_name((uint8_t)p_rec->code));
            break;

        default:    /* COMMS_KIND_LOG and anything unknown */
            jw_str(&jw, "LOG");
            jw_raw(&jw, ",\"data\":");
            jw_int(&jw, p_rec->data);
            jw_raw(&jw, ",\"code\":");
            jw_uint(&jw, p_rec->code);
            break;
    }
    jw_raw(&jw, ",\"src\":");
    jw_uint(&jw, p_rec->src);
    jw_raw(&jw, "}");
    return jw_finish(&jw);
}

uint16_t comms_build_status(bool b_online, uint32_t uptime_s, int32_t rssi,
                            uint32_t drops, char const * p_map, char * p_buf, uint16_t cap)
{
    jw_t jw;

    jw_init(&jw, p_buf, cap);
    jw_raw(&jw, "{\"online\":");     jw_bool(&jw, b_online);
    jw_raw(&jw, ",\"uptime_s\":");   jw_uint(&jw, uptime_s);
    jw_raw(&jw, ",\"rssi\":");       jw_int(&jw, rssi);
    jw_raw(&jw, ",\"drops\":");      jw_uint(&jw, drops);
    jw_raw(&jw, ",\"map\":");
    if ((p_map != NULL) && (p_map[0] != '\0'))
    {
        static char const * const k_keys[4] = { "left", "right", "straight", "uturn" };
        char    one[2];
        uint8_t k;

        jw_raw(&jw, "{");
        for (k = 0u; k < 4u; k++)
        {
            one[0] = p_map[k];
            one[1] = '\0';
            if (k > 0u)
            {
                jw_raw(&jw, ",");
            }
            jw_str(&jw, k_keys[k]);
            jw_raw(&jw, ":");
            jw_str(&jw, one);
        }
        jw_raw(&jw, "}");
    }
    else
    {
        jw_raw(&jw, "null");            /* no map yet: barcodes are not acted on */
    }
    jw_raw(&jw, "}");
    return jw_finish(&jw);
}

uint16_t comms_build_ack(comms_cmd_t const * p_cmd, comms_err_t err,
                         char * p_buf, uint16_t cap)
{
    jw_t jw;

    jw_init(&jw, p_buf, cap);
    jw_raw(&jw, "{\"id\":");
    if (p_cmd->b_has_id)
    {
        jw_uint(&jw, p_cmd->id);
    }
    else
    {
        jw_raw(&jw, "null");
    }
    jw_raw(&jw, ",\"ok\":");
    jw_bool(&jw, err == COMMS_ERR_NONE);
    if (err != COMMS_ERR_NONE)
    {
        jw_raw(&jw, ",\"err\":");
        jw_str(&jw, comms_err_name(err));
    }
    jw_raw(&jw, "}");
    return jw_finish(&jw);
}

/* ---------------------------------------------------------------------
 *  Command parser: one flat JSON object of string / integer / true /
 *  false / null values.  Nested objects and arrays are rejected.
 * --------------------------------------------------------------------- */
typedef struct
{
    uint8_t const * p;
    uint16_t        len;
    uint16_t        pos;
} rd_t;

typedef enum
{
    VAL_NONE = 0,
    VAL_STR,
    VAL_INT,
    VAL_OTHER           /* non-integer number, true, false, null */
} val_kind_t;

typedef struct
{
    val_kind_t kind;
    uint16_t   start;   /* VAL_STR: content offset */
    uint16_t   n;       /* VAL_STR: content length */
    int32_t    value;   /* VAL_INT                 */
} val_t;

static bool rd_more(rd_t const * p_rd)
{
    return (p_rd->pos < p_rd->len);
}

static uint8_t rd_peek(rd_t const * p_rd)
{
    return rd_more(p_rd) ? p_rd->p[p_rd->pos] : 0u;
}

static void rd_skip_ws(rd_t * p_rd)
{
    while (rd_more(p_rd))
    {
        uint8_t c = p_rd->p[p_rd->pos];

        if ((c != (uint8_t)' ') && (c != (uint8_t)'\t') &&
            (c != (uint8_t)'\r') && (c != (uint8_t)'\n'))
        {
            break;
        }
        p_rd->pos++;
    }
}

/* At the opening quote.  Escapes are skipped over, not decoded. */
static bool rd_string(rd_t * p_rd, uint16_t * p_start, uint16_t * p_n)
{
    p_rd->pos++;                            /* opening quote */
    *p_start = p_rd->pos;
    while (rd_more(p_rd))
    {
        uint8_t c = p_rd->p[p_rd->pos];

        if (c == (uint8_t)'"')
        {
            *p_n = (uint16_t)(p_rd->pos - *p_start);
            p_rd->pos++;
            return true;
        }
        if (c == (uint8_t)'\\')
        {
            p_rd->pos++;                    /* skip the escaped byte too */
        }
        else if (c < 0x20u)
        {
            return false;                   /* raw control char */
        }
        p_rd->pos++;
    }
    return false;                           /* unterminated */
}

static bool rd_number(rd_t * p_rd, val_t * p_val)
{
    bool     b_neg    = false;
    bool     b_digits = false;
    bool     b_int    = true;
    uint32_t mag      = 0u;
    uint32_t const lim = 0x7FFFFFFFuL;

    if (rd_peek(p_rd) == (uint8_t)'-')
    {
        b_neg = true;
        p_rd->pos++;
    }
    while (rd_more(p_rd) && (rd_peek(p_rd) >= (uint8_t)'0') && (rd_peek(p_rd) <= (uint8_t)'9'))
    {
        uint32_t d = (uint32_t)(rd_peek(p_rd) - (uint8_t)'0');

        b_digits = true;
        if (mag > ((lim - d) / 10u))
        {
            b_int = false;                  /* does not fit an int32 */
        }
        else
        {
            mag = (mag * 10u) + d;
        }
        p_rd->pos++;
    }
    if (!b_digits)
    {
        return false;
    }
    /* Fraction or exponent: valid JSON, but not an integer. */
    while (rd_more(p_rd))
    {
        uint8_t c = rd_peek(p_rd);

        if (((c >= (uint8_t)'0') && (c <= (uint8_t)'9')) || (c == (uint8_t)'.') ||
            (c == (uint8_t)'e') || (c == (uint8_t)'E') || (c == (uint8_t)'+') ||
            (c == (uint8_t)'-'))
        {
            b_int = false;
            p_rd->pos++;
        }
        else
        {
            break;
        }
    }
    if (b_int)
    {
        p_val->kind  = VAL_INT;
        p_val->value = b_neg ? -(int32_t)mag : (int32_t)mag;
    }
    else
    {
        p_val->kind = VAL_OTHER;
    }
    return true;
}

static bool rd_literal(rd_t * p_rd, char const * p_word)
{
    while (*p_word != '\0')
    {
        if (rd_peek(p_rd) != (uint8_t)*p_word)
        {
            return false;
        }
        p_rd->pos++;
        p_word++;
    }
    return true;
}

static bool rd_value(rd_t * p_rd, val_t * p_val)
{
    uint8_t c = rd_peek(p_rd);

    p_val->kind = VAL_NONE;
    if (c == (uint8_t)'"')
    {
        p_val->kind = VAL_STR;
        return rd_string(p_rd, &p_val->start, &p_val->n);
    }
    if ((c == (uint8_t)'-') || ((c >= (uint8_t)'0') && (c <= (uint8_t)'9')))
    {
        return rd_number(p_rd, p_val);
    }
    p_val->kind = VAL_OTHER;
    if (c == (uint8_t)'t')
    {
        return rd_literal(p_rd, "true");
    }
    if (c == (uint8_t)'f')
    {
        return rd_literal(p_rd, "false");
    }
    if (c == (uint8_t)'n')
    {
        return rd_literal(p_rd, "null");
    }
    return false;                           /* '{', '[' or junk */
}

static bool str_eq(uint8_t const * p_s, uint16_t n, char const * p_lit)
{
    uint16_t i;

    for (i = 0u; i < n; i++)
    {
        if ((p_lit[i] == '\0') || (p_s[i] != (uint8_t)p_lit[i]))
        {
            return false;
        }
    }
    return (p_lit[n] == '\0');
}

comms_param_t const * comms_param_by_name(char const * p_name, uint16_t name_len)
{
    uint8_t i;

    for (i = 0u; i < COMMS_PARAM_COUNT; i++)
    {
        if (str_eq((uint8_t const *)p_name, name_len, k_params[i].p_name))
        {
            return &k_params[i];
        }
    }
    return NULL;
}

comms_param_t const * comms_param_by_id(uint8_t id)
{
    uint8_t i;

    for (i = 0u; i < COMMS_PARAM_COUNT; i++)
    {
        if (k_params[i].id == id)
        {
            return &k_params[i];
        }
    }
    return NULL;
}

comms_err_t comms_parse_cmd(uint8_t const * p_data, uint16_t len, comms_cmd_t * p_cmd)
{
    rd_t    rd;
    val_t   cmd_val   = { VAL_NONE, 0u, 0u, 0 };
    val_t   param_val = { VAL_NONE, 0u, 0u, 0 };
    val_t   value_val = { VAL_NONE, 0u, 0u, 0 };
    val_t   map_val[4] = { { VAL_NONE, 0u, 0u, 0 }, { VAL_NONE, 0u, 0u, 0 },
                           { VAL_NONE, 0u, 0u, 0 }, { VAL_NONE, 0u, 0u, 0 } };
    bool    b_done    = false;

    p_cmd->id       = 0u;
    p_cmd->b_has_id = false;
    p_cmd->cmd      = (uint8_t)COMMS_CMD_NONE;
    p_cmd->param_id = 0u;
    p_cmd->value    = 0;

    rd.p   = p_data;
    rd.len = len;
    rd.pos = 0u;

    rd_skip_ws(&rd);
    if (rd_peek(&rd) != (uint8_t)'{')
    {
        return COMMS_ERR_BAD_JSON;
    }
    rd.pos++;
    rd_skip_ws(&rd);
    if (rd_peek(&rd) == (uint8_t)'}')
    {
        rd.pos++;
        b_done = true;
    }

    while (!b_done)
    {
        uint16_t key_start;
        uint16_t key_n;
        val_t    val;

        rd_skip_ws(&rd);
        if ((rd_peek(&rd) != (uint8_t)'"') || !rd_string(&rd, &key_start, &key_n))
        {
            return COMMS_ERR_BAD_JSON;
        }
        rd_skip_ws(&rd);
        if (rd_peek(&rd) != (uint8_t)':')
        {
            return COMMS_ERR_BAD_JSON;
        }
        rd.pos++;
        rd_skip_ws(&rd);
        if (!rd_value(&rd, &val))
        {
            return COMMS_ERR_BAD_JSON;
        }

        if (str_eq(&p_data[key_start], key_n, "id"))
        {
            if ((val.kind == VAL_INT) && (val.value >= 0))
            {
                p_cmd->id       = (uint32_t)val.value;
                p_cmd->b_has_id = true;
            }
            else
            {
                p_cmd->b_has_id = false;
            }
        }
        else if (str_eq(&p_data[key_start], key_n, "cmd"))
        {
            cmd_val = val;
        }
        else if (str_eq(&p_data[key_start], key_n, "param"))
        {
            param_val = val;
        }
        else if (str_eq(&p_data[key_start], key_n, "value"))
        {
            value_val = val;
        }
        else if (str_eq(&p_data[key_start], key_n, "left"))
        {
            map_val[0] = val;
        }
        else if (str_eq(&p_data[key_start], key_n, "right"))
        {
            map_val[1] = val;
        }
        else if (str_eq(&p_data[key_start], key_n, "straight"))
        {
            map_val[2] = val;
        }
        else if (str_eq(&p_data[key_start], key_n, "uturn"))
        {
            map_val[3] = val;
        }
        else
        {
            /* unknown keys are ignored */
        }

        rd_skip_ws(&rd);
        if (rd_peek(&rd) == (uint8_t)',')
        {
            rd.pos++;
        }
        else if (rd_peek(&rd) == (uint8_t)'}')
        {
            rd.pos++;
            b_done = true;
        }
        else
        {
            return COMMS_ERR_BAD_JSON;
        }
    }

    /* Only whitespace or NUL may follow the object. */
    while (rd_more(&rd))
    {
        uint8_t c = rd_peek(&rd);

        if ((c != 0u) && (c != (uint8_t)' ') && (c != (uint8_t)'\t') &&
            (c != (uint8_t)'\r') && (c != (uint8_t)'\n'))
        {
            return COMMS_ERR_BAD_JSON;
        }
        rd.pos++;
    }

    if (!p_cmd->b_has_id)
    {
        return COMMS_ERR_NO_ID;
    }
    if (cmd_val.kind != VAL_STR)
    {
        return COMMS_ERR_BAD_CMD;
    }

    if (str_eq(&p_data[cmd_val.start], cmd_val.n, "start"))
    {
        p_cmd->cmd = (uint8_t)COMMS_CMD_START;
    }
    else if (str_eq(&p_data[cmd_val.start], cmd_val.n, "stop"))
    {
        p_cmd->cmd = (uint8_t)COMMS_CMD_STOP;
    }
    else if (str_eq(&p_data[cmd_val.start], cmd_val.n, "estop"))
    {
        p_cmd->cmd = (uint8_t)COMMS_CMD_ESTOP;
    }
    else if (str_eq(&p_data[cmd_val.start], cmd_val.n, "reset"))
    {
        p_cmd->cmd = (uint8_t)COMMS_CMD_RESET;
    }
    else if (str_eq(&p_data[cmd_val.start], cmd_val.n, "set"))
    {
        comms_param_t const * p_param;

        p_cmd->cmd = (uint8_t)COMMS_CMD_SET;
        if (param_val.kind != VAL_STR)
        {
            return COMMS_ERR_BAD_PARAM;
        }
        p_param = comms_param_by_name((char const *)&p_data[param_val.start], param_val.n);
        if (p_param == NULL)
        {
            return COMMS_ERR_BAD_PARAM;
        }
        p_cmd->param_id = p_param->id;
        if ((value_val.kind != VAL_INT) ||
            (value_val.value < p_param->min) || (value_val.value > p_param->max))
        {
            return COMMS_ERR_RANGE;
        }
        p_cmd->value = value_val.value;
    }
    else if (str_eq(&p_data[cmd_val.start], cmd_val.n, "map"))
    {
        char     ch[4];
        uint32_t packed = 0u;
        uint8_t  k;
        uint8_t  j;

        p_cmd->cmd = (uint8_t)COMMS_CMD_MAP;
        for (k = 0u; k < 4u; k++)
        {
            if ((map_val[k].kind != VAL_STR) || (map_val[k].n != 1u))
            {
                return COMMS_ERR_BAD_MAP;
            }
            ch[k] = (char)p_data[map_val[k].start];
            if ((ch[k] >= 'a') && (ch[k] <= 'z'))
            {
                ch[k] = (char)(ch[k] - 'a' + 'A');      /* "f" means F */
            }
            if (!comms_map_char_ok(ch[k]))
            {
                return COMMS_ERR_BAD_MAP;
            }
            for (j = 0u; j < k; j++)
            {
                if (ch[j] == ch[k])
                {
                    return COMMS_ERR_BAD_MAP;
                }
            }
            packed |= ((uint32_t)(uint8_t)ch[k]) << (8u * k);
        }
        p_cmd->value = (int32_t)packed;
    }
    else
    {
        return COMMS_ERR_BAD_CMD;
    }
    return COMMS_ERR_NONE;
}

bool comms_map_char_ok(char c)
{
    uint8_t i;

    for (i = 0u; i < (uint8_t)(sizeof(k_map_chars) - 1u); i++)
    {
        if (k_map_chars[i] == c)
        {
            return true;
        }
    }
    return false;
}

comms_err_t comms_check_ready(comms_cmd_t const * p_cmd, bool b_calibrated)
{
    if ((p_cmd->cmd == (uint8_t)COMMS_CMD_START) && !b_calibrated)
    {
        return COMMS_ERR_NOT_CAL;
    }
    return COMMS_ERR_NONE;
}

comms_err_t comms_check_cmd(comms_cmd_t const * p_cmd, bool b_idle)
{
    if (((p_cmd->cmd == (uint8_t)COMMS_CMD_START) ||
         (p_cmd->cmd == (uint8_t)COMMS_CMD_SET) ||
         (p_cmd->cmd == (uint8_t)COMMS_CMD_MAP)) && !b_idle)
    {
        return COMMS_ERR_BUSY;
    }
    return COMMS_ERR_NONE;
}

/* ---------------------------------------------------------------------
 *  Duplicate suppression
 * --------------------------------------------------------------------- */
void comms_dedup_init(comms_dedup_t * p_dd)
{
    p_dd->next  = 0u;
    p_dd->count = 0u;
}

bool comms_cmd_repeat_safe(uint8_t cmd)
{
    return (cmd == (uint8_t)COMMS_CMD_STOP) || (cmd == (uint8_t)COMMS_CMD_ESTOP);
}

comms_dedup_result_t comms_dedup_check(comms_dedup_t const * p_dd,
                                       comms_cmd_t const * p_cmd, uint32_t now_ms)
{
    uint8_t i;

    for (i = 0u; i < p_dd->count; i++)
    {
        comms_dedup_entry_t const * p_e = &p_dd->e[i];

        /* Unsigned difference: correct across the 32-bit ms wrap. */
        if ((p_e->id == p_cmd->id) && ((uint32_t)(now_ms - p_e->t_ms) < COMMS_DEDUP_WINDOW_MS))
        {
            if ((p_e->cmd == p_cmd->cmd) && (p_e->param_id == p_cmd->param_id) &&
                (p_e->value == p_cmd->value))
            {
                return COMMS_DEDUP_REPEAT;
            }
            return COMMS_DEDUP_CONFLICT;
        }
    }
    return COMMS_DEDUP_NEW;
}

void comms_dedup_add(comms_dedup_t * p_dd, comms_cmd_t const * p_cmd, uint32_t now_ms)
{
    comms_dedup_entry_t * p_e = &p_dd->e[p_dd->next];

    p_e->id       = p_cmd->id;
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
void comms_ring_init(comms_ring_t * p_ring)
{
    p_ring->head  = 0u;
    p_ring->count = 0u;
}

bool comms_ring_push(comms_ring_t * p_ring, comms_rec_t const * p_rec, uint32_t seq)
{
    bool    b_overwrote = false;
    uint8_t idx;

    if (p_ring->count == COMMS_RING_LEN)
    {
        p_ring->head = (uint8_t)((p_ring->head + 1u) % COMMS_RING_LEN);  /* drop oldest */
        p_ring->count--;
        b_overwrote = true;
    }
    idx = (uint8_t)((p_ring->head + p_ring->count) % COMMS_RING_LEN);
    p_ring->rec[idx] = *p_rec;
    p_ring->seq[idx] = seq;
    p_ring->count++;
    return b_overwrote;
}

bool comms_ring_peek(comms_ring_t const * p_ring, comms_rec_t * p_rec, uint32_t * p_seq)
{
    if (p_ring->count == 0u)
    {
        return false;
    }
    *p_rec = p_ring->rec[p_ring->head];
    *p_seq = p_ring->seq[p_ring->head];
    return true;
}

void comms_ring_pop(comms_ring_t * p_ring)
{
    if (p_ring->count > 0u)
    {
        p_ring->head = (uint8_t)((p_ring->head + 1u) % COMMS_RING_LEN);
        p_ring->count--;
    }
}
