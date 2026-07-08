/* damiao.c - DaMiao DM-series motor codec. See damiao.h. */
#include "damiao.h"
#include <string.h>

/* Model table: p_max (rad), v_max (rad/s), t_max (N*m). Mirrors OpenArm's
 * DAMIAO_MODEL_LIMITS. All models share p_max=12.5, so position scaling is
 * model-independent (only v/t scaling differs). */
static const struct { const char *name; float p, v, t; } DM_MODELS[] = {
    {"4310",     12.5f, 30.0f, 10.0f},
    {"4310P",    12.5f, 50.0f, 10.0f},
    {"4340",     12.5f, 10.0f, 28.0f},
    {"4340P",    12.5f, 10.0f, 28.0f},
    {"4340_v20", 12.5f, 20.0f, 28.0f},
    {"6006",     12.5f, 45.0f, 12.0f},
    {"8006",     12.5f, 45.0f, 20.0f},
    {"8009",     12.5f, 45.0f, 54.0f},
    {"10010",    12.5f, 25.0f, 200.0f},
};

int dm_limits_for_model(const char *model, dm_limits_t *out)
{
    if (model) {
        for (unsigned i = 0; i < sizeof DM_MODELS / sizeof DM_MODELS[0]; i++)
            if (strcmp(model, DM_MODELS[i].name) == 0) {
                out->p_max = DM_MODELS[i].p;
                out->v_max = DM_MODELS[i].v;
                out->t_max = DM_MODELS[i].t;
                return 1;
            }
    }
    out->p_max = 12.5f; out->v_max = 30.0f; out->t_max = 10.0f;   /* generic */
    return 0;
}

uint32_t dm_float_to_uint(float x, float xmin, float xmax, int bits)
{
    float span = xmax - xmin;
    uint32_t max_code = (bits >= 32) ? 0xFFFFFFFFu : ((1u << bits) - 1u);
    if (span <= 0.0f) return 0;
    if (x < xmin) x = xmin;
    if (x > xmax) x = xmax;
    return (uint32_t)((x - xmin) * (float)max_code / span + 0.5f);
}

float dm_uint_to_float(uint32_t xi, float xmin, float xmax, int bits)
{
    uint32_t max_code = (bits >= 32) ? 0xFFFFFFFFu : ((1u << bits) - 1u);
    float span = xmax - xmin;
    if (max_code == 0) return xmin;
    return (float)xi * span / (float)max_code + xmin;
}

void dm_build_mit(dm_frame_t *f, uint32_t id, const dm_limits_t *lim,
                  float pos, float vel, float kp, float kd, float tau)
{
    uint32_t p  = dm_float_to_uint(pos, -lim->p_max, lim->p_max, 16);
    uint32_t v  = dm_float_to_uint(vel, -lim->v_max, lim->v_max, 12);
    uint32_t ki = dm_float_to_uint(kp, DM_KP_MIN, DM_KP_MAX, 12);
    uint32_t di = dm_float_to_uint(kd, DM_KD_MIN, DM_KD_MAX, 12);
    uint32_t t  = dm_float_to_uint(tau, -lim->t_max, lim->t_max, 12);

    f->id  = id + DM_ID_MIT;
    f->dlc = 8;
    f->data[0] = (uint8_t)(p >> 8);
    f->data[1] = (uint8_t)(p & 0xFF);
    f->data[2] = (uint8_t)(v >> 4);
    f->data[3] = (uint8_t)(((v & 0xF) << 4) | (ki >> 8));
    f->data[4] = (uint8_t)(ki & 0xFF);
    f->data[5] = (uint8_t)(di >> 4);
    f->data[6] = (uint8_t)(((di & 0xF) << 4) | (t >> 8));
    f->data[7] = (uint8_t)(t & 0xFF);
}

void dm_build_pos_vel(dm_frame_t *f, uint32_t id, float pos, float vlim)
{
    f->id = id + DM_ID_POSVEL;
    f->dlc = 8;
    memcpy(&f->data[0], &pos, 4);    /* little-endian host (x86/ARM) */
    memcpy(&f->data[4], &vlim, 4);
}

static void dm_special(dm_frame_t *f, uint32_t id, uint8_t last)
{
    f->id = id; f->dlc = 8;
    for (int i = 0; i < 7; i++) f->data[i] = 0xFF;
    f->data[7] = last;
}
void dm_build_enable(dm_frame_t *f, uint32_t id)   { dm_special(f, id, 0xFC); }
void dm_build_disable(dm_frame_t *f, uint32_t id)  { dm_special(f, id, 0xFD); }
void dm_build_set_zero(dm_frame_t *f, uint32_t id) { dm_special(f, id, 0xFE); }

void dm_build_read_reg(dm_frame_t *f, uint32_t id, uint8_t rid)
{
    f->id = 0x7FF; f->dlc = 4;
    memset(f->data, 0, sizeof f->data);
    f->data[0] = (uint8_t)(id & 0xFF);
    f->data[1] = (uint8_t)(id >> 8);
    f->data[2] = 0x33;               /* read command */
    f->data[3] = rid;
}

void dm_build_write_reg_u32(dm_frame_t *f, uint32_t id, uint8_t rid, uint32_t val)
{
    f->id = 0x7FF; f->dlc = 8;
    f->data[0] = (uint8_t)(id & 0xFF);
    f->data[1] = (uint8_t)(id >> 8);
    f->data[2] = 0x55;               /* write command */
    f->data[3] = rid;
    memcpy(&f->data[4], &val, 4);    /* little-endian */
}

int dm_parse_feedback(const dm_frame_t *f, const dm_limits_t *lim, dm_state_t *st)
{
    if (f->dlc < 8) return 0;
    st->id  = f->data[0] & 0x0F;
    st->err = (f->data[0] >> 4) & 0x0F;
    uint32_t p = ((uint32_t)f->data[1] << 8) | f->data[2];
    uint32_t v = ((uint32_t)f->data[3] << 4) | (f->data[4] >> 4);
    uint32_t t = (((uint32_t)f->data[4] & 0x0F) << 8) | f->data[5];
    st->pos = dm_uint_to_float(p, -lim->p_max, lim->p_max, 16);
    st->vel = dm_uint_to_float(v, -lim->v_max, lim->v_max, 12);
    st->tau = dm_uint_to_float(t, -lim->t_max, lim->t_max, 12);
    st->t_mos   = f->data[6];
    st->t_rotor = f->data[7];
    return 1;
}

const char *dm_status_text(int err)
{
    switch (err) {
        case 0:  return "disabled";
        case 1:  return "enabled";
        case 8:  return "overvoltage";
        case 9:  return "undervoltage";
        case 10: return "overcurrent";
        case 11: return "mos-overtemp";
        case 12: return "rotor-overtemp";
        case 13: return "comm-loss";
        case 14: return "overload";
        default: return "unknown";
    }
}
