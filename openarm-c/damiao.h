/* damiao.h - DaMiao DM-series motor codec (classic CAN 2.0, MIT protocol).
 *
 * Portable, no-OS: builds 8-byte CAN frames and parses feedback. Reused by the
 * control engine and calibration. Written fresh for the OpenArm-C application.
 */
#ifndef DAMIAO_H
#define DAMIAO_H

#include <stdint.h>

/* MIT gain ranges (fixed by DaMiao firmware, independent of motor model). */
#define DM_KP_MIN 0.0f
#define DM_KP_MAX 500.0f
#define DM_KD_MIN 0.0f
#define DM_KD_MAX 5.0f

/* Arbitration-id offsets by control mode (added to motor id). */
#define DM_ID_MIT     0x000u
#define DM_ID_POSVEL  0x100u
#define DM_ID_VEL     0x200u

/* Feedback frame id = motor id + this offset. */
#define DM_FB_OFFSET  0x10u

/* Register ids (0x7FF config protocol). */
#define DM_RID_MST_ID     7u
#define DM_RID_ESC_ID     8u
#define DM_RID_CTRL_MODE  10u   /* 1=MIT 2=PosVel 3=Vel 4=ForcePos */
#define DM_RID_PMAX       21u
#define DM_RID_VMAX       22u
#define DM_RID_TMAX       23u

/* A raw classic-CAN frame (<=8 data bytes). */
typedef struct { uint32_t id; uint8_t dlc; uint8_t data[8]; } dm_frame_t;

/* Per-model mapping ranges (position/velocity/torque full-scale). */
typedef struct { float p_max, v_max, t_max; } dm_limits_t;

/* Decoded MIT feedback. */
typedef struct {
    int      id;        /* motor id from the feedback frame low nibble */
    int      err;       /* status/error code (feedback high nibble)     */
    float    pos, vel, tau;
    int      t_mos, t_rotor;
} dm_state_t;

/* Named model -> limits, mirroring the OpenArm model table. Returns 1 if known. */
int  dm_limits_for_model(const char *model, dm_limits_t *out);

/* Scaling helpers (public for calibration/tests). */
uint32_t dm_float_to_uint(float x, float xmin, float xmax, int bits);
float    dm_uint_to_float(uint32_t xi, float xmin, float xmax, int bits);

/* Frame builders. */
void dm_build_mit(dm_frame_t *f, uint32_t id, const dm_limits_t *lim,
                  float pos, float vel, float kp, float kd, float tau);
void dm_build_pos_vel(dm_frame_t *f, uint32_t id, float pos, float vlim);
void dm_build_enable(dm_frame_t *f, uint32_t id);
void dm_build_disable(dm_frame_t *f, uint32_t id);
void dm_build_set_zero(dm_frame_t *f, uint32_t id);
void dm_build_read_reg(dm_frame_t *f, uint32_t id, uint8_t rid);
void dm_build_write_reg_u32(dm_frame_t *f, uint32_t id, uint8_t rid, uint32_t val);

/* Parse an MIT feedback frame into st. Returns 1 on success. */
int  dm_parse_feedback(const dm_frame_t *f, const dm_limits_t *lim, dm_state_t *st);

/* Human-readable status/error text. */
const char *dm_status_text(int err);

#endif /* DAMIAO_H */
