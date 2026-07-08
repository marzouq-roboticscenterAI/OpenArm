/* calib.c - bounded joint calibration. See calib.h. */
#define _GNU_SOURCE
#include "calib.h"
#include "socketcan.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>

/* Tunables (seconds / radians). Bounded so nothing hangs. */
#define SWEEP_DEADLINE_S   30.0    /* absolute cap per sweep direction (slower) */
#define STARTUP_GRACE_S     0.8    /* ignore stall until motion could start   */
#define STALL_DWELL_S       0.60   /* pos must be static this long to be a stop*/
#define STALL_EPS_RAD       0.02   /* "static" threshold                       */
#define SWEEP_SPEED         0.6    /* rad/s the commanded target advances       */
#define LEAD_RAD            0.12    /* commanded target stays this far ahead    */
#define GRAV_LEAD           0.20    /* larger lead for gravity-loaded joints    */
#define RAMP_IN             0.8    /* seconds to ramp kp/lead in (soft start)  */
#define KD_HOLD             1.0f
#define KD_CAP              2.5f

/* J1/J2 (shoulder) and J4 (forearm) hold the weight of the whole arm; they need
 * much more torque to sweep uphill into their hardstops than the wrist joints. */
static int is_gravity_joint(uint32_t id) { return id == 1 || id == 2 || id == 4; }

/* Max degrees a joint may sweep FROM its start pose in EACH direction during
 * calibration; 0 = uncapped. This keeps the gravity joints from swinging the arm
 * past its balance point and flipping it -- the sweep stops at the cap if it
 * hasn't hit a real hardstop first. Env-tunable (degrees). */
static float env_deg(const char *name, float def_deg)
{ const char *e = getenv(name); if (e && *e) { float d = (float)atof(e); if (d > 0) return d; } return def_deg; }
static float joint_travel_cap(uint32_t id)   /* radians per direction; 0 = uncapped */
{
    float deg = 0.0f;
    if      (id == 1) deg = env_deg("CAL_TRAVEL_J1", 135.0f);   /* base:     +/-135 -> 270 span */
    else if (id == 2) deg = env_deg("CAL_TRAVEL_J2", 90.0f);    /* shoulder: +/-90  (no flip)   */
    else if (id == 4) deg = env_deg("CAL_TRAVEL_J4", 100.0f);   /* forearm:  +/-100             */
    else return 0.0f;
    return deg * 3.14159265f / 180.0f;
}

static double now_s(void)
{
    struct timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec * 1e-9;
}
static void sleep_s(double s)
{
    struct timespec ts = { (time_t)s, (long)((s - (long)s) * 1e9) };
    nanosleep(&ts, NULL);
}

/* Sweep kp scales with the motor's torque capacity (heavy joints need more),
 * but kept gentle so the joint eases into its hardstops rather than slamming. */
static float sweep_kp(uint32_t id, const dm_limits_t *lim)
{
    /* enough sustained torque to actually reach the hardstops (so the range is
     * full), while the soft-start ramp keeps the initial motion gentle. Gravity
     * joints get a big boost + higher cap so they can climb into their stops. */
    if (is_gravity_joint(id)) {
        float k = lim->t_max * 2.5f;
        if (k < 50.0f)  k = 50.0f;
        if (k > 150.0f) k = 150.0f;
        return k;
    }
    float k = lim->t_max * 0.7f;
    if (k < 10.0f) k = 10.0f;
    if (k > 35.0f) k = 35.0f;
    return k;
}

/* Optional companion joint held (streamed) while another joint sweeps. */
static struct { int on; uint32_t id; float pos, kp, kd; dm_limits_t lim; } g_hold = {0,0,0,0,0,{0,0,0}};
void calib_hold_companion(uint32_t id, float pos, const dm_limits_t *lim, float kp, float kd)
{ g_hold.on = 1; g_hold.id = id; g_hold.pos = pos; g_hold.lim = *lim; g_hold.kp = kp; g_hold.kd = kd; }
void calib_hold_clear(void) { g_hold.on = 0; }
static void companion_tick(int fd)
{
    if (!g_hold.on) return;
    dm_frame_t f; dm_build_mit(&f, g_hold.id, &g_hold.lim, g_hold.pos, 0, g_hold.kp, g_hold.kd, 0);
    scan_send(fd, &f);
}

static int read_fb(int fd, uint32_t fb_id, const dm_limits_t *lim, dm_state_t *st, int to_ms)
{
    dm_frame_t f;
    double end = now_s() + to_ms / 1000.0;
    do {
        int r = scan_recv(fd, &f, to_ms);
        if (r == 1 && f.id == fb_id && dm_parse_feedback(&f, lim, st)) return 1;
        if (r < 0) return -1;
    } while (now_s() < end);
    return 0;
}

/* Advance a MIT target in `dir` until the joint stalls against a hardstop.
 * Returns 1 and *stop=hardstop position, or 0 on timeout/abort. */
static int sweep_to_stop(int fd, uint32_t id, const dm_limits_t *lim, int dir,
                         volatile int *abort, float *stop, char *msg,
                         int has_limit, float limit)
{
    uint32_t fb = id + DM_FB_OFFSET;
    float kp = sweep_kp(id, lim), kd = KD_HOLD;
    if (kd > KD_CAP) kd = KD_CAP;
    float lead_max = is_gravity_joint(id) ? GRAV_LEAD : LEAD_RAD;

    dm_state_t st;
    dm_frame_t f;
    /* seed current position */
    dm_build_mit(&f, id, lim, 0, 0, 0, 0, 0); scan_send(fd, &f);
    if (read_fb(fd, fb, lim, &st, 200) != 1) { snprintf(msg, 96, "no feedback"); return 0; }

    float pos = st.pos, cmd = st.pos, last_pos = st.pos;
    double t0 = now_s(), last_prog = t0;

    while (!(*abort)) {
        double t = now_s();
        if (t - t0 > SWEEP_DEADLINE_S) { snprintf(msg, 96, "no hardstop (timeout at %.3f)", pos); return 0; }

        /* Soft start: ramp kp (and the lead) in over RAMP_IN so the joint EASES
         * into motion instead of jerking the instant it's commanded. */
        float ramp = (t - t0 < RAMP_IN) ? (float)((t - t0) / RAMP_IN) : 1.0f;
        float kp_eff = kp * ramp;
        float lead = lead_max * ramp;

        cmd += dir * SWEEP_SPEED * 0.02f;                 /* advance target ~50 Hz */
        if (has_limit) {                                   /* never command past the cap */
            if (dir < 0 && cmd < limit) cmd = limit;
            if (dir > 0 && cmd > limit) cmd = limit;
        }
        float target = cmd;                                /* commanded runs ahead */
        if (dir < 0 && target < pos - lead) target = pos - lead;
        if (dir > 0 && target > pos + lead) target = pos + lead;
        dm_build_mit(&f, id, lim, target, 0, kp_eff, kd, 0);
        if (scan_send(fd, &f) < 0) { snprintf(msg, 96, "CAN send failed"); return 0; }
        companion_tick(fd);                                /* hold J2 while J1 sweeps */

        if (read_fb(fd, fb, lim, &st, 5) == 1) {
            if (st.err >= 8) { snprintf(msg, 96, "fault: %s", dm_status_text(st.err)); return 0; }
            pos = st.pos;
            if (fabsf(pos - last_pos) > STALL_EPS_RAD) { last_pos = pos; last_prog = t; }
        }
        /* range cap reached before a hardstop -> use it as the (soft) stop */
        if (has_limit && ((dir < 0 && pos <= limit) || (dir > 0 && pos >= limit))) {
            *stop = limit;
            snprintf(msg, 96, "range cap reached (%.3f rad)", pos);
            return 1;
        }
        /* stall = no progress for a dwell, past the startup grace */
        if (t - t0 > STARTUP_GRACE_S && t - last_prog > STALL_DWELL_S) {
            *stop = pos;
            return 1;
        }
        sleep_s(0.02);
    }
    snprintf(msg, 96, "aborted");
    return 0;
}

/* Ramp smoothly from the current pose to target over `secs` (no jump). */
static void move_to(int fd, uint32_t id, const dm_limits_t *lim, float target,
                    float kp, float kd, double secs, volatile int *abort)
{
    dm_frame_t f; dm_state_t st;
    float start = target;
    dm_build_mit(&f, id, lim, 0, 0, 0, 0, 0); scan_send(fd, &f);
    if (read_fb(fd, id + DM_FB_OFFSET, lim, &st, 200) == 1) start = st.pos;
    double t0 = now_s();
    while (now_s() - t0 < secs && !(*abort)) {
        double el = now_s() - t0;
        float a = (el < secs * 0.8) ? (float)(el / (secs * 0.8)) : 1.0f;   /* ramp over 80%, hold */
        float cur = start + (target - start) * a;
        dm_build_mit(&f, id, lim, cur, 0, kp, kd, 0);
        scan_send(fd, &f);
        companion_tick(fd);
        read_fb(fd, id + DM_FB_OFFSET, lim, &st, 5);
        sleep_s(0.02);
    }
}

void calib_gentle_release(int fd, uint32_t id, const dm_limits_t *lim, volatile int *abort)
{
    dm_frame_t f; dm_state_t st;
    double t0 = now_s();
    while (now_s() - t0 < 3.5 && !(*abort)) {
        /* kp=0 -> no position hold; kd -> damps velocity, so gravity lowers the
         * joint slowly instead of free-dropping. */
        dm_build_mit(&f, id, lim, 0, 0, 0.0f, KD_HOLD, 0);
        scan_send(fd, &f);
        companion_tick(fd);
        int settled = 0;
        if (read_fb(fd, id + DM_FB_OFFSET, lim, &st, 5) == 1)
            settled = (now_s() - t0 > 0.6) && fabsf(st.vel) < 0.04f;
        if (settled) break;      /* reached rest / a stop -> stop damping */
        sleep_s(0.02);
    }
    dm_build_disable(&f, id); scan_send(fd, &f);
}

int calib_run(int fd, uint32_t id, const dm_limits_t *lim, int mode,
              volatile int *abort, calib_result_t *res)
{
    memset(res, 0, sizeof *res);
    dm_frame_t f;

    /* Force MIT mode (register write valid only while disabled), then enable. */
    dm_build_disable(&f, id); scan_send(fd, &f); sleep_s(0.03);
    dm_build_write_reg_u32(&f, id, DM_RID_CTRL_MODE, 1u); scan_send(fd, &f); sleep_s(0.05);
    dm_build_enable(&f, id); scan_send(fd, &f); sleep_s(0.05);

    /* Record the joint's initial angle (pre-zero frame) BEFORE any sweep moves it,
     * so we can put the slider back where the operator left the arm afterwards. */
    float initial_raw = 0; int have_initial = 0;
    { dm_state_t st0; dm_build_mit(&f, id, lim, 0,0,0,0,0); scan_send(fd, &f);
      if (read_fb(fd, id + DM_FB_OFFSET, lim, &st0, 200) == 1) { initial_raw = st0.pos; have_initial = 1; } }

    float lo = 0, hi = 0;
    float kp = sweep_kp(id, lim), kd = KD_HOLD; if (kd > KD_CAP) kd = KD_CAP;

    /* Per-direction travel cap from the START pose (gravity joints): the sweep
     * won't drive the arm more than this far from where it began, so it can't
     * swing over-center and flip. Reverse limit = start-cap, forward = start+cap. */
    float tc = joint_travel_cap(id);
    int   cap = tc > 0.0f;

    if (!sweep_to_stop(fd, id, lim, -1, abort, &lo, res->msg, cap, initial_raw - tc)) goto fail;
    if (!sweep_to_stop(fd, id, lim, +1, abort, &hi, res->msg, cap, initial_raw + tc)) goto fail;
    if (hi - lo < 0.05f) { snprintf(res->msg, 96, "span too small (%.3f)", hi - lo); goto fail; }

    res->low_stop = lo; res->high_stop = hi; res->span = hi - lo;

    float zero_at;
    if (mode == CAL_GRIPPER) {
        zero_at = lo;                          /* closed = low stop */
    } else if (mode == CAL_HANG) {
        /* lower slowly to gravity rest (don't just cut power), then read rest */
        calib_gentle_release(fd, id, lim, abort);
        dm_state_t st; float rest = (lo + hi) * 0.5f;
        dm_build_enable(&f, id); scan_send(fd, &f); sleep_s(0.05);
        dm_build_mit(&f, id, lim, 0, 0, 0, 0, 0); scan_send(fd, &f);
        if (read_fb(fd, id + DM_FB_OFFSET, lim, &st, 200) == 1) rest = st.pos;
        zero_at = rest;
    } else { /* CAL_CENTER */
        zero_at = (lo + hi) * 0.5f;
        move_to(fd, id, lim, zero_at, kp, kd, 2.5, abort);
    }

    /* zero the encoder here */
    dm_build_set_zero(&f, id); scan_send(fd, &f); sleep_s(0.05);

    /* recompute stops relative to the new zero */
    res->pmin = lo - zero_at;
    res->pmax = hi - zero_at;
    res->zero = 0.0f;
    /* initial angle mapped into the new (zeroed) frame, clamped to the range;
     * this is where the slider should start (back to the operator's start pose) */
    res->start_pos = (have_initial ? initial_raw : zero_at) - zero_at;
    if (res->start_pos < res->pmin) res->start_pos = res->pmin;
    if (res->start_pos > res->pmax) res->start_pos = res->pmax;
    /* release torque gently (lower under gravity rather than free-drop) */
    calib_gentle_release(fd, id, lim, abort);

    res->ok = 1;
    snprintf(res->msg, 96, "ok span=%.3f range[%.3f,%.3f]", res->span, res->pmin, res->pmax);
    return 1;

fail:
    dm_build_disable(&f, id); scan_send(fd, &f);
    res->ok = 0;
    return 0;
}
