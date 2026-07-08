/* control.c - OpenArm-C control engine. See control.h. */
#define _GNU_SOURCE
#include "control.h"
#include "damiao.h"
#include "socketcan.h"
#include "gamepad.h"
#include "calib.h"
#include "log.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <pthread.h>

#define CTRL_HZ       50
#define SLIDER_SPEED  1.6f     /* rad/s at full stick */
#define SLEW_RATE     3.5f     /* rad/s the commanded target eases toward goal */
#define MAX_TRAVEL    3.0f     /* uncalibrated jog span from start pose */
#define RETURN_SECS   2.5      /* gentle return-to-start duration after calibration */
#define KD_CAP        2.5f     /* matches OpenArm: Kd>=~3.5 limit-cycles */
#define CALIB_FILE    "arm_calib.txt"

/* ---- per-joint gains by model tier (id map == calibration) ---- */
static void joint_gains(int id, float scale, float *kp, float *kd)
{
    if      (id <= 2) { *kp = 40.0f; *kd = 2.0f; }   /* 8009 base/shoulder */
    else if (id <= 4) { *kp = 22.0f; *kd = 1.2f; }   /* 4340               */
    else              { *kp = 12.0f; *kd = 0.6f; }   /* 4310 wrist/grip    */
    *kp *= scale;
    if (*kd > KD_CAP) *kd = KD_CAP;
    if (*kp > DM_KP_MAX) *kp = DM_KP_MAX;
    if (*kp < 0) *kp = 0;
    if (*kd < 0) *kd = 0;
}
static const char *model_for(int id)
{ return (id <= 2) ? "8009" : (id <= 4) ? "4340" : "4310"; }

/* ---- shared state ---- */
static oa_state_t     S;
static int            g_fd[OA_NBUS];
static pthread_t      g_thread;
static pthread_mutex_t g_lock = PTHREAD_MUTEX_INITIALIZER;
static volatile int   g_stop = 0;
static volatile int   g_run  = 0;
static volatile int   g_cal_abort = 0;   /* set by E-STOP/Ctrl-C to abort a running calibration */

/* command flags (under g_lock) */
static int   c_estop, c_clear, c_calib, c_connect, c_disconnect;
static int   c_manual_start, c_manual_stop, c_manual_mark;
static int   mc_count[OA_NBUS][OA_MAX_MOTOR + 1];   /* endpoints recorded (0/1/2) */
static float mc_first[OA_NBUS][OA_MAX_MOTOR + 1];   /* first recorded endpoint (raw) */
static float c_jog[OA_NBUS][OA_MAX_MOTOR + 1];
static float c_target[OA_NBUS][OA_MAX_MOTOR + 1];   /* absolute target (sliders) */
static int   c_target_set[OA_NBUS][OA_MAX_MOTOR + 1];
static int   c_sel_bus = -1, c_sel_motor = -1;
static float c_gain_scale = 1.0f;

static double now_s(void)
{ struct timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts); return ts.tv_sec + ts.tv_nsec*1e-9; }
static void sleep_ms(int ms)
{ struct timespec ts = { ms/1000, (long)(ms%1000)*1000000L }; nanosleep(&ts, NULL); }

/* limits used for MIT scaling: all models share p_max=12.5, so generic is fine */
static void bus_limits(int id, dm_limits_t *lim) { dm_limits_for_model(model_for(id), lim); }

static int read_pos_err(int fd, int id, const dm_limits_t *lim, float *pos, int *err)
{
    dm_frame_t f; dm_build_mit(&f, id, lim, 0, 0, 0, 0, 0);
    if (scan_send(fd, &f) < 0) return 0;
    for (int i = 0; i < 6; i++) {
        int r = scan_recv(fd, &f, 6);
        if (r == 1 && f.id == (uint32_t)(id + DM_FB_OFFSET)) {
            dm_state_t st;
            if (dm_parse_feedback(&f, lim, &st)) { *pos = st.pos; if (err) *err = st.err; return 1; }
        }
        if (r < 0) return 0;
    }
    return 0;
}

/* ---- calibration file (iface id pmin pmax) ---- */
static void load_calib(void)
{
    FILE *fp = fopen(CALIB_FILE, "r");
    if (!fp) return;
    char ifn[32]; int id; float lo, hi;
    while (fscanf(fp, "%31s %d %f %f", ifn, &id, &lo, &hi) == 4) {
        for (int b = 0; b < S.nbus; b++)
            if (!strcmp(ifn, S.bus[b].iface) && id >= OA_MIN_MOTOR && id <= OA_MAX_MOTOR) {
                S.bus[b].m[id].tmin = lo; S.bus[b].m[id].tmax = hi; S.bus[b].m[id].has_cal = 1;
            }
    }
    fclose(fp);
}
static void save_calib_locked(void)
{
    FILE *fp = fopen(CALIB_FILE, "w");
    if (!fp) return;
    for (int b = 0; b < S.nbus; b++)
        for (int id = OA_MIN_MOTOR; id <= OA_MAX_MOTOR; id++)
            if (S.bus[b].m[id].has_cal)
                fprintf(fp, "%s %d %.5f %.5f\n", S.bus[b].iface, id,
                        S.bus[b].m[id].tmin, S.bus[b].m[id].tmax);
    fclose(fp);
}

int control_init(const char *ifaces[], int n)
{
    memset(&S, 0, sizeof S);
    S.gain_scale = 1.0f;
    S.sel_bus = 0; S.sel_motor = 0;
    if (n > OA_NBUS) n = OA_NBUS;
    S.nbus = n;
    for (int b = 0; b < n; b++) {
        snprintf(S.bus[b].iface, sizeof S.bus[b].iface, "%s", ifaces[b]);
        g_fd[b] = scan_open(ifaces[b]);
        S.bus[b].opened = (g_fd[b] >= 0);
        if (g_fd[b] < 0) { snprintf(S.status, sizeof S.status, "cannot open %s", ifaces[b]); continue; }
        for (int id = OA_MIN_MOTOR; id <= OA_MAX_MOTOR; id++) {
            dm_limits_t lim; bus_limits(id, &lim);
            float p; int e = 0;
            if (read_pos_err(g_fd[b], id, &lim, &p, &e)) {
                oa_motor_t *m = &S.bus[b].m[id];
                m->present = 1; m->pos = m->target = p; m->err = e;
                m->tmin = m->tmax = p;
                joint_gains(id, S.gain_scale, &m->kp, &m->kd);
                snprintf(m->model, sizeof m->model, "%s", model_for(id));
                S.bus[b].nmotors++;
            }
        }
    }
    load_calib();
    /* Decide jog bounds. A saved calibration is only valid if the joint's CURRENT
     * position falls inside it (same zero frame); otherwise the ranges are stale
     * (e.g. after a power-cycle) and would clamp the joint to a wrong window and
     * snap it on start. In that case fall back to a symmetric window around the
     * current pose so the joint moves freely both directions. Target = current
     * pose either way (no snap). */
    for (int b = 0; b < n; b++)
        for (int id = OA_MIN_MOTOR; id <= OA_MAX_MOTOR; id++) {
            oa_motor_t *m = &S.bus[b].m[id];
            if (!m->present) continue;
            int cal_ok = m->has_cal && m->pos >= m->tmin - 0.10f && m->pos <= m->tmax + 0.10f;
            if (!cal_ok) {
                m->has_cal = 0;
                m->tmin = m->pos - MAX_TRAVEL;
                m->tmax = m->pos + MAX_TRAVEL;
            }
            /* on a fresh start, leave the arm where it is: target = goal = current
             * pose (no motion). With the default symmetric window this puts the
             * slider handle in the middle; after a calibration the slider start is
             * set from the recorded initial angle instead (see run_calibration). */
            m->target = m->goal = m->pos;
        }
    snprintf(S.status, sizeof S.status, "initialized %d bus(es)", n);
    return n;
}

/* force MIT + enable + hold at current pose */
static void enable_all(void)
{
    for (int b = 0; b < S.nbus; b++) {
        if (g_fd[b] < 0) continue;
        for (int id = OA_MIN_MOTOR; id <= OA_MAX_MOTOR; id++) {
            if (!S.bus[b].m[id].present) continue;
            dm_frame_t f;
            dm_build_disable(&f, id); scan_send(g_fd[b], &f); sleep_ms(20);
            dm_build_write_reg_u32(&f, id, DM_RID_CTRL_MODE, 1u); scan_send(g_fd[b], &f); sleep_ms(30);
            dm_build_enable(&f, id); scan_send(g_fd[b], &f);
            S.bus[b].m[id].active = 1;
        }
    }
}
static void disable_all(void)
{
    for (int b = 0; b < S.nbus; b++) {
        if (g_fd[b] < 0) continue;
        for (int id = OA_MIN_MOTOR; id <= OA_MAX_MOTOR; id++) {
            dm_frame_t f; dm_build_disable(&f, id); scan_send(g_fd[b], &f);
            S.bus[b].m[id].active = 0;
        }
    }
}
/* set every present joint's commanded target+goal to its ACTUAL current pose, so
 * on (re)connect the arm holds where it is instead of snapping to a stale target. */
static void hold_current(void)
{
    for (int b = 0; b < S.nbus; b++) {
        if (g_fd[b] < 0) continue;
        for (int id = OA_MIN_MOTOR; id <= OA_MAX_MOTOR; id++) {
            if (!S.bus[b].m[id].present) continue;
            dm_limits_t lim; bus_limits(id, &lim);
            float p = S.bus[b].m[id].pos;
            read_pos_err(g_fd[b], id, &lim, &p, NULL);
            pthread_mutex_lock(&g_lock);
            S.bus[b].m[id].target = S.bus[b].m[id].goal = p;
            pthread_mutex_unlock(&g_lock);
        }
    }
}

/* Mark a joint present and initialize its control fields (first time it's seen). */
static void mark_present(int b, int id, float pos, int err)
{
    pthread_mutex_lock(&g_lock);
    oa_motor_t *m = &S.bus[b].m[id];
    if (!m->present) {
        m->present = 1; m->err = err;
        m->pos = m->target = m->goal = pos;
        m->tmin = pos - MAX_TRAVEL; m->tmax = pos + MAX_TRAVEL; m->has_cal = 0;
        joint_gains(id, S.gain_scale, &m->kp, &m->kd);
        snprintf(m->model, sizeof m->model, "%s", model_for(id));
        S.bus[b].nmotors++;
    } else {
        m->err = err;
    }
    pthread_mutex_unlock(&g_lock);
}

/* Presence probe -- zero torque, no motion. Robust enough to catch a flickering
 * / laggy joint (~30ms worst case for an absent id), but only one is probed per
 * loop tick so the control stream isn't disrupted. */
static int quick_probe(int fd, int id, const dm_limits_t *lim, float *pos, int *err)
{
    dm_frame_t f; dm_build_mit(&f, id, lim, 0, 0, 0, 0, 0);
    if (scan_send(fd, &f) < 0) return 0;
    for (int k = 0; k < 6; k++) {
        if (scan_recv(fd, &f, 5) == 1 && f.id == (uint32_t)(id + DM_FB_OFFSET)) {
            dm_state_t st;
            if (dm_parse_feedback(&f, lim, &st)) { *pos = st.pos; if (err) *err = st.err; return 1; }
        }
    }
    return 0;
}

/* Round-robin: probe ONE not-yet-present id per call so joints that came online
 * (or were flickering) after startup get discovered without stalling the loop. */
static void discover_one(void)
{
    static int cursor = 0;
    int span = (S.nbus > 0 ? S.nbus : 1) * OA_MAX_MOTOR;
    for (int n = 0; n < span; n++) {
        int idx = cursor; cursor = (cursor + 1) % span;
        int b = idx / OA_MAX_MOTOR, id = OA_MIN_MOTOR + (idx % OA_MAX_MOTOR);
        if (b >= S.nbus || g_fd[b] < 0 || S.bus[b].m[id].present) continue;
        dm_limits_t lim; bus_limits(id, &lim);
        float p; int e = 0;
        if (quick_probe(g_fd[b], id, &lim, &p, &e)) {
            mark_present(b, id, p, e);
            if (S.connected && !S.estopped) {   /* enable the new joint, holding here */
                dm_frame_t f;
                dm_build_disable(&f, id); scan_send(g_fd[b], &f);
                dm_build_write_reg_u32(&f, id, DM_RID_CTRL_MODE, 1u); scan_send(g_fd[b], &f);
                dm_build_enable(&f, id); scan_send(g_fd[b], &f);
                pthread_mutex_lock(&g_lock); S.bus[b].m[id].active = 1; pthread_mutex_unlock(&g_lock);
            }
        }
        return;   /* one probe per call */
    }
}

/* D-pad selects all joints J1..J8 (J8 = claw/gripper, driven like any joint). */
#define OA_MAX_SELECT OA_MAX_MOTOR

/* pick next/prev present joint id in [J1..J7] on a bus */
static int step_motor(int b, int cur, int dir)
{
    if (cur < OA_MIN_MOTOR) cur = OA_MIN_MOTOR;
    if (cur > OA_MAX_SELECT) cur = OA_MAX_SELECT;
    for (int id = cur + dir; id >= OA_MIN_MOTOR && id <= OA_MAX_SELECT; id += dir)
        if (S.bus[b].m[id].present) return id;
    return cur;   /* clamp at the ends (no wrap) */
}
static int first_present(int b)
{
    for (int id = OA_MIN_MOTOR; id <= OA_MAX_SELECT; id++) if (S.bus[b].m[id].present) return id;
    return 0;
}

/* Manual teach-cal: record an endpoint for the currently-selected joint. First
 * press stores one limit; second press sets the range [min,max] and saves. */
static void manual_mark(void)
{
    int b = S.sel_bus, id = S.sel_motor;
    if (b < 0 || b >= S.nbus || id < OA_MIN_MOTOR || id > OA_MAX_MOTOR) return;
    if (!S.bus[b].m[id].present) return;
    pthread_mutex_lock(&g_lock);
    float raw = S.bus[b].m[id].pos;                       /* record the actual pose */
    if (mc_count[b][id] == 1) {
        float e1 = mc_first[b][id], e2 = raw;
        float lo = e1 < e2 ? e1 : e2, hi = e1 < e2 ? e2 : e1;
        if (hi - lo < 0.05f) {   /* endpoints basically the same -> zero-width slider */
            snprintf(S.status, sizeof S.status, "%s J%d: endpoints too close (%.3f rad) -- jog further, press A again",
                     S.bus[b].iface, id, hi - lo);
            pthread_mutex_unlock(&g_lock);
            return;              /* stay at step 1, wait for a proper 2nd endpoint */
        }
        /* Range is exactly the two poses you jogged to -> covers the full defined
         * motion (span = |e2-e1|: 90 stays 90, 270 stays 270). */
        S.bus[b].m[id].tmin = lo; S.bus[b].m[id].tmax = hi; S.bus[b].m[id].has_cal = 1;
        S.bus[b].m[id].cal_step = 2;
        S.bus[b].m[id].target = raw;                      /* at an extreme now */
        S.bus[b].m[id].goal = (lo + hi) * 0.5f;           /* ease to middle (no corner pin) */
        mc_count[b][id] = 2;
        snprintf(S.status, sizeof S.status, "%s J%d set: span %.1f deg [%.3f, %.3f] rad -> centering",
                 S.bus[b].iface, id, (hi - lo) * 57.2958f, lo, hi);
        save_calib_locked();
        oa_log("MANUAL-CAL %s J%d: endpoint2 at %.4f -> range [%.4f, %.4f] span %.1f deg (saved)",
               S.bus[b].iface, id, raw, lo, hi, (hi - lo) * 57.2958f);
    } else {                                              /* 0 or 2 -> fresh capture */
        mc_first[b][id] = raw; mc_count[b][id] = 1; S.bus[b].m[id].cal_step = 1;
        snprintf(S.status, sizeof S.status, "%s J%d: endpoint 1 set -- jog to the other end, press A", S.bus[b].iface, id);
        oa_log("MANUAL-CAL %s J%d: endpoint1 at %.4f rad", S.bus[b].iface, id, raw);
    }
    pthread_mutex_unlock(&g_lock);
}

/* Re-probe one joint and calibrate it (bounded, no-hang). Returns 1 on success
 * and fills *out. Skips (returns 0) if the joint isn't answering now. */
static int cal_one(int b, int id, calib_result_t *out)
{
    dm_limits_t lim; bus_limits(id, &lim);
    float pp = 0; int found = 0;
    for (int t = 0; t < 3 && !found && !g_cal_abort && !g_stop; t++)
        found = read_pos_err(g_fd[b], id, &lim, &pp, NULL);
    if (!found) return 0;
    if (!S.bus[b].m[id].present) mark_present(b, id, pp, 0);
    int mode = (id <= 2) ? CAL_HANG : (id == 8) ? CAL_GRIPPER : CAL_CENTER;
    pthread_mutex_lock(&g_lock);
    snprintf(S.status, sizeof S.status, "calibrating %s J%d...", S.bus[b].iface, id);
    pthread_mutex_unlock(&g_lock);
    calib_result_t r;
    if (calib_run(g_fd[b], id, &lim, mode, &g_cal_abort, &r) && r.ok) {
        pthread_mutex_lock(&g_lock);
        S.bus[b].m[id].tmin = r.pmin; S.bus[b].m[id].tmax = r.pmax; S.bus[b].m[id].has_cal = 1;
        pthread_mutex_unlock(&g_lock);
        oa_log("AUTO-CAL %s J%d: ok range [%.4f, %.4f] span %.1f deg", S.bus[b].iface, id, r.pmin, r.pmax, r.span*57.2958f);
        if (out) *out = r;
        return 1;
    }
    oa_log("AUTO-CAL %s J%d: FAILED (%s)", S.bus[b].iface, id, r.msg);
    return 0;
}

/* Enable a joint and ramp it smoothly to `target`, leaving it enabled + holding. */
static void ramp_move(int b, int id, float target, double secs)
{
    dm_limits_t lim; bus_limits(id, &lim);
    dm_frame_t f;
    dm_build_disable(&f, id); scan_send(g_fd[b], &f); sleep_ms(20);
    dm_build_write_reg_u32(&f, id, DM_RID_CTRL_MODE, 1u); scan_send(g_fd[b], &f); sleep_ms(30);
    dm_build_enable(&f, id); scan_send(g_fd[b], &f);
    float kp, kd; joint_gains(id, S.gain_scale, &kp, &kd);
    float start = S.bus[b].m[id].pos;
    read_pos_err(g_fd[b], id, &lim, &start, NULL);
    double t0 = now_s();
    while (now_s() - t0 < secs && !g_cal_abort && !g_stop) {
        double el = now_s() - t0;
        float a = (el < secs * 0.8) ? (float)(el / (secs * 0.8)) : 1.0f;
        float cur = start + (target - start) * a;
        dm_build_mit(&f, id, &lim, cur, 0, kp, kd, 0); scan_send(g_fd[b], &f);
        dm_frame_t rf;
        if (scan_recv(g_fd[b], &rf, 1) == 1 && rf.id == (uint32_t)(id + DM_FB_OFFSET)) {
            dm_state_t s2; if (dm_parse_feedback(&rf, &lim, &s2)) S.bus[b].m[id].pos = s2.pos;
        }
        sleep_ms(1000 / CTRL_HZ);
    }
    pthread_mutex_lock(&g_lock);
    S.bus[b].m[id].target = S.bus[b].m[id].goal = target; S.bus[b].m[id].active = 1;
    pthread_mutex_unlock(&g_lock);
}

/* run calibration for every present joint on every bus (bounded, no-hang) */
static void run_calibration(void)
{
    int   cal_done[OA_NBUS][OA_MAX_MOTOR + 1];
    float cal_start[OA_NBUS][OA_MAX_MOTOR + 1];   /* pre-cal pose, new frame */
    memset(cal_done, 0, sizeof cal_done);

    g_cal_abort = 0;   /* cleared here; set by control_estop() / Ctrl-C to abort */
    pthread_mutex_lock(&g_lock); S.calibrating = 1; pthread_mutex_unlock(&g_lock);
    for (int b = 0; b < S.nbus && !g_stop && !g_cal_abort; b++) {
        if (g_fd[b] < 0) continue;
        calib_result_t r;

        /* Shoulder first: calibrate J2, extend it OUT (so the arm doesn't fold onto
         * itself), THEN calibrate the base J1 while holding J2 out there -- keeps
         * the arm clear/balanced so J1's sweep doesn't collide or flip it.
         * The "out" angle defaults to 80% toward J2's far stop; override in rad
         * with CAL_J2_PREP_RAD (per J2's calibrated frame). */
        float j2prep = 0; int didJ2 = 0;
        if (cal_one(b, 2, &r)) {
            didJ2 = 1; cal_done[b][2] = 1; cal_start[b][2] = r.start_pos;
            const char *e = getenv("CAL_J2_PREP_RAD");
            if (e && *e) j2prep = (float)atof(e);
            else j2prep = (fabsf(r.pmin) >= fabsf(r.pmax) ? r.pmin : r.pmax) * 0.8f;
            if (j2prep < r.pmin) j2prep = r.pmin;
            if (j2prep > r.pmax) j2prep = r.pmax;
        }
        if (!g_stop && !g_cal_abort) {
            int haveJ1 = 0;
            { dm_limits_t l1; bus_limits(1, &l1); float pp;
              for (int t = 0; t < 3 && !haveJ1 && !g_cal_abort; t++) haveJ1 = read_pos_err(g_fd[b], 1, &l1, &pp, NULL); }
            if (haveJ1 && didJ2) {
                pthread_mutex_lock(&g_lock);
                snprintf(S.status, sizeof S.status, "%s: extending J2 out (%.2f rad) before J1...", S.bus[b].iface, j2prep);
                pthread_mutex_unlock(&g_lock);
                ramp_move(b, 2, j2prep, 2.5);                /* J2 -> extended "out" pose */
                dm_limits_t l2; bus_limits(2, &l2);
                float kp2, kd2; joint_gains(2, S.gain_scale, &kp2, &kd2);
                calib_hold_companion(2, j2prep, &l2, kp2, kd2);  /* keep J2 out during J1 */
            }
            if (haveJ1 && cal_one(b, 1, &r)) { cal_done[b][1] = 1; cal_start[b][1] = r.start_pos; }
            calib_hold_clear();
        }

        /* the remaining joints, in order */
        for (int id = 3; id <= OA_MAX_MOTOR && !g_stop && !g_cal_abort; id++)
            if (cal_one(b, id, &r)) { cal_done[b][id] = 1; cal_start[b][id] = r.start_pos; }
    }

    /* E-STOP (or Ctrl-C) during calibration: leave every motor disabled and stop. */
    if (g_cal_abort || g_stop) {
        disable_all();
        pthread_mutex_lock(&g_lock);
        S.calibrating = 0; S.estopped = 1;
        snprintf(S.status, sizeof S.status, "E-STOP -- calibration aborted");
        pthread_mutex_unlock(&g_lock);
        return;
    }

    /* Gentle, coordinated return to each joint's ORIGINAL pose (recorded before
     * its sweep). Joints sag while others calibrate, so we re-enable, read the
     * actual pose, then ramp everything back over RETURN_SECS at a slow, uniform
     * rate -- smooth, not a jerk. Non-calibrated joints just hold where they are. */
    enable_all();
    pthread_mutex_lock(&g_lock); snprintf(S.status, sizeof S.status, "returning to start pose..."); pthread_mutex_unlock(&g_lock);
    float from[OA_NBUS][OA_MAX_MOTOR + 1], to[OA_NBUS][OA_MAX_MOTOR + 1];
    for (int b = 0; b < S.nbus; b++)
        for (int id = OA_MIN_MOTOR; id <= OA_MAX_MOTOR; id++) {
            if (!S.bus[b].m[id].present || g_fd[b] < 0) continue;
            dm_limits_t lim; bus_limits(id, &lim);
            float p = S.bus[b].m[id].pos;
            read_pos_err(g_fd[b], id, &lim, &p, NULL);
            from[b][id] = p;
            float t = p;
            if (cal_done[b][id]) {
                t = cal_start[b][id];
                if (t < S.bus[b].m[id].tmin) t = S.bus[b].m[id].tmin;
                if (t > S.bus[b].m[id].tmax) t = S.bus[b].m[id].tmax;
            }
            to[b][id] = t;
        }
    double t0 = now_s(), dur = RETURN_SECS;
    while (!g_stop && !g_cal_abort) {
        double el = now_s() - t0; if (el >= dur) break;
        float a = (float)(el / dur);
        for (int b = 0; b < S.nbus; b++) {
            if (g_fd[b] < 0) continue;
            for (int id = OA_MIN_MOTOR; id <= OA_MAX_MOTOR; id++) {
                if (!S.bus[b].m[id].present) continue;
                dm_limits_t lim; bus_limits(id, &lim);
                float cur = from[b][id] + (to[b][id] - from[b][id]) * a;
                dm_frame_t f; dm_build_mit(&f, id, &lim, cur, 0, S.bus[b].m[id].kp, S.bus[b].m[id].kd, 0);
                scan_send(g_fd[b], &f);
                dm_frame_t rf; float fbpos = cur;
                if (scan_recv(g_fd[b], &rf, 1) == 1 && rf.id == (uint32_t)(id + DM_FB_OFFSET)) {
                    dm_state_t s2; if (dm_parse_feedback(&rf, &lim, &s2)) fbpos = s2.pos;
                }
                /* keep target+goal on the live ramp position so the dashboard
                 * slider follows the return and ends aligned (no grab-jerk). */
                pthread_mutex_lock(&g_lock);
                S.bus[b].m[id].pos = fbpos;
                S.bus[b].m[id].target = S.bus[b].m[id].goal = cur;
                pthread_mutex_unlock(&g_lock);
            }
        }
        sleep_ms(1000 / CTRL_HZ);
    }
    if (g_cal_abort || g_stop) {   /* E-STOP during the return */
        disable_all();
        pthread_mutex_lock(&g_lock); S.calibrating = 0; S.estopped = 1;
        snprintf(S.status, sizeof S.status, "E-STOP -- calibration aborted"); pthread_mutex_unlock(&g_lock);
        return;
    }
    for (int b = 0; b < S.nbus; b++)
        for (int id = OA_MIN_MOTOR; id <= OA_MAX_MOTOR; id++)
            if (S.bus[b].m[id].present) {
                pthread_mutex_lock(&g_lock);
                S.bus[b].m[id].target = S.bus[b].m[id].goal = to[b][id];
                pthread_mutex_unlock(&g_lock);
            }

    pthread_mutex_lock(&g_lock);
    save_calib_locked();
    S.calibrating = 0;
    snprintf(S.status, sizeof S.status, "calibration done");
    pthread_mutex_unlock(&g_lock);
}

static void *control_loop(void *arg)
{
    (void)arg;
    gamepad_t pad; gamepad_init(&pad);
    /* Do NOT enable motors on start -- the arm stays limp until the operator
     * presses Connect in the dashboard. */

    double next = now_s();
    while (!g_stop) {
        /* ---- consume commands ---- */
        int do_estop, do_clear, do_calib; float scale;
        float jog[OA_NBUS][OA_MAX_MOTOR + 1];
        float tgt[OA_NBUS][OA_MAX_MOTOR + 1]; int tset[OA_NBUS][OA_MAX_MOTOR + 1];
        int sel_b, sel_m;
        int do_connect, do_disconnect, do_mstart, do_mstop, do_mmark;
        pthread_mutex_lock(&g_lock);
        do_estop = c_estop; c_estop = 0;
        do_clear = c_clear; c_clear = 0;
        do_calib = c_calib; c_calib = 0;
        do_connect = c_connect; c_connect = 0;
        do_disconnect = c_disconnect; c_disconnect = 0;
        do_mstart = c_manual_start; c_manual_start = 0;
        do_mstop = c_manual_stop; c_manual_stop = 0;
        do_mmark = c_manual_mark; c_manual_mark = 0;
        scale = c_gain_scale;
        memcpy(jog, c_jog, sizeof jog); memset(c_jog, 0, sizeof c_jog);
        memcpy(tgt, c_target, sizeof tgt);
        memcpy(tset, c_target_set, sizeof tset); memset(c_target_set, 0, sizeof c_target_set);
        sel_b = c_sel_bus; c_sel_bus = -1;
        sel_m = c_sel_motor; c_sel_motor = -1;
        pthread_mutex_unlock(&g_lock);

        if (do_connect && !S.connected && !S.estopped) {
            enable_all(); hold_current();
            pthread_mutex_lock(&g_lock); S.connected = 1; pthread_mutex_unlock(&g_lock);
            oa_log("CONNECT: motors enabled + holding");
        }
        if (do_disconnect && S.connected) {
            disable_all();
            pthread_mutex_lock(&g_lock); S.connected = 0; pthread_mutex_unlock(&g_lock);
            oa_log("DISCONNECT: motors limp");
        }
        if (do_estop)  { disable_all(); pthread_mutex_lock(&g_lock); S.estopped = 1; S.manual_cal = 0; snprintf(S.status,sizeof S.status,"E-STOP"); pthread_mutex_unlock(&g_lock); oa_log("E-STOP: all motors disabled"); }
        if (do_clear)  {
            int was; pthread_mutex_lock(&g_lock); S.estopped = 0; was = S.connected; pthread_mutex_unlock(&g_lock);
            oa_log("CLEAR E-STOP (was connected=%d)", was);
            if (was) { enable_all(); hold_current(); }   /* re-hold at actual pose (no snap) */
        }
        if (do_calib && S.connected && !S.estopped) { oa_log("AUTO-CALIBRATE: start"); run_calibration(); oa_log("AUTO-CALIBRATE: done"); }
        if (do_mstart && S.connected && !S.estopped) {
            pthread_mutex_lock(&g_lock); S.manual_cal = 1;
            memset(mc_count, 0, sizeof mc_count);     /* fresh endpoint capture */
            for (int b = 0; b < S.nbus; b++)
                for (int id = OA_MIN_MOTOR; id <= OA_MAX_MOTOR; id++) S.bus[b].m[id].cal_step = 0;
            snprintf(S.status, sizeof S.status, "MANUAL CAL: pick a joint (d-pad), jog to a limit, press A (x2)");
            pthread_mutex_unlock(&g_lock);
            oa_log("MANUAL-CAL: start");
        }
        if (do_mstop) { pthread_mutex_lock(&g_lock); S.manual_cal = 0; pthread_mutex_unlock(&g_lock); oa_log("MANUAL-CAL: exit"); }
        if (do_mmark && S.manual_cal && S.connected && !S.estopped) manual_mark();

        /* refresh gains if scale changed */
        for (int b = 0; b < S.nbus; b++)
            for (int id = OA_MIN_MOTOR; id <= OA_MAX_MOTOR; id++)
                if (S.bus[b].m[id].present) joint_gains(id, scale, &S.bus[b].m[id].kp, &S.bus[b].m[id].kd);

        /* ---- discover joints that appeared after startup (J1/J8/flicker) ----
         * one absent id probed per couple of ticks; found joints stick in the
         * roster, so /api/status changes and the dashboard rebuilds to show them. */
        if (!S.calibrating) { static int disc = 0; if (++disc >= 2) { disc = 0; discover_one(); } }

        /* ---- gamepad ---- */
        if (!pad.fd || pad.fd < 0) gamepad_detect(&pad);
        int pad_on = gamepad_poll(&pad);

        int sbus = S.sel_bus, smot = S.sel_motor;
        if (smot == 0) smot = first_present(sbus);

        if (pad_on && S.connected && !S.estopped) {
            if (pad.btn_start) control_estop();
            if (pad.dpad_left_edge)  { sbus = (sbus + 1) % S.nbus; smot = first_present(sbus); }
            if (pad.dpad_right_edge) { sbus = (sbus + 1) % S.nbus; smot = first_present(sbus); }
            /* up -> toward J1 (J7->J1), down -> toward J7 (J1->J7) */
            if (pad.dpad_up_edge)    smot = step_motor(sbus, smot ? smot : first_present(sbus), -1);
            if (pad.dpad_down_edge)  smot = step_motor(sbus, smot ? smot : first_present(sbus), +1);
            /* in manual cal, A marks an endpoint for the selected joint */
            if (S.manual_cal && pad.btn_a_edge) { S.sel_bus = sbus; S.sel_motor = smot; manual_mark(); }
            if (smot >= OA_MIN_MOTOR && smot <= OA_MAX_SELECT && fabsf(pad.stick_x) > 0.0f) {
                oa_motor_t *m = &S.bus[sbus].m[smot];
                m->goal += pad.stick_x * SLIDER_SPEED / CTRL_HZ;
                if (S.manual_cal) {                 /* free jog to find the limits */
                    dm_limits_t lim; bus_limits(smot, &lim);
                    if (m->goal < -lim.p_max) m->goal = -lim.p_max;
                    if (m->goal >  lim.p_max) m->goal =  lim.p_max;
                } else {                            /* normal: confined to calibrated range */
                    if (m->goal < m->tmin) m->goal = m->tmin;
                    if (m->goal > m->tmax) m->goal = m->tmax;
                }
            }
        }

        /* apply explicit selection / jog commands (from web UI) */
        if (sel_b >= 0 && sel_b < S.nbus) { sbus = sel_b; if (sel_m >= OA_MIN_MOTOR) smot = sel_m; }
        for (int b = 0; b < S.nbus; b++)
            for (int id = OA_MIN_MOTOR; id <= OA_MAX_MOTOR; id++)
                if (jog[b][id] != 0.0f && S.bus[b].m[id].present && S.connected && !S.estopped) {
                    oa_motor_t *m = &S.bus[b].m[id];
                    m->goal += jog[b][id];
                    if (m->goal < m->tmin) m->goal = m->tmin;
                    if (m->goal > m->tmax) m->goal = m->tmax;
                }
        /* absolute goal from the dashboard sliders (target eases toward it) */
        for (int b = 0; b < S.nbus; b++)
            for (int id = OA_MIN_MOTOR; id <= OA_MAX_MOTOR; id++)
                if (tset[b][id] && S.bus[b].m[id].present && S.connected && !S.estopped) {
                    oa_motor_t *m = &S.bus[b].m[id];
                    float t = tgt[b][id];
                    if (S.manual_cal) {          /* free jog to find limits */
                        dm_limits_t lim; bus_limits(id, &lim);
                        if (t < -lim.p_max) t = -lim.p_max;
                        if (t >  lim.p_max) t =  lim.p_max;
                    } else {                     /* normal: confine to calibrated range */
                        if (t < m->tmin) t = m->tmin;
                        if (t > m->tmax) t = m->tmax;
                    }
                    m->goal = t;
                }

        /* ---- CONNECTED: drive every active motor (hold/ease to target) ----
         * NOT connected (or estopped): send only a zero-torque poke so the arm
         * stays limp but the dashboard still shows live joint positions. */
        int drive = S.connected && !S.estopped;
        for (int b = 0; b < S.nbus; b++) {
            if (g_fd[b] < 0) continue;
            for (int id = OA_MIN_MOTOR; id <= OA_MAX_MOTOR; id++) {
                oa_motor_t *m = &S.bus[b].m[id];
                if (!m->present) continue;
                dm_limits_t lim; bus_limits(id, &lim);
                dm_frame_t f;
                if (drive && m->active) {
                    float step = SLEW_RATE / CTRL_HZ;          /* ease target->goal */
                    if (m->target < m->goal) m->target = fminf(m->goal, m->target + step);
                    else if (m->target > m->goal) m->target = fmaxf(m->goal, m->target - step);
                    dm_build_mit(&f, id, &lim, m->target, 0, m->kp, m->kd, 0);
                } else {
                    dm_build_mit(&f, id, &lim, 0, 0, 0, 0, 0);  /* zero-torque telemetry */
                    m->target = m->goal = m->pos;               /* track limp pose */
                }
                scan_send(g_fd[b], &f);
                dm_frame_t rf;
                if (scan_recv(g_fd[b], &rf, 1) == 1 && rf.id == (uint32_t)(id + DM_FB_OFFSET)) {
                    dm_state_t st;
                    if (dm_parse_feedback(&rf, &lim, &st)) {
                        m->pos = st.pos; m->vel = st.vel; m->tau = st.tau; m->err = st.err;
                        if (drive && m->active && st.err >= 8) { dm_frame_t d; dm_build_disable(&d, id); scan_send(g_fd[b], &d); m->active = 0; }
                    }
                }
            }
        }

        /* ---- periodic motor-state log (~1 Hz): pos / target / goal / err ---- */
        { static int lg = 0;
          if (++lg >= CTRL_HZ) { lg = 0;
              for (int b = 0; b < S.nbus; b++) {
                  char line[300]; int n = snprintf(line, sizeof line, "MOTORS %s%s:", S.bus[b].iface,
                                                    S.manual_cal ? " [manual-cal]" : (S.connected ? "" : " [limp]"));
                  int any = 0;
                  for (int id = OA_MIN_MOTOR; id <= OA_MAX_MOTOR && n < (int)sizeof line - 40; id++)
                      if (S.bus[b].m[id].present) {
                          any = 1; oa_motor_t *m = &S.bus[b].m[id];
                          n += snprintf(line + n, sizeof line - n, " J%d(p%.3f t%.3f g%.3f e%d)",
                                        id, m->pos, m->target, m->goal, m->err);
                      }
                  if (any) oa_log("%s", line);
              }
          }
        }

        /* ---- publish snapshot ---- */
        pthread_mutex_lock(&g_lock);
        S.sel_bus = sbus; S.sel_motor = smot;
        S.pad_connected = pad_on;
        S.gain_scale = scale;
        if (pad_on) {
            snprintf(S.pad_name, sizeof S.pad_name, "%s", pad.name);
            S.pad_stick_x = pad.stick_x;
            S.pad_dpad_x = pad.dpad_x; S.pad_dpad_y = pad.dpad_y;
            S.pad_btn_a = pad.btn_a;   S.pad_btn_start = pad.btn_start;
        } else {
            S.pad_name[0] = 0;
            S.pad_stick_x = 0; S.pad_dpad_x = S.pad_dpad_y = 0;
            S.pad_btn_a = S.pad_btn_start = 0;
        }
        if (!S.calibrating && !S.estopped && !S.manual_cal)
            snprintf(S.status, sizeof S.status, "%s",
                     !S.connected ? "not connected — press Connect to enable motors"
                                  : (pad_on ? "connected — scheme-1 gamepad control"
                                            : "connected — holding (sliders/gamepad ready)"));
        pthread_mutex_unlock(&g_lock);

        /* ---- pace to CTRL_HZ ---- */
        next += 1.0 / CTRL_HZ;
        double dt = next - now_s();
        if (dt > 0) sleep_ms((int)(dt * 1000));
        else next = now_s();
    }
    gamepad_close(&pad);
    disable_all();
    return NULL;
}

int control_start(void)
{
    g_stop = 0; g_run = 1;
    return pthread_create(&g_thread, NULL, control_loop, NULL) == 0 ? 0 : -1;
}

void control_stop(void)
{
    if (!g_run) return;
    g_stop = 1; g_cal_abort = 1;   /* also break out of a running calibration */
    pthread_join(g_thread, NULL);
    for (int b = 0; b < S.nbus; b++) if (g_fd[b] >= 0) scan_close(g_fd[b]);
    g_run = 0;
}

void control_get_state(oa_state_t *out)
{ pthread_mutex_lock(&g_lock); *out = S; pthread_mutex_unlock(&g_lock); }

void control_connect(void)      { pthread_mutex_lock(&g_lock); c_connect = 1; pthread_mutex_unlock(&g_lock); }
void control_disconnect(void)   { pthread_mutex_lock(&g_lock); c_disconnect = 1; pthread_mutex_unlock(&g_lock); }
void control_estop(void)        { g_cal_abort = 1; pthread_mutex_lock(&g_lock); c_estop = 1; pthread_mutex_unlock(&g_lock); }
void control_clear_estop(void)  { pthread_mutex_lock(&g_lock); c_clear = 1; pthread_mutex_unlock(&g_lock); }
void control_request_calibration(void){ pthread_mutex_lock(&g_lock); c_calib = 1; pthread_mutex_unlock(&g_lock); }
void control_manual_start(void){ pthread_mutex_lock(&g_lock); c_manual_start = 1; pthread_mutex_unlock(&g_lock); }
void control_manual_stop(void){ pthread_mutex_lock(&g_lock); c_manual_stop = 1; pthread_mutex_unlock(&g_lock); }
void control_manual_mark(void){ pthread_mutex_lock(&g_lock); c_manual_mark = 1; pthread_mutex_unlock(&g_lock); }
void control_set_gain_scale(float s){ if (s>0 && s<=5) { pthread_mutex_lock(&g_lock); c_gain_scale = s; pthread_mutex_unlock(&g_lock); } }
void control_select(int bus, int motor){ pthread_mutex_lock(&g_lock); c_sel_bus = bus; c_sel_motor = motor; pthread_mutex_unlock(&g_lock); }
void control_jog(int bus, int motor, float delta)
{
    if (bus < 0 || bus >= OA_NBUS || motor < OA_MIN_MOTOR || motor > OA_MAX_MOTOR) return;
    pthread_mutex_lock(&g_lock); c_jog[bus][motor] += delta; pthread_mutex_unlock(&g_lock);
}
void control_set_target(int bus, int motor, float pos)
{
    if (bus < 0 || bus >= OA_NBUS || motor < OA_MIN_MOTOR || motor > OA_MAX_MOTOR) return;
    pthread_mutex_lock(&g_lock);
    c_target[bus][motor] = pos; c_target_set[bus][motor] = 1;
    pthread_mutex_unlock(&g_lock);
}
