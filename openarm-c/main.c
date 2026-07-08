/* main.c - OpenArm-C application entry point.
 *
 * Brings up the control engine over classic CAN, starts the HTTP dashboard, and
 * auto-detects a gamepad. While a controller is connected the control engine
 * runs Control Scheme 1 automatically; the dashboard shows live state and offers
 * manual jog/calibrate/e-stop when no pad is present.
 *
 *   openarm [--port N] [--web DIR] [can0 can1 ...]
 *
 * CAN interfaces default to every can* the kernel currently exposes. Bring the
 * buses up first (needs sudo): ./canup.sh can0 1000000
 */
#define _GNU_SOURCE
#include "control.h"
#include "httpd.h"
#include "damiao.h"
#include "socketcan.h"
#include "calib.h"
#include "log.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <dirent.h>
#include <time.h>

static double nowf(void)
{ struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t); return t.tv_sec + t.tv_nsec*1e-9; }
static void naps(double s)
{ struct timespec t = {(time_t)s, (long)((s-(long)s)*1e9)}; nanosleep(&t, NULL); }

/* Single-joint ramped move: force MIT, enable, ramp to target over ~1.5s, hold,
 * disable. Only this joint is energized. kp/kd default to the model tier. */
static int do_move(const char *iface, int id, float target, float kp, float kd)
{
    if (kp <= 0) { if (id<=2){kp=40;kd=2.0f;} else if (id<=4){kp=22;kd=1.2f;} else {kp=12;kd=0.6f;} }
    if (kd > 2.5f) kd = 2.5f;                       /* anti-vibration cap */
    const char *model = (id<=2)?"8009":(id<=4)?"4340":"4310";
    dm_limits_t lim; dm_limits_for_model(model, &lim);
    int fd = scan_open(iface);
    if (fd < 0) { fprintf(stderr, "cannot open %s\n", iface); return 1; }

    dm_frame_t f; dm_state_t st; memset(&st, 0, sizeof st);
    /* force MIT (valid only while disabled) */
    dm_build_disable(&f, id); scan_send(fd, &f); naps(0.03);
    dm_build_write_reg_u32(&f, id, DM_RID_CTRL_MODE, 1u); scan_send(fd, &f); naps(0.05);
    /* read current pos */
    float start = target;
    dm_build_mit(&f, id, &lim, 0,0,0,0,0); scan_send(fd, &f);
    for (int k = 0; k < 12; k++)
        if (scan_recv(fd, &f, 20)==1 && f.id==(uint32_t)(id+DM_FB_OFFSET) && dm_parse_feedback(&f,&lim,&st)) { start = st.pos; break; }
    printf("move %s J%d: %+.3f -> %+.3f rad (model %s, kp=%.0f kd=%.1f)\n",
           iface, id, start, target, model, kp, kd);
    dm_build_enable(&f, id); scan_send(fd, &f);

    double t0 = nowf();
    while (nowf() - t0 < 3.0) {
        double el = nowf() - t0;
        float a = el < 1.5 ? (float)(el/1.5) : 1.0f;
        float cur = start + (target - start) * a;
        dm_build_mit(&f, id, &lim, cur, 0, kp, kd, 0);
        if (scan_send(fd, &f) < 0) { printf("CAN send failed (bus dropped?)\n"); break; }
        if (scan_recv(fd, &f, 5)==1 && f.id==(uint32_t)(id+DM_FB_OFFSET) && dm_parse_feedback(&f,&lim,&st)) {
            if (st.err >= 8) { printf("FAULT: %s -- stopping\n", dm_status_text(st.err)); break; }
        }
        naps(0.02);
    }
    printf("  reached pos=%+.3f  tau=%+.2f  %s\n", st.pos, st.tau, dm_status_text(st.err));
    dm_build_disable(&f, id); scan_send(fd, &f);
    scan_close(fd);
    return 0;
}

static volatile sig_atomic_t g_quit = 0;
static volatile int          g_abort = 0;   /* int* for calib_run() */
static void on_sig(int s) { (void)s; g_quit = 1; g_abort = 1; }

#define CALIB_FILE "arm_calib.txt"
static const char *cal_model(int id){ return id<=2?"8009":id<=4?"4340":"4310"; }
static int cal_mode(int id){ return id<=2?CAL_HANG : (id==8?CAL_GRIPPER:CAL_CENTER); }

/* zero-torque presence probe (no motion). Flushes stale feedback left in the
 * socket by the previous joint's calibration first, then pokes and waits up to
 * ~100ms for THIS joint's reply (id-matched). */
static int probe_present(int fd, int id)
{
    dm_limits_t lim; dm_limits_for_model(cal_model(id), &lim);
    dm_frame_t f;
    while (scan_recv(fd, &f, 0) == 1) { }        /* drain buffered frames */
    dm_build_mit(&f, id, &lim, 0,0,0,0,0);
    if (scan_send(fd, &f) < 0) return 0;
    for (int k = 0; k < 20; k++) {
        int r = scan_recv(fd, &f, 5);
        if (r == 1 && f.id == (uint32_t)(id + DM_FB_OFFSET)) return 1;
        if (r < 0) return 0;
    }
    return 0;
}

/* Calibrate every present joint on the given interfaces. Skips absent/dropped
 * joints (never hangs), bounded per-joint, Ctrl-C aborts. Merges results into
 * arm_calib.txt (preserving interfaces not calibrated this run). */
static int do_calibrate(const char *ifaces[], int n)
{
    signal(SIGINT, on_sig); signal(SIGTERM, on_sig);
    struct { char ifc[16]; int id; float lo, hi; } res[OA_NBUS * 8];
    int nres = 0, ok = 0, failed = 0, skipped = 0;

    for (int b = 0; b < n && !g_abort; b++) {
        int fd = scan_open(ifaces[b]);
        if (fd < 0) { printf("!! cannot open %s\n", ifaces[b]); continue; }
        printf("#### %s ####\n", ifaces[b]);
        for (int id = OA_MIN_MOTOR; id <= OA_MAX_MOTOR && !g_abort; id++) {
            if (!probe_present(fd, id)) { skipped++; continue; }   /* absent -> skip (no hang) */
            dm_limits_t lim; dm_limits_for_model(cal_model(id), &lim);
            const char *mn = cal_mode(id)==CAL_HANG?"hang":cal_mode(id)==CAL_GRIPPER?"gripper":"center";
            printf("  J%d (%s, %s) ...\n", id, cal_model(id), mn);
            calib_result_t r;
            if (calib_run(fd, id, &lim, cal_mode(id), &g_abort, &r) && r.ok) {
                snprintf(res[nres].ifc, sizeof res[nres].ifc, "%s", ifaces[b]);
                res[nres].id = id; res[nres].lo = r.pmin; res[nres].hi = r.pmax; nres++;
                printf("     ok: span=%.3f range[%.3f, %.3f]\n", r.span, r.pmin, r.pmax); ok++;
            } else { printf("     FAILED: %s\n", r.msg); failed++; }
        }
        scan_close(fd);
    }

    /* merge-save: keep lines for interfaces we didn't calibrate, then our results */
    char kept[8192]; size_t kl = 0; kept[0] = 0;
    FILE *rf = fopen(CALIB_FILE, "r");
    if (rf) {
        char ln[160];
        while (fgets(ln, sizeof ln, rf)) {
            char ifc[32]; if (sscanf(ln, "%31s", ifc) != 1) continue;
            int mine = 0; for (int b = 0; b < n; b++) if (!strcmp(ifc, ifaces[b])) mine = 1;
            if (!mine) { size_t l = strlen(ln); if (kl + l < sizeof kept) { memcpy(kept+kl, ln, l); kl += l; } }
        }
        fclose(rf);
    }
    FILE *wf = fopen(CALIB_FILE, "w");
    if (wf) {
        fwrite(kept, 1, kl, wf);
        for (int i = 0; i < nres; i++)
            fprintf(wf, "%s %d %.5f %.5f\n", res[i].ifc, res[i].id, res[i].lo, res[i].hi);
        fclose(wf);
    }
    printf("== calibrated %d, failed %d, skipped %d -> %s ==\n", ok, failed, skipped, CALIB_FILE);
    return 0;
}

static int list_can(const char *out[], int max)
{
    DIR *d = opendir("/sys/class/net");
    if (!d) return 0;
    int n = 0; struct dirent *e;
    static char names[OA_NBUS][16];
    while ((e = readdir(d)) && n < max && n < OA_NBUS) {
        if (strncmp(e->d_name, "can", 3) != 0) continue;
        snprintf(names[n], sizeof names[n], "%.15s", e->d_name);
        out[n] = names[n]; n++;
    }
    closedir(d);
    return n;
}

int main(int argc, char **argv)
{
    int port = 8080;
    const char *web = "web";
    int scan_only = 0, calib_only = 0;
    const char *ifaces[OA_NBUS]; int nif = 0;

    /* --move IFACE ID TARGET [KP KD] : move one joint and exit (handled early). */
    for (int i = 1; i < argc; i++)
        if (!strcmp(argv[i], "--move")) {
            if (i + 3 >= argc) { fprintf(stderr, "usage: --move IFACE ID TARGET [KP KD]\n"); return 2; }
            float kp = (i+4 < argc) ? (float)atof(argv[i+4]) : 0;
            float kd = (i+5 < argc) ? (float)atof(argv[i+5]) : 0;
            return do_move(argv[i+1], atoi(argv[i+2]), (float)atof(argv[i+3]), kp, kd);
        }

    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--port") && i + 1 < argc) port = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--web") && i + 1 < argc) web = argv[++i];
        else if (!strcmp(argv[i], "--scan")) scan_only = 1;
        else if (!strcmp(argv[i], "--calibrate")) calib_only = 1;
        else if (!strncmp(argv[i], "can", 3) && nif < OA_NBUS) ifaces[nif++] = argv[i];
        else { fprintf(stderr, "usage: %s [--port N] [--web DIR] [--scan|--calibrate] [can0 can1]\n", argv[0]); return 2; }
    }
    if (nif == 0) nif = list_can(ifaces, OA_NBUS);
    if (nif == 0) { fprintf(stderr, "No can* interface found. Bring one up: sudo ./canup.sh can0 1000000\n"); return 1; }

    if (calib_only) return do_calibrate(ifaces, nif);

    if (scan_only) {
        /* non-energizing: control_init probes with zero-torque frames only. */
        control_init(ifaces, nif);
        oa_state_t s; control_get_state(&s);
        for (int b = 0; b < s.nbus; b++) {
            printf("%s: %d motor(s)\n", s.bus[b].iface, s.bus[b].nmotors);
            for (int id = OA_MIN_MOTOR; id <= OA_MAX_MOTOR; id++)
                if (s.bus[b].m[id].present) {
                    oa_motor_t *m = &s.bus[b].m[id];
                    printf("   J%d  model~%s  pos=%+.3f rad  status=%s%s%s\n", id, m->model, m->pos,
                           dm_status_text(m->err), m->err >= 8 ? " <-- FAULT" : "",
                           m->has_cal ? "  (calibrated)" : "");
                }
        }
        return 0;
    }

    signal(SIGINT, on_sig);
    signal(SIGTERM, on_sig);

    oa_log_open("openarm.log");
    printf("OpenArm-C: interfaces =");
    for (int i = 0; i < nif; i++) printf(" %s", ifaces[i]);
    printf("\n");
    { char line[128] = "startup ifaces:"; for (int i=0;i<nif;i++){ strncat(line," ",sizeof line-strlen(line)-1); strncat(line,ifaces[i],sizeof line-strlen(line)-1);} oa_log("%s port=%d", line, port); }

    control_init(ifaces, nif);
    if (control_start() != 0) { fprintf(stderr, "control thread failed to start\n"); return 1; }
    if (httpd_start(port, web) != 0) { fprintf(stderr, "HTTP server failed on port %d\n", port); control_stop(); return 1; }

    printf("Dashboard: http://0.0.0.0:%d   (Ctrl-C to stop)\n", port);
    printf("Plug in a controller and it takes over with Control Scheme 1 automatically.\n");

    while (!g_quit) pause();

    printf("\nstopping...\n");
    oa_log("shutdown requested");
    httpd_stop();
    control_stop();
    oa_log("all motors disabled; exit");
    oa_log_close();
    printf("all motors disabled. bye.\n");
    return 0;
}
