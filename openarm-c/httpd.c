/* httpd.c - minimal HTTP/1.1 server (blocking accept loop in a thread). */
#define _GNU_SOURCE
#include "httpd.h"
#include "control.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <pthread.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>

static int        g_listen = -1;
static int        g_port;
static char       g_webroot[256];
static pthread_t  g_thr;
static volatile int g_stop;

/* ---- helpers ---- */
static void send_all(int fd, const char *buf, size_t n)
{
    size_t off = 0;
    while (off < n) {
        ssize_t w = write(fd, buf + off, n - off);
        if (w <= 0) { if (errno == EINTR) continue; break; }
        off += (size_t)w;
    }
}

static void respond(int fd, int code, const char *status, const char *ctype,
                    const char *body, size_t blen)
{
    char hdr[512];
    int h = snprintf(hdr, sizeof hdr,
        "HTTP/1.1 %d %s\r\n"
        "Content-Type: %s\r\n"
        "Content-Length: %zu\r\n"
        "Access-Control-Allow-Origin: *\r\n"
        "Connection: close\r\n\r\n",
        code, status, ctype, blen);
    send_all(fd, hdr, (size_t)h);
    if (body && blen) send_all(fd, body, blen);
}

/* Get an int/float query param: /path?bus=1&motor=3 */
static int qparam_int(const char *q, const char *key, int def)
{
    if (!q) return def;
    char pat[32]; snprintf(pat, sizeof pat, "%s=", key);
    const char *p = strstr(q, pat);
    if (!p) return def;
    return atoi(p + strlen(pat));
}
static float qparam_flt(const char *q, const char *key, float def)
{
    if (!q) return def;
    char pat[32]; snprintf(pat, sizeof pat, "%s=", key);
    const char *p = strstr(q, pat);
    if (!p) return def;
    return (float)atof(p + strlen(pat));
}

/* ---- JSON status ---- */
static void build_status(char *out, size_t cap)
{
    oa_state_t s; control_get_state(&s);
    int n = snprintf(out, cap,
        "{\"pad_connected\":%d,\"pad_name\":\"%s\",\"sel_bus\":%d,\"sel_motor\":%d,"
        "\"connected\":%d,\"estopped\":%d,\"calibrating\":%d,\"manual_cal\":%d,\"gain_scale\":%.2f,\"status\":\"%s\","
        "\"pad\":{\"stick_x\":%.3f,\"rstick_x\":%.3f,\"rstick_y\":%.3f,\"dpad_x\":%d,\"dpad_y\":%d,"
        "\"a\":%d,\"start\":%d,\"l1\":%d,\"r1\":%d,\"l2\":%.3f,\"r2\":%.3f},"
        "\"rover\":{\"present\":%d,\"enabled\":%d,\"estop\":%d,\"lin\":%.3f,\"ang\":%.3f,\"voltage\":%.1f},"
        "\"lift\":{\"present\":%d,\"enabled\":%d,\"vel\":%d},"
        "\"buses\":[",
        s.pad_connected, s.pad_name, s.sel_bus, s.sel_motor,
        s.connected, s.estopped, s.calibrating, s.manual_cal, s.gain_scale, s.status,
        s.pad_stick_x, s.pad_rstick_x, s.pad_rstick_y, s.pad_dpad_x, s.pad_dpad_y,
        s.pad_btn_a, s.pad_btn_start, s.pad_btn_l1, s.pad_btn_r1, s.pad_l2, s.pad_r2,
        s.ranger_present, s.ranger_enabled, s.ranger_estop, s.ranger_lin, s.ranger_ang, s.ranger_voltage,
        s.ds2c_present, s.ds2c_enabled, s.ds2c_vel);
    for (int b = 0; b < s.nbus; b++) {
        n += snprintf(out + n, cap - n, "%s{\"iface\":\"%s\",\"nmotors\":%d,\"motors\":[",
                      b ? "," : "", s.bus[b].iface, s.bus[b].nmotors);
        int first = 1;
        for (int id = OA_MIN_MOTOR; id <= OA_MAX_MOTOR; id++) {
            oa_motor_t *m = &s.bus[b].m[id];
            if (!m->present) continue;
            n += snprintf(out + n, cap - n,
                "%s{\"id\":%d,\"model\":\"%s\",\"active\":%d,\"err\":%d,"
                "\"pos\":%.4f,\"vel\":%.3f,\"tau\":%.3f,\"target\":%.4f,\"goal\":%.4f,"
                "\"tmin\":%.4f,\"tmax\":%.4f,\"kp\":%.1f,\"kd\":%.2f,\"cal\":%d,\"cal_step\":%d}",
                first ? "" : ",", id, m->model, m->active, m->err,
                m->pos, m->vel, m->tau, m->target, m->goal, m->tmin, m->tmax, m->kp, m->kd, m->has_cal, m->cal_step);
            first = 0;
        }
        n += snprintf(out + n, cap - n, "]}");
    }
    snprintf(out + n, cap - n, "]}");
}

/* ---- static file ---- */
static const char *mime(const char *path)
{
    const char *dot = strrchr(path, '.');
    if (!dot) return "application/octet-stream";
    if (!strcmp(dot, ".html")) return "text/html; charset=utf-8";
    if (!strcmp(dot, ".js"))   return "application/javascript";
    if (!strcmp(dot, ".css"))  return "text/css";
    return "text/plain";
}

static void serve_file(int fd, const char *relpath)
{
    /* prevent path traversal */
    if (strstr(relpath, "..")) { respond(fd, 403, "Forbidden", "text/plain", "no", 2); return; }
    char full[512];
    snprintf(full, sizeof full, "%s/%s", g_webroot,
             (!strcmp(relpath, "/") || !relpath[0]) ? "index.html" : relpath + (relpath[0] == '/' ? 1 : 0));
    int f = open(full, O_RDONLY);
    if (f < 0) { respond(fd, 404, "Not Found", "text/plain", "not found", 9); return; }
    static char buf[65536];
    ssize_t rd = read(f, buf, sizeof buf);
    close(f);
    if (rd < 0) rd = 0;
    respond(fd, 200, "OK", mime(full), buf, (size_t)rd);
}

static void handle(int fd)
{
    char req[4096];
    ssize_t n = read(fd, req, sizeof req - 1);
    if (n <= 0) return;
    req[n] = 0;

    char method[8] = {0}, path[512] = {0};
    if (sscanf(req, "%7s %511s", method, path) != 2) return;

    char *query = strchr(path, '?');
    if (query) *query++ = 0;

    char ok[] = "{\"ok\":true}";
    if (!strcmp(path, "/api/status")) {
        static char js[8192]; build_status(js, sizeof js);
        respond(fd, 200, "OK", "application/json", js, strlen(js));
    } else if (!strcmp(path, "/api/connect")) {
        control_connect(); respond(fd, 200, "OK", "application/json", ok, strlen(ok));
    } else if (!strcmp(path, "/api/disconnect")) {
        control_disconnect(); respond(fd, 200, "OK", "application/json", ok, strlen(ok));
    } else if (!strcmp(path, "/api/estop")) {
        control_estop(); respond(fd, 200, "OK", "application/json", ok, strlen(ok));
    } else if (!strcmp(path, "/api/clear")) {
        control_clear_estop(); respond(fd, 200, "OK", "application/json", ok, strlen(ok));
    } else if (!strcmp(path, "/api/calibrate")) {
        control_request_calibration(); respond(fd, 200, "OK", "application/json", ok, strlen(ok));
    } else if (!strcmp(path, "/api/manual_start")) {
        control_manual_start(); respond(fd, 200, "OK", "application/json", ok, strlen(ok));
    } else if (!strcmp(path, "/api/manual_stop")) {
        control_manual_stop(); respond(fd, 200, "OK", "application/json", ok, strlen(ok));
    } else if (!strcmp(path, "/api/manual_mark")) {
        control_manual_mark(); respond(fd, 200, "OK", "application/json", ok, strlen(ok));
    } else if (!strcmp(path, "/api/jog")) {
        control_jog(qparam_int(query,"bus",0), qparam_int(query,"motor",0), qparam_flt(query,"delta",0));
        respond(fd, 200, "OK", "application/json", ok, strlen(ok));
    } else if (!strcmp(path, "/api/target")) {
        control_set_target(qparam_int(query,"bus",0), qparam_int(query,"motor",0), qparam_flt(query,"pos",0));
        respond(fd, 200, "OK", "application/json", ok, strlen(ok));
    } else if (!strcmp(path, "/api/select")) {
        control_select(qparam_int(query,"bus",0), qparam_int(query,"motor",0));
        respond(fd, 200, "OK", "application/json", ok, strlen(ok));
    } else if (!strcmp(path, "/api/gain")) {
        control_set_gain_scale(qparam_flt(query,"scale",1.0f));
        respond(fd, 200, "OK", "application/json", ok, strlen(ok));
    } else if (!strcmp(path, "/api/rover")) {
        control_set_web_rover(qparam_flt(query,"lin",0), qparam_flt(query,"ang",0));
        respond(fd, 200, "OK", "application/json", ok, strlen(ok));
    } else if (!strcmp(path, "/api/lift")) {
        control_set_web_lift(qparam_flt(query,"vel",0));
        respond(fd, 200, "OK", "application/json", ok, strlen(ok));
    } else if (!strcmp(path, "/api/log")) {
        int lf = open("openarm.log", O_RDONLY);
        if (lf < 0) { respond(fd, 200, "OK", "text/plain", "(no log yet)", 12); }
        else {
            static char lb[131072];
            off_t sz = lseek(lf, 0, SEEK_END);
            off_t off = (sz > (off_t)sizeof lb) ? sz - (off_t)sizeof lb : 0;  /* tail */
            lseek(lf, off, SEEK_SET);
            ssize_t rd = read(lf, lb, sizeof lb); close(lf);
            if (rd < 0) rd = 0;
            respond(fd, 200, "OK", "text/plain; charset=utf-8", lb, (size_t)rd);
        }
    } else {
        serve_file(fd, path);
    }
}

static void *accept_loop(void *arg)
{
    (void)arg;
    while (!g_stop) {
        struct sockaddr_in cli; socklen_t cl = sizeof cli;
        int c = accept(g_listen, (struct sockaddr *)&cli, &cl);
        if (c < 0) { if (errno == EINTR) continue; if (g_stop) break; continue; }
        handle(c);
        close(c);
    }
    return NULL;
}

int httpd_start(int port, const char *webroot)
{
    g_port = port;
    snprintf(g_webroot, sizeof g_webroot, "%s", webroot);
    g_listen = socket(AF_INET, SOCK_STREAM, 0);
    if (g_listen < 0) return -1;
    int one = 1; setsockopt(g_listen, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
    struct sockaddr_in a; memset(&a, 0, sizeof a);
    a.sin_family = AF_INET; a.sin_addr.s_addr = INADDR_ANY; a.sin_port = htons((uint16_t)port);
    if (bind(g_listen, (struct sockaddr *)&a, sizeof a) < 0) { close(g_listen); g_listen = -1; return -1; }
    if (listen(g_listen, 16) < 0) { close(g_listen); g_listen = -1; return -1; }
    g_stop = 0;
    return pthread_create(&g_thr, NULL, accept_loop, NULL) == 0 ? 0 : -1;
}

void httpd_stop(void)
{
    g_stop = 1;
    if (g_listen >= 0) { shutdown(g_listen, SHUT_RDWR); close(g_listen); g_listen = -1; }
    pthread_join(g_thr, NULL);
}
