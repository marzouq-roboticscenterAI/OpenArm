/* gamepad.c - evdev gamepad auto-detect + poll. See gamepad.h. */
#define _GNU_SOURCE
#include "gamepad.h"

#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <dirent.h>
#include <sys/ioctl.h>
#include <linux/input.h>

#define DEFAULT_DEADZONE 0.12f

static int looks_like_pad(int fd)
{
    unsigned long keybits[(KEY_MAX / (8 * sizeof(long))) + 1];
    memset(keybits, 0, sizeof keybits);
    if (ioctl(fd, EVIOCGBIT(EV_KEY, sizeof keybits), keybits) < 0) return 0;
    /* A real pad exposes BTN_GAMEPAD / BTN_SOUTH; skip motion/IMU/touch nodes. */
    int has = (keybits[BTN_SOUTH / (8 * sizeof(long))] >> (BTN_SOUTH % (8 * sizeof(long)))) & 1;
    int gp  = (keybits[BTN_GAMEPAD / (8 * sizeof(long))] >> (BTN_GAMEPAD % (8 * sizeof(long)))) & 1;
    return has || gp;
}

void gamepad_init(gamepad_t *g)
{
    memset(g, 0, sizeof *g);
    g->fd = -1;
    g->_ax_min = -32768; g->_ax_max = 32767;
    g->_dz = DEFAULT_DEADZONE;
}

int gamepad_detect(gamepad_t *g)
{
    if (g->fd >= 0) return 1;
    DIR *d = opendir("/dev/input");
    if (!d) return 0;
    struct dirent *e;
    while ((e = readdir(d))) {
        if (strncmp(e->d_name, "event", 5) != 0) continue;
        char path[64];
        snprintf(path, sizeof path, "/dev/input/%.48s", e->d_name);
        int fd = open(path, O_RDONLY | O_NONBLOCK);
        if (fd < 0) continue;
        if (!looks_like_pad(fd)) { close(fd); continue; }

        g->fd = fd;
        snprintf(g->devnode, sizeof g->devnode, "%s", path);
        if (ioctl(fd, EVIOCGNAME(sizeof g->name), g->name) < 0)
            snprintf(g->name, sizeof g->name, "controller");
        /* Calibrate left-stick X range. */
        struct input_absinfo ai;
        if (ioctl(fd, EVIOCGABS(ABS_X), &ai) == 0 && ai.maximum > ai.minimum) {
            g->_ax_min = ai.minimum; g->_ax_max = ai.maximum;
        }
        g->_hat_x = g->_hat_y = 0;
        g->_prev_dx = g->_prev_dy = g->_prev_a = 0;
        break;
    }
    closedir(d);
    return g->fd >= 0;
}

static float norm_axis(gamepad_t *g, int value)
{
    float c = (g->_ax_max + g->_ax_min) / 2.0f;
    float half = (g->_ax_max - g->_ax_min) / 2.0f;
    if (half <= 0) return 0.0f;
    float v = (value - c) / half;             /* -1..+1 */
    if (v > 1) v = 1;
    if (v < -1) v = -1;
    /* rescaled deadzone */
    float a = v < 0 ? -v : v;
    if (a < g->_dz) return 0.0f;
    float s = (a - g->_dz) / (1.0f - g->_dz);
    return (v < 0 ? -s : s);
}

int gamepad_poll(gamepad_t *g)
{
    g->dpad_up_edge = g->dpad_down_edge = g->dpad_left_edge = g->dpad_right_edge = 0;
    g->btn_a_edge = g->btn_l3_edge = g->btn_r3_edge = 0;
    if (g->fd < 0) return 0;

    struct input_event ev;
    for (;;) {
        ssize_t n = read(g->fd, &ev, sizeof ev);
        if (n == (ssize_t)sizeof ev) {
            if (ev.type == EV_ABS) {
                if (ev.code == ABS_X)          g->stick_x = norm_axis(g, ev.value);
                else if (ev.code == ABS_HAT0X) g->_hat_x = ev.value;
                else if (ev.code == ABS_HAT0Y) g->_hat_y = ev.value;
            } else if (ev.type == EV_KEY) {
                if (ev.code == BTN_SOUTH) g->btn_a = ev.value ? 1 : 0;
                else if (ev.code == BTN_START) g->btn_start = ev.value ? 1 : 0;
                else if (ev.code == BTN_THUMBL) g->btn_l3 = ev.value ? 1 : 0;
                else if (ev.code == BTN_THUMBR) g->btn_r3 = ev.value ? 1 : 0;
            }
            continue;
        }
        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) break;   /* drained */
        if (n < 0 && errno == EINTR) continue;
        /* device error / unplugged */
        gamepad_close(g);
        return 0;
    }

    g->dpad_x = (g->_hat_x < 0) ? -1 : (g->_hat_x > 0 ? 1 : 0);
    g->dpad_y = (g->_hat_y < 0) ? -1 : (g->_hat_y > 0 ? 1 : 0);

    /* rising edges */
    if (g->dpad_y < 0 && g->_prev_dy >= 0) g->dpad_up_edge = 1;
    if (g->dpad_y > 0 && g->_prev_dy <= 0) g->dpad_down_edge = 1;
    if (g->dpad_x < 0 && g->_prev_dx >= 0) g->dpad_left_edge = 1;
    if (g->dpad_x > 0 && g->_prev_dx <= 0) g->dpad_right_edge = 1;
    if (g->btn_a && !g->_prev_a) g->btn_a_edge = 1;
    if (g->btn_l3 && !g->_prev_l3) g->btn_l3_edge = 1;
    if (g->btn_r3 && !g->_prev_r3) g->btn_r3_edge = 1;
    g->_prev_dx = g->dpad_x; g->_prev_dy = g->dpad_y; g->_prev_a = g->btn_a;
    g->_prev_l3 = g->btn_l3; g->_prev_r3 = g->btn_r3;
    return 1;
}

void gamepad_close(gamepad_t *g)
{
    if (g->fd >= 0) close(g->fd);
    g->fd = -1;
    g->stick_x = 0; g->dpad_x = g->dpad_y = 0;
    g->btn_a = g->btn_start = g->btn_l3 = g->btn_r3 = 0;
    g->name[0] = 0; g->devnode[0] = 0;
}
