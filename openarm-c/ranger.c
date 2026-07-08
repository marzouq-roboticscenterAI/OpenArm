/* ranger.c - AgileX Ranger Air chassis driver. See ranger.h.
 *
 * Protocol (Ranger Air manual 3.2, CAN 2.0B @ 500 kbit/s, big-endian fields):
 *   0x421 control-mode  (1 byte): 0=STANDBY 1=CAN_COMMAND
 *   0x141 motion-mode   (1 byte): 0=ACKERMANN 1=TILT 2=SPIN 3=PARK
 *   0x111 motion        (8 byte): >h h xx h -> lin(mm/s) ang(mrad/s) rsv steer(mrad)
 *   0x441 clear-error   (1 byte)
 *   0x211 system feedback: b0 normal, b2:4 voltage/10, b7 bit7 = e-stop
 */
#define _POSIX_C_SOURCE 199309L
#include "ranger.h"
#include "socketcan.h"
#include "damiao.h"

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <time.h>

#define RID_MOTION       0x111u
#define RID_MOTION_MODE  0x141u
#define RID_CONTROL_MODE 0x421u
#define RID_CLEAR_ERROR  0x441u
#define RID_FB_SYSTEM    0x211u

#define CTRL_STANDBY     0x00
#define CTRL_CAN_COMMAND 0x01

#define LIN_MAX   1.5f      /* m/s */
#define ANG_MAX   3.259f    /* rad/s (spin) */
#define STEER_MAX 0.698f    /* rad (ackermann) */
#define SETTLE_S  0.5       /* wheels re-orient on a mode switch */

static double now_s(void)
{ struct timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts); return ts.tv_sec + ts.tv_nsec*1e-9; }

static float clampf(float v, float lim) { return v < -lim ? -lim : (v > lim ? lim : v); }

static void put_be16(unsigned char *p, int v)
{ p[0] = (unsigned char)((v >> 8) & 0xFF); p[1] = (unsigned char)(v & 0xFF); }

static void send1(ranger_t *r, unsigned id, unsigned char val)
{
    if (r->fd < 0) return;
    dm_frame_t f; memset(&f, 0, sizeof f);
    f.id = id; f.dlc = 1; f.data[0] = val;
    scan_send(r->fd, &f);
}

void ranger_init(ranger_t *r)
{
    memset(r, 0, sizeof *r);
    r->fd = -1;
    r->cur_mode = RANGER_MODE_ACKERMANN;
    r->want_mode = RANGER_MODE_ACKERMANN;
}

int ranger_open(ranger_t *r, const char *iface)
{
    r->fd = scan_open(iface);
    snprintf(r->iface, sizeof r->iface, "%s", iface);
    return r->fd >= 0;
}

void ranger_close(ranger_t *r)
{
    if (r->fd >= 0) { ranger_disable(r); scan_close(r->fd); }
    r->fd = -1; r->enabled = 0;
}

void ranger_enable(ranger_t *r)
{
    if (r->fd < 0) return;
    send1(r, RID_CONTROL_MODE, CTRL_CAN_COMMAND);
    r->enabled = 1;
    r->lin = r->ang = r->steer = 0;
    /* force a mode (re)assertion so the chassis is in a known kinematic mode */
    r->cur_mode = -1;
    r->want_mode = RANGER_MODE_ACKERMANN;
}

void ranger_disable(ranger_t *r)
{
    r->lin = r->ang = r->steer = 0;
    r->enabled = 0;
    send1(r, RID_CONTROL_MODE, CTRL_STANDBY);
}

void ranger_clear_errors(ranger_t *r)
{ send1(r, RID_CLEAR_ERROR, 0x00); }

void ranger_drive(ranger_t *r, int mode, float lin, float ang)
{
    r->want_mode = mode;
    if (mode == RANGER_MODE_SPIN) { r->ang = clampf(ang, ANG_MAX); r->lin = 0; }
    else                          { r->lin = clampf(lin, LIN_MAX); r->ang = 0; }
    r->steer = 0;
}

void ranger_tick(ranger_t *r)
{
    if (r->fd < 0) return;

    /* drain feedback (non-blocking) so voltage / e-stop stay current */
    dm_frame_t rf;
    while (scan_recv(r->fd, &rf, 0) == 1) {
        r->connected = 1;
        if (rf.id == RID_FB_SYSTEM && rf.dlc >= 8) {
            r->voltage = ((rf.data[2] << 8) | rf.data[3]) / 10.0f;
            r->estop = (rf.data[7] & 0x80) ? 1 : 0;
        }
    }

    if (!r->enabled) return;

    /* Non-blocking mode switch: on a change, command the new mode and hold zero
     * motion for a short settle window while the steering wheels re-orient. */
    if (r->want_mode != r->cur_mode) {
        send1(r, RID_MOTION_MODE, (unsigned char)r->want_mode);
        r->cur_mode = r->want_mode;
        r->settle_until = now_s() + SETTLE_S;
    }

    float lin = r->lin, ang = r->ang, steer = r->steer;
    if (now_s() < r->settle_until) { lin = ang = steer = 0; }

    dm_frame_t f; memset(&f, 0, sizeof f);
    f.id = RID_MOTION; f.dlc = 8;
    put_be16(&f.data[0], (int)lroundf(clampf(lin, LIN_MAX) * 1000.0f));     /* mm/s */
    put_be16(&f.data[2], (int)lroundf(clampf(ang, ANG_MAX) * 1000.0f));     /* mrad/s */
    /* data[4..5] reserved */
    put_be16(&f.data[6], (int)lroundf(clampf(steer, STEER_MAX) * 1000.0f)); /* mrad */
    scan_send(r->fd, &f);
}
