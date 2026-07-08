/* ds2c.c - DS2-C servo drive (CANopen / CiA402) driver. See ds2c.h.
 *
 * Object dictionary / controlwords per RoverServoLib ds2c.py:
 *   0x6040 controlword (u16)   0x6060 mode-of-operation (i8, 3=profile velocity)
 *   0x6083 profile accel (u32) 0x6084 profile decel (u32)
 *   0x60FF target velocity (i32, pulses/s)
 *   enable: NMT operational, fault-reset(0x80), 6 -> 7 -> 15
 *   quick-stop: 0x0B    disable-voltage: 0x00
 * All SDO writes are expedited + fire-and-forget (no response wait).
 */
#define _POSIX_C_SOURCE 199309L
#include "ds2c.h"
#include "socketcan.h"
#include "damiao.h"

#include <stdio.h>
#include <string.h>
#include <time.h>

#define OD_CONTROLWORD   0x6040
#define OD_MODE_OP       0x6060
#define OD_PROFILE_ACCEL 0x6083
#define OD_PROFILE_DECEL 0x6084
#define OD_TARGET_VEL    0x60FF

#define MODE_PROFILE_VELOCITY 3

#define CW_SHUTDOWN         0x0006
#define CW_SWITCH_ON        0x0007
#define CW_ENABLE_OPERATION 0x000F
#define CW_QUICK_STOP       0x000B
#define CW_DISABLE_VOLTAGE  0x0000
#define CW_FAULT_RESET      0x0080

#define SDO_TX_BASE 0x600u   /* host -> drive */
#define SDO_RX_BASE 0x580u   /* drive -> host */
#define NMT_COB_ID  0x000u
#define NMT_START_REMOTE 0x01

#define PROFILE_ACCEL 20000u
#define PROFILE_DECEL 20000u

static void sleep_ms(int ms)
{ struct timespec ts = { ms/1000, (long)(ms%1000)*1000000L }; nanosleep(&ts, NULL); }

static int clampv(int pps)
{
    if (pps >  DS2C_MAX_PPS) return  DS2C_MAX_PPS;
    if (pps < -DS2C_MAX_PPS) return -DS2C_MAX_PPS;
    return pps;
}

/* Expedited SDO download (write), fire-and-forget. size = 1, 2 or 4 bytes. */
static void sdo_write(ds2c_t *d, int index, int sub, int size, int value)
{
    if (d->fd < 0) return;
    unsigned char cmd = (size == 1) ? 0x2F : (size == 2) ? 0x2B : 0x23;
    dm_frame_t f; memset(&f, 0, sizeof f);
    f.id = SDO_TX_BASE + (unsigned)d->node; f.dlc = 8;
    f.data[0] = cmd;
    f.data[1] = (unsigned char)(index & 0xFF);
    f.data[2] = (unsigned char)((index >> 8) & 0xFF);
    f.data[3] = (unsigned char)sub;
    unsigned int u = (unsigned int)value;
    f.data[4] = (unsigned char)(u & 0xFF);
    f.data[5] = (unsigned char)((u >> 8) & 0xFF);
    f.data[6] = (unsigned char)((u >> 16) & 0xFF);
    f.data[7] = (unsigned char)((u >> 24) & 0xFF);
    scan_send(d->fd, &f);
}

static void nmt_operational(ds2c_t *d)
{
    if (d->fd < 0) return;
    dm_frame_t f; memset(&f, 0, sizeof f);
    f.id = NMT_COB_ID; f.dlc = 2;
    f.data[0] = NMT_START_REMOTE; f.data[1] = (unsigned char)d->node;
    scan_send(d->fd, &f);
}

void ds2c_init(ds2c_t *d)
{
    memset(d, 0, sizeof *d);
    d->fd = -1;
    d->node = 1;
}

int ds2c_open(ds2c_t *d, const char *iface, int node)
{
    d->fd = scan_open(iface);
    d->node = node > 0 ? node : 1;
    snprintf(d->iface, sizeof d->iface, "%s", iface);
    return d->fd >= 0;
}

void ds2c_close(ds2c_t *d)
{
    if (d->fd >= 0) { ds2c_estop(d); scan_close(d->fd); }
    d->fd = -1; d->enabled = 0;
}

void ds2c_enable(ds2c_t *d)
{
    if (d->fd < 0) return;
    nmt_operational(d);                               sleep_ms(100);
    sdo_write(d, OD_CONTROLWORD, 0, 2, CW_FAULT_RESET); sleep_ms(200);
    /* profile-velocity mode; start at zero so the motor doesn't lurch on enable */
    sdo_write(d, OD_MODE_OP, 0, 1, MODE_PROFILE_VELOCITY);
    sdo_write(d, OD_PROFILE_ACCEL, 0, 4, PROFILE_ACCEL);
    sdo_write(d, OD_PROFILE_DECEL, 0, 4, PROFILE_DECEL);
    sdo_write(d, OD_TARGET_VEL, 0, 4, 0);             sleep_ms(50);
    sdo_write(d, OD_CONTROLWORD, 0, 2, CW_SHUTDOWN);         sleep_ms(100);
    sdo_write(d, OD_CONTROLWORD, 0, 2, CW_SWITCH_ON);        sleep_ms(100);
    sdo_write(d, OD_CONTROLWORD, 0, 2, CW_ENABLE_OPERATION); sleep_ms(100);
    d->enabled = 1;
    d->cur_vel = 0; d->want_vel = 0;
}

void ds2c_disable(ds2c_t *d)
{
    d->want_vel = 0; d->cur_vel = 0;
    if (d->fd >= 0) {
        sdo_write(d, OD_TARGET_VEL, 0, 4, 0);
        sdo_write(d, OD_CONTROLWORD, 0, 2, CW_DISABLE_VOLTAGE);
    }
    d->enabled = 0;
}

void ds2c_estop(ds2c_t *d)
{
    d->want_vel = 0; d->cur_vel = 0;
    if (d->fd >= 0) {
        sdo_write(d, OD_TARGET_VEL, 0, 4, 0);
        sdo_write(d, OD_CONTROLWORD, 0, 2, CW_QUICK_STOP);
        sleep_ms(20);
        sdo_write(d, OD_CONTROLWORD, 0, 2, CW_DISABLE_VOLTAGE);
    }
    d->enabled = 0;
}

void ds2c_set_velocity(ds2c_t *d, int pps)
{ d->want_vel = clampv(pps); }

void ds2c_tick(ds2c_t *d)
{
    if (d->fd < 0) return;

    /* drain rx (non-blocking): note any SDO server / heartbeat traffic */
    dm_frame_t rf;
    while (scan_recv(d->fd, &rf, 0) == 1) {
        d->connected = 1;
        if (rf.id == SDO_RX_BASE + (unsigned)d->node && rf.dlc >= 8 &&
            rf.data[1] == 0x41 && rf.data[2] == 0x60)          /* 0x6041 statusword upload */
            d->statusword = rf.data[4] | (rf.data[5] << 8);
    }

    if (!d->enabled) return;
    if (d->want_vel != d->cur_vel) {
        sdo_write(d, OD_TARGET_VEL, 0, 4, d->want_vel);
        d->cur_vel = d->want_vel;
    }
}
