/* ds2c.h - DS2-C servo drive (CANopen / CiA402, 1 Mbit/s) driver.
 *
 * Ported from RoverServoLib/ds2c-servo. Runs the drive in profile-velocity mode
 * so the right thumbstick jogs it continuously. SDO writes are fire-and-forget
 * (no blocking wait for the server response) so streaming velocity never stalls
 * the 50 Hz control loop.
 *
 * SAFETY: this is a SOFTWARE stop only; direction is INVERTED on this rig
 * (positive pulses/velocity move the carriage physically DOWN).
 */
#ifndef DS2C_H
#define DS2C_H

#define DS2C_PULSES_PER_REV 10000
#define DS2C_MAX_PPS        20000   /* software speed ceiling (=120 rpm) */

typedef struct {
    int  fd;             /* -1 when not open */
    char iface[16];
    int  node;           /* CANopen node id (default 1) */
    int  enabled;        /* CiA402 operation-enabled sequence sent */
    int  cur_vel;        /* last target velocity streamed (pps) */
    int  want_vel;       /* desired target velocity (pps) */
    int  connected;      /* any SDO/heartbeat response seen */
    int  statusword;     /* last statusword read (0 if none) */
} ds2c_t;

void ds2c_init(ds2c_t *d);
int  ds2c_open(ds2c_t *d, const char *iface, int node);   /* 1 on success */
void ds2c_close(ds2c_t *d);

void ds2c_enable(ds2c_t *d);            /* fault-reset, velocity mode, 6->7->15 */
void ds2c_disable(ds2c_t *d);           /* disable voltage */
void ds2c_estop(ds2c_t *d);             /* quick-stop then disable */

void ds2c_set_velocity(ds2c_t *d, int pps);   /* signed pps; clamped to limit */
void ds2c_tick(ds2c_t *d);              /* push velocity if it changed; drain rx */

#endif /* DS2C_H */
