/* ranger.h - AgileX Ranger Air chassis driver (classic CAN 2.0B, 500 kbit/s).
 *
 * Ported from RoverServoLib/ranger_air_control. Streams the 0x111 motion command
 * at the control-loop rate (the chassis halts if it sees no command for 500 ms).
 * Forward/back use Ackermann linear velocity; spin-in-place uses SPIN mode
 * angular velocity. Mode switches are non-blocking: a short settle window holds
 * zero motion while the steering wheels re-orient.
 */
#ifndef RANGER_H
#define RANGER_H

/* MotionMode values (0x141 command). */
#define RANGER_MODE_ACKERMANN 0
#define RANGER_MODE_TILT      1
#define RANGER_MODE_SPIN      2
#define RANGER_MODE_PARK      3

typedef struct {
    int   fd;              /* -1 when not open */
    char  iface[16];
    int   enabled;         /* heartbeat armed (CAN command mode) */

    int   cur_mode;        /* mode currently commanded on the chassis */
    int   want_mode;       /* mode the setpoint needs */
    double settle_until;   /* hold zero motion until this monotonic time */

    float lin, ang, steer; /* desired setpoint (m/s, rad/s, rad) */

    /* feedback (from 0x211 system frame) */
    int   connected;       /* any feedback frame seen */
    float voltage;
    int   estop;           /* chassis e-stop bit */
} ranger_t;

void ranger_init(ranger_t *r);
int  ranger_open(ranger_t *r, const char *iface);   /* 1 on success */
void ranger_close(ranger_t *r);

void ranger_enable(ranger_t *r);   /* enter CAN command mode, arm heartbeat */
void ranger_disable(ranger_t *r);  /* zero motion, return to standby */
void ranger_clear_errors(ranger_t *r);

/* Set the desired motion. mode picks ACKERMANN (use lin) or SPIN (use ang). */
void ranger_drive(ranger_t *r, int mode, float lin, float ang);

/* Call once per control tick (~50 Hz): handles mode-switch settling, streams
 * the 0x111 motion frame while enabled, and drains feedback. */
void ranger_tick(ranger_t *r);

#endif /* RANGER_H */
