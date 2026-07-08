/* calib.h - bounded, no-hang DaMiao joint calibration over classic CAN.
 *
 * Sweeps a joint to its hardstops using a slowly-advancing MIT target and stall
 * detection, then zeros it. Every sweep has a wall-clock deadline, so a motor
 * that never moves (or drops off the bus) fails fast instead of hanging.
 */
#ifndef CALIB_H
#define CALIB_H

#include "damiao.h"

enum { CAL_CENTER = 0, CAL_HANG = 1, CAL_GRIPPER = 2 };

typedef struct {
    int   ok;
    float low_stop, high_stop, zero, pmin, pmax, span;
    float start_pos;   /* initial angle mapped into the new frame (slider start) */
    char  msg[96];
} calib_result_t;

/* Calibrate one joint. mode: CAL_CENTER (zero at midpoint), CAL_HANG (zero at
 * gravity rest, for base joints), CAL_GRIPPER (zero at closed). *abort is polled
 * for Ctrl-C. Returns 1 on success (res->ok also set). Forces MIT mode first. */
int calib_run(int fd, uint32_t id, const dm_limits_t *lim, int mode,
              volatile int *abort, calib_result_t *res);

/* Keep a companion joint (on the same bus/fd) streamed to a fixed hold pose
 * during subsequent sweeps -- e.g. hold J2 at mid-range while calibrating J1 so
 * the arm stays put. Set before calib_run(); clear after. */
void calib_hold_companion(uint32_t id, float pos, const dm_limits_t *lim, float kp, float kd);
void calib_hold_clear(void);

/* Release a joint gently: ramp holding stiffness to zero while keeping damping so
 * it lowers slowly under gravity (instead of free-dropping when power is cut),
 * then disable. Returns when settled or after a timeout. */
void calib_gentle_release(int fd, uint32_t id, const dm_limits_t *lim, volatile int *abort);

#endif /* CALIB_H */
