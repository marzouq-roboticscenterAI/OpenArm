/* gamepad.h - evdev gamepad auto-detection + polling with hotplug.
 *
 * Scans /dev/input for a game controller, reads it non-blocking, and normalizes
 * the axes/buttons the control scheme needs. Re-detects on unplug/replug.
 */
#ifndef GAMEPAD_H
#define GAMEPAD_H

typedef struct {
    int   fd;               /* -1 when no controller is connected */
    char  name[128];
    char  devnode[64];      /* e.g. /dev/input/event18 */

    /* normalized, deadzoned inputs (updated by gamepad_poll) */
    float stick_x;          /* left stick X, -1..+1 */
    int   dpad_x, dpad_y;   /* -1/0/+1 */
    int   btn_a;            /* south button (A) */
    int   btn_start;        /* start button (e-stop) */
    int   btn_l3, btn_r3;   /* left / right stick click (thumb) */

    /* edge helpers: set for one poll when the button transitions 0->1 */
    int   dpad_up_edge, dpad_down_edge, dpad_left_edge, dpad_right_edge;
    int   btn_a_edge, btn_l3_edge, btn_r3_edge;

    /* internal */
    int   _hat_x, _hat_y;
    int   _prev_dx, _prev_dy, _prev_a, _prev_l3, _prev_r3;
    int   _ax_min, _ax_max;   /* left-stick X calibration */
    float _dz;                /* deadzone fraction */
} gamepad_t;

/* Initialize state (no device yet). */
void gamepad_init(gamepad_t *g);

/* Try to (re)open a controller if none is connected. Returns 1 if connected. */
int  gamepad_detect(gamepad_t *g);

/* Drain pending events (non-blocking) and refresh normalized fields + edges.
 * Detects disconnect (closes fd, returns 0). Returns 1 if connected. */
int  gamepad_poll(gamepad_t *g);

void gamepad_close(gamepad_t *g);

#endif /* GAMEPAD_H */
