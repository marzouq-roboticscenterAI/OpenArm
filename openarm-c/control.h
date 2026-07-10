/* control.h - OpenArm-C control engine.
 *
 * Owns the CAN buses and a 50 Hz control thread. On start it forces every present
 * motor into MIT mode and holds its current pose (position permanence). It
 * auto-detects a gamepad; while one is connected it runs Control Scheme 1
 * (per-joint jog). The HTTP layer reads a thread-safe snapshot and issues
 * thread-safe commands (estop, jog, calibrate).
 */
#ifndef CONTROL_H
#define CONTROL_H

#define OA_MIN_MOTOR 1
#define OA_MAX_MOTOR 8
#define OA_NBUS      2

typedef struct {
    int   present;      /* motor answered on the bus            */
    int   active;       /* enabled + being streamed             */
    int   err;          /* last feedback status code            */
    float pos, vel, tau;
    float target;       /* commanded position (eases toward goal) */
    float goal;         /* where inputs want it; target slews here */
    float tmin, tmax;   /* jog bounds (calibrated range if any)  */
    float kp, kd;       /* per-joint gains actually sent         */
    int   has_cal;      /* tmin/tmax came from calibration       */
    int   cal_step;     /* manual cal: 0=none,1=one end set,2=done */
    char  model[12];
} oa_motor_t;

typedef struct {
    char       iface[16];
    int        opened;                 /* socket opened          */
    int        nmotors;                /* present motors          */
    oa_motor_t m[OA_MAX_MOTOR + 1];    /* index by id (1..8)      */
} oa_bus_t;

typedef struct {
    oa_bus_t bus[OA_NBUS];
    int      nbus;
    int      pad_connected;
    char     pad_name[128];
    /* live controller input (for the on-screen gamepad highlight) */
    float    pad_stick_x;              /* -1..+1                  */
    float    pad_rstick_x, pad_rstick_y;
    int      pad_dpad_x, pad_dpad_y;   /* -1/0/+1                 */
    int      pad_btn_a, pad_btn_start;
    int      pad_btn_l1, pad_btn_r1;
    float    pad_l2, pad_r2;
    int      sel_bus, sel_motor;       /* scheme-1 selection      */

    /* Ranger Air rover chassis (can2) */
    int      ranger_present, ranger_enabled, ranger_estop;
    float    ranger_lin, ranger_ang, ranger_voltage;
    /* DS2-C lift servo (can3) */
    int      ds2c_present, ds2c_enabled, ds2c_vel;
    int      connected;                /* operator pressed Connect */
    int      estopped;
    int      calibrating;              /* auto-calibration running  */
    int      manual_cal;               /* manual teach-cal mode on  */
    float    gain_scale;
    char     status[160];
} oa_state_t;

/* Open the given interfaces and scan for motors. Returns #buses opened. */
int  control_init(const char *ifaces[], int n);
/* Spawn the control thread (enables + holds present motors). */
int  control_start(void);
/* Stop the thread, disable all motors, close buses. */
void control_stop(void);

/* Thread-safe snapshot for the HTTP layer. */
void control_get_state(oa_state_t *out);

/* Thread-safe commands (safe to call from the HTTP thread). */
void control_connect(void);      /* enable + hold present motors        */
void control_disconnect(void);   /* disable all motors (limp)           */
void control_estop(void);
void control_clear_estop(void);
void control_jog(int bus, int motor, float delta);
void control_set_target(int bus, int motor, float pos);   /* absolute (sliders) */
void control_select(int bus, int motor);
void control_request_calibration(void);      /* auto sweep-calibration */
void control_manual_start(void);              /* enter manual teach-cal */
void control_manual_stop(void);               /* exit manual teach-cal  */
void control_manual_mark(void);               /* record an endpoint for the selected joint */
void control_set_gain_scale(float s);
/* Web base/lift setpoints (normalized [-1,1]); gamepad takes precedence when
 * active, and each decays to 0 if not refreshed within the freshness window. */
void control_set_web_rover(float lin_norm, float ang_norm);
void control_set_web_lift(float vel_norm);

#endif /* CONTROL_H */
