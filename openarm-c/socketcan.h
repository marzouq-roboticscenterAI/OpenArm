/* socketcan.h - Linux SocketCAN transport for classic CAN 2.0 frames. */
#ifndef SOCKETCAN_H
#define SOCKETCAN_H

#include "damiao.h"

/* Open a bound SocketCAN socket on iface (e.g. "can0"). Returns fd or -1. */
int  scan_open(const char *iface);
void scan_close(int fd);

/* Send one frame. Returns 0 on success, -1 on error. */
int  scan_send(int fd, const dm_frame_t *f);

/* Receive one frame with a timeout (ms). Returns 1=frame, 0=timeout, -1=error.
 * EINTR is reported as a timeout so a signal returns control to the caller. */
int  scan_recv(int fd, dm_frame_t *f, int timeout_ms);

#endif /* SOCKETCAN_H */
