/* socketcan.c - classic CAN 2.0 transport. See socketcan.h. */
#define _GNU_SOURCE
#include "socketcan.h"

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <poll.h>
#include <linux/can.h>
#include <linux/can/raw.h>

int scan_open(const char *iface)
{
    int fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (fd < 0) return -1;

    struct ifreq ifr;
    memset(&ifr, 0, sizeof ifr);
    strncpy(ifr.ifr_name, iface, IFNAMSIZ - 1);
    if (ioctl(fd, SIOCGIFINDEX, &ifr) < 0) { close(fd); return -1; }

    struct sockaddr_can addr;
    memset(&addr, 0, sizeof addr);
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(fd, (struct sockaddr *)&addr, sizeof addr) < 0) { close(fd); return -1; }
    return fd;
}

void scan_close(int fd) { if (fd >= 0) close(fd); }

int scan_send(int fd, const dm_frame_t *f)
{
    struct can_frame cf;
    memset(&cf, 0, sizeof cf);
    cf.can_id  = f->id & CAN_SFF_MASK;
    cf.can_dlc = (f->dlc > 8) ? 8 : f->dlc;
    memcpy(cf.data, f->data, cf.can_dlc);
    ssize_t n = write(fd, &cf, sizeof cf);
    return (n == (ssize_t)sizeof cf) ? 0 : -1;
}

int scan_recv(int fd, dm_frame_t *f, int timeout_ms)
{
    struct pollfd pfd = { .fd = fd, .events = POLLIN };
    int pr = poll(&pfd, 1, timeout_ms);
    if (pr < 0) return (errno == EINTR) ? 0 : -1;   /* signal -> treat as timeout */
    if (pr == 0) return 0;

    struct can_frame cf;
    ssize_t n;
    do { n = read(fd, &cf, sizeof cf); } while (n < 0 && errno == EINTR);
    if (n < (ssize_t)sizeof cf) return -1;

    f->id  = cf.can_id & CAN_SFF_MASK;
    f->dlc = cf.can_dlc;
    memset(f->data, 0, sizeof f->data);
    memcpy(f->data, cf.data, (cf.can_dlc > 8) ? 8 : cf.can_dlc);
    return 1;
}
