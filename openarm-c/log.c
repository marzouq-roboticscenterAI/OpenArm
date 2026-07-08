/* log.c - see log.h. */
#define _GNU_SOURCE
#include "log.h"

#include <stdio.h>
#include <stdarg.h>
#include <time.h>
#include <pthread.h>

static FILE           *g_lf = NULL;
static pthread_mutex_t g_lm = PTHREAD_MUTEX_INITIALIZER;

void oa_log_open(const char *path)
{
    pthread_mutex_lock(&g_lm);
    if (!g_lf) g_lf = fopen(path, "a");
    pthread_mutex_unlock(&g_lm);
    oa_log("======== log opened ========");
}

void oa_log(const char *fmt, ...)
{
    pthread_mutex_lock(&g_lm);
    if (g_lf) {
        struct timespec ts; clock_gettime(CLOCK_REALTIME, &ts);
        struct tm tm; localtime_r(&ts.tv_sec, &tm);
        fprintf(g_lf, "[%02d:%02d:%02d.%03ld] ",
                tm.tm_hour, tm.tm_min, tm.tm_sec, ts.tv_nsec / 1000000L);
        va_list ap; va_start(ap, fmt); vfprintf(g_lf, fmt, ap); va_end(ap);
        fputc('\n', g_lf);
        fflush(g_lf);
    }
    pthread_mutex_unlock(&g_lm);
}

void oa_log_close(void)
{
    pthread_mutex_lock(&g_lm);
    if (g_lf) { fclose(g_lf); g_lf = NULL; }
    pthread_mutex_unlock(&g_lm);
}
