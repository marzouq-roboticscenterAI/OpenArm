/* log.h - tiny thread-safe timestamped logger (actions + motor state) to a file. */
#ifndef OA_LOG_H
#define OA_LOG_H

void oa_log_open(const char *path);          /* open/append the log file */
void oa_log(const char *fmt, ...)            /* timestamped line, flushed */
    __attribute__((format(printf, 1, 2)));
void oa_log_close(void);

#endif /* OA_LOG_H */
