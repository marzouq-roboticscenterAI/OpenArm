/* httpd.h - tiny dependency-free HTTP server for the dashboard + control API. */
#ifndef HTTPD_H
#define HTTPD_H

/* Start the server on port, serving static files from webroot. Spawns a thread.
 * Returns 0 on success. */
int  httpd_start(int port, const char *webroot);
void httpd_stop(void);

#endif /* HTTPD_H */
