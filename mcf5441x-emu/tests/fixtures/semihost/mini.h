/* A freestanding console on semihosting: the toolchain has no ColdFire C library. */
#ifndef MINI_H
#define MINI_H

int semi_write(int fd, const void *buf, int len);
int semi_read(int fd, void *buf, int len);
int semi_isatty(int fd);
void semi_exit(int status) __attribute__((noreturn));
void semi_raw(int code, unsigned block[4]);

int strlen(const char *s);
int puts(const char *s);
int printf(const char *fmt, ...);
int fdprintf(int fd, const char *fmt, ...);

#endif
