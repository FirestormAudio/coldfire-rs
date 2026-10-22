/* puts and a small printf (%d %u %x %c %s %%, with widths, '-' and '0'), in 32-bit arithmetic only. */
#include <stdarg.h>
#include "mini.h"

int strlen(const char *s)
{
    int n = 0;
    while (s[n])
        n++;
    return n;
}

int puts(const char *s)
{
    semi_write(1, s, strlen(s));
    semi_write(1, "\n", 1);
    return 0;
}

struct out {
    int fd;
    int len;
    char buf[128];
};

static void flush(struct out *o)
{
    if (o->len)
        semi_write(o->fd, o->buf, o->len);
    o->len = 0;
}

static void put(struct out *o, char c)
{
    if (o->len == (int)sizeof o->buf)
        flush(o);
    o->buf[o->len++] = c;
}

static void field(struct out *o, const char *s, int n, int width, int left, char pad)
{
    if (!left)
        for (int i = n; i < width; i++)
            put(o, pad);
    for (int i = 0; i < n; i++)
        put(o, s[i]);
    if (left)
        for (int i = n; i < width; i++)
            put(o, ' ');
}

static int vfdprintf(int fd, const char *fmt, va_list ap)
{
    struct out o;
    o.fd = fd;
    o.len = 0;
    char digits[12];
    for (; *fmt; fmt++) {
        if (*fmt != '%') {
            put(&o, *fmt);
            continue;
        }
        fmt++;
        int left = 0, width = 0;
        char pad = ' ';
        if (*fmt == '-') {
            left = 1;
            fmt++;
        }
        if (*fmt == '0') {
            pad = '0';
            fmt++;
        }
        while (*fmt >= '0' && *fmt <= '9')
            width = width * 10 + (*fmt++ - '0');
        unsigned v, base = 10;
        int neg = 0, n = 0;
        switch (*fmt) {
        case 'd': {
            int d = va_arg(ap, int);
            neg = d < 0;
            v = neg ? -(unsigned)d : (unsigned)d;
            goto number;
        }
        case 'x':
            base = 16;
            /* fall through */
        case 'u':
            v = va_arg(ap, unsigned);
        number:
            do {
                digits[sizeof digits - 1 - n++] = "0123456789abcdef"[v % base];
                v /= base;
            } while (v);
            if (neg) {
                if (pad == '0') {
                    put(&o, '-');
                    width--;
                } else
                    digits[sizeof digits - 1 - n++] = '-';
            }
            field(&o, digits + sizeof digits - n, n, width, left, pad);
            break;
        case 'c': {
            char c = (char)va_arg(ap, int);
            field(&o, &c, 1, width, left, ' ');
            break;
        }
        case 's': {
            const char *s = va_arg(ap, const char *);
            field(&o, s, strlen(s), width, left, ' ');
            break;
        }
        case '%':
            put(&o, '%');
            break;
        default:
            return -1;
        }
    }
    flush(&o);
    return 0;
}

int printf(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    int r = vfdprintf(1, fmt, ap);
    va_end(ap);
    return r;
}

int fdprintf(int fd, const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    int r = vfdprintf(fd, fmt, ap);
    va_end(ap);
    return r;
}
