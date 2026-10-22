/* Copy stdin to stdout in 7-byte reads until EOF or a NUL byte (QEMU's console never reports EOF). */
#include "mini.h"

int main(void)
{
    char buf[7];
    int n;
    while ((n = semi_read(0, buf, sizeof buf)) > 0) {
        int i = 0;
        while (i < n && buf[i])
            i++;
        semi_write(1, buf, i);
        if (i < n)
            return 0;
    }
    return n;
}
