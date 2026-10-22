/* ISATTY on fds 0-4 and a WRITE to fd 7: each call's result, and errno when it fails. */
#include "mini.h"

int main(void)
{
    unsigned b[4];
    for (int fd = 0; fd < 5; fd++) {
        b[0] = fd;
        b[1] = 0;
        semi_raw(12, b);
        if (b[0] == 1)
            printf("isatty(%d) = 1\n", fd);
        else
            printf("isatty(%d) = %d, errno %d\n", fd, (int)b[0], (int)b[1]);
    }
    b[0] = 7;
    b[1] = (unsigned)"x";
    b[2] = 1;
    semi_raw(5, b);
    printf("write(7) = %d, errno %d\n", (int)b[0], (int)b[1]);
    return 0;
}
