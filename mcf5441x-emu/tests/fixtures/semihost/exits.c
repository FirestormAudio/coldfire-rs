/* Exit with the first byte of stdin, or 42 at EOF. */
#include "mini.h"

int main(void)
{
    unsigned char c;
    if (semi_read(0, &c, 1) == 1)
        return c;
    return 42;
}
