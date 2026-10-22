#include "mini.h"

int main(void)
{
    puts("hello, world");
    fdprintf(2, "a line on stderr\n");
    return 0;
}
