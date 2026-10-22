/* 200 lines of formatted numbers, strings and widths. */
#include "mini.h"

static const char *const words[] = {"alpha", "beta", "gamma", "delta", "epsilon"};

int main(void)
{
    unsigned x = 2463534242u;
    for (int i = 0; i < 200; i++) {
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        int d = (int)x / 1000 - i * 7919;
        printf("%3d|%-8s|%08x|%u|%6d|%-6d|%c|%%|%s\n", i, words[i % 5], x, x % 100000u, d % 100000, -i,
               'a' + i % 26, i & 1 ? "odd" : "even");
    }
    return 0;
}
