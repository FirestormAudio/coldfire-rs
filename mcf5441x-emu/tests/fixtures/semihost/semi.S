/* C-callable semihosting calls. Each builds the 4-longword parameter block on the stack and returns its first
   longword, the call's result. */
#include "trap.h"

    .text

/* int semi_call3(int code, int a, int b, int c) */
semi_call3:
    lea     -16(%sp),%sp
    move.l  24(%sp),(%sp)
    move.l  28(%sp),4(%sp)
    move.l  32(%sp),8(%sp)
    clr.l   12(%sp)
    move.l  20(%sp),%d0
    move.l  %sp,%d1
    TRAP
    move.l  (%sp),%d0
    lea     16(%sp),%sp
    rts

/* int semi_write(int fd, const void *buf, int len) */
    .global semi_write
semi_write:
    move.l  12(%sp),-(%sp)
    move.l  12(%sp),-(%sp)
    move.l  12(%sp),-(%sp)
    pea     5
    bsr     semi_call3
    lea     16(%sp),%sp
    rts

/* int semi_read(int fd, void *buf, int len) */
    .global semi_read
semi_read:
    move.l  12(%sp),-(%sp)
    move.l  12(%sp),-(%sp)
    move.l  12(%sp),-(%sp)
    pea     4
    bsr     semi_call3
    lea     16(%sp),%sp
    rts

/* int semi_isatty(int fd) */
    .global semi_isatty
semi_isatty:
    clr.l   -(%sp)
    clr.l   -(%sp)
    move.l  12(%sp),-(%sp)
    pea     12
    bsr     semi_call3
    lea     16(%sp),%sp
    rts

/* void semi_exit(int status) */
    .global semi_exit
semi_exit:
    moveq   #0,%d0
    move.l  4(%sp),%d1
    TRAP
1:  bra     1b

/* void semi_raw(int code, unsigned block[4]): the call with D1 at the caller's block, for its errno */
    .global semi_raw
semi_raw:
    move.l  4(%sp),%d0
    move.l  8(%sp),%d1
    TRAP
    rts
