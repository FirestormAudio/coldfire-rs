/* Start-up: the stack, a cleared .bss (.data is linked in place), INIT_SIM, main, then EXIT with its result.

   INIT_SIM may move the stack (D1 and SP), but QEMU moves it to its RAM size, 0x08000000, which is not RAM on
   the mcf5208evb (RAM starts at 0x40000000), so the stack is set again after it. mcf5441x-run leaves both alone. */
#include "trap.h"

    .text
    .global _start
_start:
    move.l  #__stack,%sp
    lea     __bss_start,%a0
    lea     _end,%a1
1:  cmp.l   %a1,%a0
    bcc     2f
    clr.l   (%a0)+
    bra     1b
2:  moveq   #1,%d0
    move.l  #__stack,%d1
    TRAP
    move.l  #__stack,%sp
    jsr     main
    move.l  %d0,%d1
    moveq   #0,%d0
    TRAP
3:  bra     3b
