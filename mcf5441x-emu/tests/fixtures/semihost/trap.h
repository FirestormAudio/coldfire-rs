/* The libgloss m68k semihosting trap: a 4-byte-aligned NOP, HALT and the sentinel. D0 is the call, D1 its
   argument or parameter block. */
.macro TRAP
    .balignw 4,0x4e71
    nop
    halt
    .long 0x4e7bf000
.endm
