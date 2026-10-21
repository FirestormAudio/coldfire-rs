| Count five periods of PIT0 with the core in STOP between them.
    .text
    move.l #pit_isr, %d0
    move.l %d0, 0x40200334          | vector 205 (INTC2 source 13)
    moveq #6, %d0
    move.b %d0, 0xFC05004D          | INTC2 ICR13 = level 6
    moveq #13, %d0
    move.b %d0, 0xFC05001D          | INTC2 CIMR: unmask source 13
    move.w #0x021E, %d0             | PRE=2 OVW PIE PIF RLD (EN=0)
    move.w %d0, 0xFC080000
    move.w #32999, %d0              | (32999+1)*4 = 132000 ticks
    move.w %d0, 0xFC080002
    move.w #0x021F, %d0             | EN
    move.w %d0, 0xFC080000
loop:
    stop #0x2000
    move.l 0x40102000, %d1
    cmp.l #5, %d1
    bne.s loop
    moveq #0x6F, %d0
    bsr.s putc                      | 'o'
    moveq #0x6B, %d0
    bsr.s putc                      | 'k'
done:
    bra.s done
putc:                               | UART9: wait for TXRDY, then UTB = d0
    move.b 0xEC074004, %d1
    btst #2, %d1
    beq.s putc
    move.b %d0, 0xEC07400C
    rts
pit_isr:
    move.w 0xFC080000, %d0
    move.w %d0, 0xFC080000          | write PCSR back with PIF set: clears PIF
    move.l 0x40102000, %d1
    addq.l #1, %d1
    move.l %d1, 0x40102000
    rte
