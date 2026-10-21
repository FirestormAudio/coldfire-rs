| An interrupt that becomes deliverable exactly at a STOP is taken once, and the STOP still waits for the next.
    .text
    move.l #pit_isr, %d0
    move.l %d0, 0x40200334          | vector 205 (INTC2 source 13)
    moveq #6, %d0
    move.b %d0, 0xFC05004D          | INTC2 ICR13 = level 6
    moveq #13, %d0
    move.b %d0, 0xFC05001D          | INTC2 CIMR: unmask source 13
    move.w #32999, %d0
    move.w %d0, 0xFC080002
    move.w #0x021F, %d0             | PRE=2 OVW PIE PIF RLD EN
    move.w %d0, 0xFC080000
wait:
    move.w 0xFC080000, %d0          | still at IPL 7: wait until PIF is pending
    btst #2, %d0
    beq.s wait
    move.w #0x2000, %sr             | the interrupt is deliverable from here
    stop #0x2000
done:
    bra.s done
pit_isr:
    move.w 0xFC080000, %d0
    move.w %d0, 0xFC080000          | clear PIF
    move.l 0x40102000, %d1
    addq.l #1, %d1
    move.l %d1, 0x40102000
    rte
