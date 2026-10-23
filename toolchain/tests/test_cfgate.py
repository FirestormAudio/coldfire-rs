import subprocess
from pathlib import Path

import pytest

from cfgate import check, normalise


@pytest.mark.parametrize("raw, want", [
    ("movel %d1,4001000c", "movel %d1,0x4001000c"),           # bare absolute address
    ("lea 40000b14,%a3", "lea 0x40000b14,%a3"),
    ("pea 7ff", "pea 0x7ff"),                                  # short bare operand
    ("lea %sp@(2c,%d3:l),%a0", "lea %sp@(0x2c,%d3:l),%a0"),    # indexed displacement
    ("bnes 0x40000600", "bnes ."),                             # PC-relative branch
    ("bras 0x4000057c", "bras ."),
    ("bset #3,%d0", "bset #3,%d0"),                            # bit ops are not branches
    ("btst #28,%d0", "btst #28,%d0"),
    ("jsr 400005ce", "jsr 0x400005ce"),
    ("movel #-8193,%d0", "movel #-8193,%d0"),                  # immediates untouched
    ("movew #-32537,%d0", "movew #-32537,%d0"),                # 5-digit negative decimal immediate
    ("movel #305419896,%d0", "movel #305419896,%d0"),          # large decimal immediate
    ("cpushl bc,%a0@", "cpushl %bc,%a0@"),                     # cache operand, not the number 0xbc
    ("cpushl dc,%a1@", "cpushl %dc,%a1@"),
    ("cpushp bc,%a0@", "cpushp 0xbc,%a0@"),                    # not ColdFire: left for the assembler to reject
])
def test_normalise(raw, want):
    assert normalise(raw) == want


def _obj(tmp_path: Path, src: str, cpu: str) -> Path:
    s, o = tmp_path / "t.s", tmp_path / "t.o"
    s.write_text("    .text\n" + src)
    subprocess.run(["m68k-elf-as", f"-mcpu={cpu}", str(s), "-o", str(o)], check=True)
    return o


def test_coldfire_clean_code_passes(tmp_path):
    r = check(_obj(tmp_path, "move.l %d0,%d1\n lea 0x40000b14,%a3\n bra.s .\n mvs.w %d0,%d1\n rts\n", "54418"))
    assert r.ok and r.instructions == 5


def test_non_coldfire_code_fails(tmp_path):
    r = check(_obj(tmp_path, "add.w %d0,%d1\n rol.l #1,%d0\n nop\n", "68020"))
    assert not r.ok and len(r.undecodable) >= 2


def test_cache_push_passes(tmp_path):
    r = check(_obj(tmp_path, "cpushl %bc,(%a0)\n cpushl %dc,(%a1)\n rts\n", "54418"))
    assert r.ok, r.rejected


def test_a_semihosting_sentinel_after_its_trap_passes(tmp_path):
    r = check(_obj(tmp_path, ".balignw 4,0x4e71\n nop\n halt\n .long 0x4e7bf000\n rts\n", "54418"))
    assert r.ok, r.rejected


def test_the_sentinel_without_its_trap_fails(tmp_path):
    # no HALT before it, and a NOP off the 4-byte boundary: an ordinary (invalid) MOVEC either way
    assert not check(_obj(tmp_path, ".balignw 4,0x4e71\n nop\n nop\n .long 0x4e7bf000\n rts\n", "54418")).ok
    assert not check(_obj(tmp_path, ".balignw 4,0x4e71\n nop\n nop\n halt\n .long 0x4e7bf000\n rts\n", "54418")).ok


def test_fpu_instruction_fails(tmp_path):
    assert not check(_obj(tmp_path, "fmove.d %fp0,%fp1\n nop\n", "5475")).ok


def test_decimal_displacements_untouched():
    # objdump prints displacements in decimal; they are not bare absolute addresses
    assert normalise("movel %a0@(20000),%d0") == "movel %a0@(20000),%d0"
    assert normalise("lea %sp@(30000),%a1") == "lea %sp@(30000),%a1"


def test_large_displacement_code_passes(tmp_path):
    r = check(_obj(tmp_path, "move.l 20000(%a0),%d0\n lea 30000(%sp),%a1\n rts\n", "54418"))
    assert r.ok, (r.undecodable, r.rejected)


def test_gcc_switch_jump_table_passes(tmp_path):
    # GCC's m68k tablejump puts a .word offset table inline in .text after `jmp %pc@(..,Xn)`
    c, o = tmp_path / "sw.c", tmp_path / "sw.o"
    c.write_text(
        "void g0(void);void g1(void);void g2(void);void g3(void);void g4(void);void g5(void);void g6(void);\n"
        "void f(unsigned x){switch(x){case 0:g0();break;case 1:g1();break;case 2:g2();break;case 3:g3();break;"
        "case 4:g4();break;case 5:g5();break;case 6:g6();break;}}\n")
    subprocess.run(["m68k-elf-gcc", "-mcpu=54418", "-O2", "-c", str(c), "-o", str(o)], check=True)
    dis = subprocess.run(["m68k-elf-objdump", "-d", str(o)], capture_output=True, text=True).stdout
    assert "%pc@(" in dis                                     # really is a PC-relative table jump
    r = check(o)
    assert r.ok, (r.undecodable, r.rejected)


def test_jump_table_whose_cases_all_come_before_it_passes(tmp_path):
    # rustc_codegen_gcc (embassy-usb's UsbDevice::run): every case body precedes the table, so all offsets
    # are negative and there is no forward target to end the table at; the code after it must still check
    s, o = tmp_path / "back.s", tmp_path / "back.o"
    s.write_text(
        "    .text\n    .globl f\n    .type f,@function\n"
        "f:  bra.s 1f\n"
        "c0: moveq #0,%d0\n    rts\n"
        "c1: moveq #1,%d0\n    rts\n"
        "c2: moveq #2,%d0\n    rts\n"
        "1:  move.w (T,%pc,%d0.l*2),%d0\n    ext.l %d0\n    jmp (T,%pc,%d0.l)\n"
        "T:  .word c0-T, c1-T, c2-T, c1-T\n"
        "    tst.b %d0\n    bne.s c0\n    rts\n"
        "    .size f, .-f\n")
    subprocess.run(["m68k-elf-as", "-mcpu=54418", str(s), "-o", str(o)], check=True)
    dis = subprocess.run(["m68k-elf-objdump", "-d", "-m", "m68k:isa-c:emac", str(o)], capture_output=True, text=True).stdout
    assert "%pc@(" in dis and ".short" in dis                 # a table jump objdump can't decode as code
    r = check(o)
    assert r.ok, (r.undecodable, r.rejected)
    assert r.instructions >= 12                               # the code after the table was checked too


def test_code_after_a_backward_table_is_checked_even_when_it_looks_like_an_entry(tmp_path):
    # the word after the table (sub.l %d1,%d0 = 0x9081, offset -0x6F7F) points inside this large
    # function, but at no instruction: it must end the table, so the 68020-only instruction after it
    # is still seen and rejected
    s, o = tmp_path / "big.s", tmp_path / "big.o"
    s.write_text(
        "    .text\n    .globl f\n    .type f,@function\n"
        "f:  bra.w 1f\n"
        "c0: moveq #0,%d0\n    rts\n"
        "    .rept 15000\n    nop\n    .endr\n"
        "1:  move.w (T,%pc,%d0.l*2),%d0\n    ext.l %d0\n    jmp (T,%pc,%d0.l)\n"
        "T:  .word c0-T, c0-T\n"
        "    sub.l %d1,%d0\n"
        "    rol.l #1,%d0\n"                    # not in the ColdFire ISA
        "    rts\n"
        "    .size f, .-f\n")
    subprocess.run(["m68k-elf-as", "-mcpu=68020", str(s), "-o", str(o)], check=True)
    r = check(o)
    assert not r.ok and r.undecodable, (r.undecodable, r.rejected)


def test_elf_without_code_fails(tmp_path):
    # minor #12: an ELF with nothing to check (empty, or not m68k at all) must not pass the gate
    s, o = tmp_path / "t.s", tmp_path / "t.o"
    s.write_text("    .data\n    .long 1\n")
    subprocess.run(["m68k-elf-as", "-mcpu=54418", str(s), "-o", str(o)], check=True)
    r = check(o)
    assert r.instructions == 0 and not r.ok


def test_emac_instructions_pass(tmp_path):
    # the MCF5441x has the EMAC: mac/msac, accumulator and MACSR moves are part of its ISA
    code = "mac.l %d0,%d1,%acc0\n msac.l %d2,%d3,%acc3\n movclr.l %acc0,%d2\n move.l %acc1,%d4\n" \
           " move.l %d0,%acc2\n move.l %d0,%macsr\n move.l %macsr,%d1\n nop\n"
    r = check(_obj(tmp_path, code, "54418"))
    assert r.ok, r


def test_the_cpu_and_decoder_are_configurable(tmp_path, monkeypatch):
    # another ColdFire (no EMAC): CF_OBJDUMP_ARCH / CF_CPU select it, and EMAC code then fails
    o = _obj(tmp_path, "mac.l %d0,%d1,%acc0\n nop\n", "54418")
    monkeypatch.setenv("CF_OBJDUMP_ARCH", "m68k:isa-c")
    monkeypatch.setenv("CF_CPU", "5208")
    assert not check(o).ok
