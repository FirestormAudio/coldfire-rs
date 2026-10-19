from pathlib import Path

from rm2svd.build import build_device, chapter_name
from rm2svd.svd import write_svd

CH = """# Chapter 5: Timers (TMR0–TMR1)

**Table 5-2. Timer Memory Map**

|**Address**|||||
|---|---|---|---|---|
|**T0**<br>**T1**|**Register**<br>**Width**<br>**(bits)**|**Access**|**Reset Value**|**Section/Page**|
|0xFC08_0000<br>0xFC08_4000|Timer Control Register (TCR_n_)<br>16|R/W|0x0000|5.2.1/5-3|
|0xFC08_0002<br>0xFC08_4002|Timer Modulus Register (TMR_n_)<br>16|R/W|0xFFFF|5.2.2/5-5|

Address: 0xFC08_0000 (TCR0) Access: User read/write 0xFC08_4000 (TCR1) 15 14 R W Reset 0 0

**Table 5-3. TCR Field Descriptions**

|**Field**|**Description**|
|---|---|
|15–1|Reserved|
|0<br>EN|Enable.|

Address: 0xFC08_0006 (TXR0) 31 30 R W Reset 0 0
"""


def manual(tmp_path, *chapters):
    (tmp_path / "ch05-timers.md").write_text(CH)
    (tmp_path / "ch01-overview.md").write_text("")
    (tmp_path / "ch17-interrupt-controller-modules-intc.md").write_text("")
    for name, text in chapters:
        (tmp_path / name).write_text(text)
    return tmp_path


def test_instances_registers_and_fields(tmp_path):
    dev, problems = build_device(manual(tmp_path))
    ps = {p.name: p for p in dev.peripherals}
    assert sorted(ps) == ["T0", "T1"]                     # from the table's instance header cells
    p0, p1 = ps["T0"], ps["T1"]
    assert p0.group == "T"
    assert (p0.base, p1.base) == (0xFC080000, 0xFC084000)
    regs = {r.name: r for r in p0.registers}
    assert regs["TCR"].offset == 0 and regs["TMR"].offset == 2 and regs["TMR"].reset == 0xFFFF
    assert [f.name for f in regs["TCR"].fields] == ["EN"]


def test_registers_missing_a_width_are_reported(tmp_path):
    dev, problems = build_device(manual(tmp_path))
    assert any("no map row" in p and "TXR0" in p for p in problems)


def test_figure_only_register_takes_its_width_from_the_figure(tmp_path):
    dev, _ = build_device(manual(tmp_path))
    t0 = next(p for p in dev.peripherals if p.name == "T0")
    txr = next(r for r in t0.registers if r.name == "TXR")
    assert (txr.offset, txr.width) == (6, 32)


SER = """# Chapter 41: Serial Modules

|**Base Address**|**Module**|**Base Address**|**Module**|
|---|---|---|---|
|0xFC06_0000|SER 0|0xEC06_4000|SER 1|

|**Offset**|**Register**|**Width**|**Access**|**Reset Value**|**Section/Page**|
|---|---|---|---|---|---|
|0x00|Mode Register (SMR_n_)|8|R/W|0x00|41.3.1/41-5|
|0x04|Status Register (SSR_n_)|8|R|0x00|41.3.3/41-8|
||Clock Select Register (SCSR_n_)|8|W|See Section|41.3.4/41-9|

Address: 0xFC06_0004 (SSR0) 0xEC06_4004 (SSR1) 7 6 5 R W Reset 0 0

**Table 41-5. SSR Field Descriptions**

|**Field**|**Description**|
|---|---|
|7<br>RB|Received break.|
|6–5<br>FE|Framing error.|
|5<br>PE|Parity error.|
"""


def test_offset_rows_repeat_for_every_base_address(tmp_path):
    dev, problems = build_device(manual(tmp_path, ("ch41-serial.md", SER)))
    ps = {p.name: p for p in dev.peripherals}
    assert (ps["SER0"].base, ps["SER1"].base, ps["SER1"].group) == (0xFC060000, 0xEC064000, "SER")
    regs = {r.name: r for r in ps["SER1"].registers}
    assert {n: r.offset for n, r in regs.items()} == {"SMR": 0, "SSR": 4, "SCSR": 4}
    # the figure has no Access: line, so fields take the register's access
    assert [(f.name, f.access) for f in regs["SSR"].fields] == [("FE", "read-only"), ("RB", "read-only")]
    assert any(p.startswith("overlap:") and "PE" in p for p in problems)


CTL = """# Chapter 17: Interrupt Controller Modules (INTC)

|**Number**|**Base Address**|
|---|---|
|CTL0|0xFC04_8000|
|CTL1|0xFC04_C000|

|**Address**|**Register**|**Width**|**Access**|**Reset Value**|**Section/Page**|
|---|---|---|---|---|---|
|0xFC04_8000|Pending Register High (PRH0)|32|R|0x0000_0000|17.2.1/17-4|
|0xFC04_8040 +_n_<br>(_n_=0:1)|Control Registers (CR0_n_)|8|R/W|0x00|17.2.9/17-12|
|0xFC04_C000|Pending Register High (PRH1)|32|R|0x0000_0000|17.2.1/17-4|
|0xFC04_C040 +_n_<br>(_n_=0:1)|Control Registers (CR1_n_)|8|R/W|0x00|17.2.9/17-12|

**Table 17-15. Interrupt Source Assignment For INTC0**

|**Source**|**Module**|**Flag**|**Source Description**|**Flag Clearing Mechanism**|
|---|---|---|---|---|
|1|T 1|TCR1[EN]|Timer 1|Write 1|
|2|DMA|TMR0[X]|Wrong label, flag names the register|Write 1|
|3|Mystery|ZZZ[Q]|Nobody's|Write 1|
"""


def test_per_instance_rows_go_to_their_block_and_drop_the_instance_digit(tmp_path):
    dev, _ = build_device(manual(tmp_path, ("ch17-interrupt-controller-modules-intc.md", CTL)))
    ps = {p.name: p for p in dev.peripherals}
    assert sorted(r.name for r in ps["CTL0"].registers) == ["CR0", "CR1", "PRH"]
    assert {r.name: r.offset for r in ps["CTL1"].registers} == {"PRH": 0, "CR0": 0x40, "CR1": 0x41}


def test_interrupts_map_to_peripherals_by_flag_register_then_label(tmp_path):
    dev, problems = build_device(manual(tmp_path, ("ch17-interrupt-controller-modules-intc.md", CTL)))
    assert {i.name: i.peripheral for i in dev.interrupts} == {"TCR1_EN": "T1", "TMR0_X": "T0"}
    assert any(p.startswith("interrupt:") and "ZZZ_Q" in p for p in problems)
    write_svd(dev)                                       # every kept interrupt names a device peripheral


def test_chapter_names():
    assert chapter_name("# Chapter 19: Enhanced Direct Memory Access (eDMA)", 19) == "EDMA"
    assert chapter_name("# Chapter 38: Programmable Interrupt Timers (PIT0–PIT3)", 38) == "PIT"
    assert chapter_name("# Chapter 41: UART Modules", 41) == "UART"
    assert chapter_name("# Chapter 15: Pin-Multiplexing and Control", 15) == "PIN_MULTIPLEXING"
    assert chapter_name("# Chapter 36: 1-Wire Module", 36) is None          # not an identifier
    assert chapter_name("# Chapter 23: Universal Serial Bus Interface – Host Module", 23) == "USBHOST"


GPIO = """# Chapter 16: Rapid GPIO (RGP)

|**Address**|**Register**|**Width**|**Access**|**Reset Value**|**Section/Page**|
|---|---|---|---|---|---|
|0x8C00_0000|Direction Register (RGP_DIR)|16|W|0x0000|16.3.2/16-5|
|**Address**|**Register**|**Width**|**Access**|**Reset Value**|**Section/Page**|
|---|---|---|---|---|---|
|0x8C00_0000|Direction Register (RGP_DIR)|16|R|0x0000|16.3.2/16-5|
"""


def test_a_write_map_and_a_read_map_of_one_register_merge(tmp_path):
    dev, problems = build_device(manual(tmp_path, ("ch16-rgpio.md", GPIO)))
    (p,) = [p for p in dev.peripherals if p.name == "RGP"]
    assert [(r.name, r.access) for r in p.registers] == [("DIR", "read-write")]
    assert not any("RGP_DIR" in p and p.startswith("duplicate") for p in problems)


def test_peripherals_are_ordered_by_group_then_index(tmp_path):
    dev, _ = build_device(manual(tmp_path, ("ch41-serial.md", SER.replace("SER 1", "SER 10"))))
    assert [p.name for p in dev.peripherals if p.group == "SER"] == ["SER0", "SER10"]
    assert [p.name for p in dev.peripherals][:2] == ["SER0", "SER10"]


HOST = """# Chapter 23: Universal Serial Bus Interface – Host Module

|**Address**|**Register**|**Width**|**Access**|**Reset Value**|**Section/Page**|
|---|---|---|---|---|---|
|0xFC0B_4000|Identification (ID)|32|R|0x0000_0000|24.3.1.1/24-8|
"""

OTG = """# Chapter 24: Universal Serial Bus Interface – On-The-Go Module

|**Address**|**Register**|**Width**|**Access**|**Reset Value**|**Section/Page**|
|---|---|---|---|---|---|
|0xFC0B_0000|Identification (ID)|32|R|0x0000_0000|24.3.1.1/24-8|

Address: 0xFC0B_0000 (ID) Access: User read-only 31 30 R W Reset 0 0

**Table 24-5. ID Field Descriptions**

|**Field**|**Description**|
|---|---|
|5–0<br>ID|Identification.|
"""


def test_rows_described_in_another_chapter_take_that_chapters_fields(tmp_path):
    dev, problems = build_device(manual(tmp_path, ("ch23-host.md", HOST), ("ch24-otg.md", OTG)))
    ps = {p.name: p for p in dev.peripherals}
    assert [f.name for f in ps["USBHOST"].registers[0].fields] == ["ID"]
    assert not any(p.startswith("no figure") and "ch23" in p for p in problems)


WIDE = """# Chapter 19: Direct Memory Access (DMX)

|**Address**|**Register**|**Width**|**Access**|**Reset Value**|**Section/Page**|
|---|---|---|---|---|---|
|0xFC04_4000|Control Register (DMX_CR)|32|R/W|0x0000_0000|19.4.1/19-5|
|0xFC04_5000<br>+ hex(32_n_)|Transfer Control Descriptor (TCD_n_)<br>for_n_= 0 – 1|256|R/W|See Section|19.4.17/19-24|
"""


def test_rows_wider_than_32_bits_are_left_out_and_reported(tmp_path):
    dev, problems = build_device(manual(tmp_path, ("ch19-dmx.md", WIDE)))
    (p,) = [p for p in dev.peripherals if p.name == "DMX"]
    assert [r.name for r in p.registers] == ["CR"]
    assert sum(p.startswith("width:") and "TCD" in p for p in problems) == 1


MMU = """# Chapter 4: Memory Unit (MU)

**Table 4-1. MU Memory Map**

|**Address**|**Register**|**Width **|**Access**|**Reset Value**|**Section/Page**|
|---|---|---|---|---|---|
|MUBAR<br>+ 0x0000|MUCR—MU control register|32|R/W|0x0000_0000|4.2.3/4-5|
"""


def test_a_memory_map_without_absolute_addresses_is_reported(tmp_path):
    dev, problems = build_device(manual(tmp_path, ("ch04-mu.md", MMU)))
    assert not any(p.name.startswith("MU") for p in dev.peripherals)
    assert any(p.startswith("not memory-mapped: ch04-mu") for p in problems)


ALT = """# Chapter 41: Serial Modules

|**Base Address**|**Module**|**Base Address**|**Module**|
|---|---|---|---|
|0xFC06_0000|SER 0|0xEC06_4000|SER 1|

|**Offset**|**Register**|**Width**|**Access**|**Reset Value**|**Section/Page**|
|---|---|---|---|---|---|
|0x00|Mode Registers (SMR1_n_), (SMR2_n_)|8|R/W|0x00|41.3.1/41-5|
|0x04|Status Register (SSR_n_)|8|R|0x00|41.3.3/41-8|

Address: 0xFC06_0004 (SCSR0) 0xEC06_4004 (SCSR1) 7 6 5 R W Reset 0 0
"""


def test_alternate_registers_at_one_address_are_kept(tmp_path):
    dev, problems = build_device(manual(tmp_path, ("ch41-serial.md", ALT)))
    ser0 = next(p for p in dev.peripherals if p.name == "SER0")
    assert {r.name: r.offset for r in ser0.registers} == {"SMR1": 0, "SMR2": 0, "SSR": 4}
    # a figure naming something else at a mapped address may be an alternate or a misspelling: reported, not added
    assert any(p.startswith("figure name differs") and "SCSR0" in p and "SSR" in p for p in problems)
    assert any(p.startswith("alternate") and "SER0.SMR2" in p for p in problems)


def test_wide_stride_rows_are_one_width_problem(tmp_path):
    dev, problems = build_device(manual(tmp_path, ("ch19-dmx.md", WIDE)))
    assert [p for p in problems if "TCD" in p] == ["width: ch19-dmx TCD 0xfc045000 is 256 bits; left out"]


SHARED_A = """# Chapter 10: Config Module (CFG)

|**Address**|**Register**|**Width**|**Access**|**Reset Value**|**Section/Page**|
|---|---|---|---|---|---|
|0xEC09_0004|Config Register (CFR)|16|R/W|0x0000|10.3.1/10-3|
|0xEC09_0008|Status Register (CSR)|16|R|0x0000|10.3.2/10-4|
"""

SHARED_B = """# Chapter 12: Reset Module (RST)

|**Address**|**Register**|**Width**|**Access**|**Reset Value**|**Section/Page**|
|---|---|---|---|---|---|
|0xEC09_0000|Reset Control Register (RCR)|8|R/W|0x00|12.3.1/12-3|
"""


def test_chapters_sharing_a_block_become_one_peripheral(tmp_path):
    dev, problems = build_device(manual(tmp_path, ("ch10-cfg.md", SHARED_A), ("ch12-rst.md", SHARED_B)))
    ps = [p for p in dev.peripherals if p.base == 0xEC090000]
    assert [p.name for p in ps] == ["CFG"]                       # the one with the most registers names it
    assert {r.name: r.offset for r in ps[0].registers} == {"RCR": 0, "CFR": 4, "CSR": 8}
    assert ps[0].group == "CFG"
    assert "shared block: 0xec090000 CFG, RST merged into CFG" in problems


W1C = """# Chapter 38: Ticker Timers (TK)

|**Address**|**Register**|**Width**|**Access**|**Reset Value**|**Section/Page**|
|---|---|---|---|---|---|
|0xFC09_C000|Status Register (TK_SR)|8|R/W|0x00|38.3.1/38-3|

Address: 0xFC09_C000 (TK_SR)

Access: User read/write

||7|6|5|4|3|2|1|0|
|---|---|---|---|---|---|---|---|---|
|R|0|0|0|0|ERR|0|FLAG|EN|
|W|||||w1c||w1c||
|Reset|0|0|0|0|0|0|0|0|

**Table 38-2. TK_SR Field Descriptions**

|**Field**|**Description**|
|---|---|
|7–4|Reserved|
|3<br>ERR|Error.|
|1<br>FLAG|Event flag.|
|0<br>EN|Enable.|
"""


def test_w1c_cells_of_a_figure_mark_their_fields_one_to_clear(tmp_path):
    dev, _ = build_device(manual(tmp_path, ("ch38-tk.md", W1C)))
    (p,) = [p for p in dev.peripherals if p.name == "TK"]
    assert {f.name: f.one_to_clear for f in p.registers[0].fields} == {"ERR": True, "FLAG": True, "EN": False}


SPI = """# Chapter 40: Serial Port (SPX)

|**Address**|**Register**|**Width**|**Access**|**Reset Value**|**Section/Page**|
|---|---|---|---|---|---|
|0xFC05_C00C|Attribute Register (SPX_CTR)|32|R/W|0x7800_0000|40.3.3/40-9|
|0xFC05_C02C|Status Register (SPX_STS)|32|R/W|0x0000_0000|40.3.4/40-14|
|0xFC05_C030|Enable Register (SPX_IER)|32|R/W|0x0000_0000|40.3.5/40-16|

Address: 0xFC05_C00C (SPX_CTR) Access: User read/write 31 30 R W Reset 0 0

**Table 40-6. SPX_CTR Field Description**

|**Field**|**Description**|
|---|---|
|30–27<br>FSZ|Frame size.|

Address 0xFC05_C02C (SPX_STS) Access: User read/write 31 30 R W Reset 0 0

**Table 40-7. SPX_STS Field Descriptions**

|**Field**|**Description**|
|---|---|
|31<br>DONE|Done.|
|11 or 10<br>ALT|Alternative positions.|

Address: 0xFC05_C030 (SPX_IER) Access: User read/write 31 30 R W Reset 0 0

**Table 40-8. SPX_CTR Field Descriptions**

|**Field**|**Description**|
|---|---|
|0<br>WRONG|A table titled for another register.|
"""


def test_each_figure_takes_its_own_field_table(tmp_path):
    dev, problems = build_device(manual(tmp_path, ("ch40-spx.md", SPI)))
    (p,) = [p for p in dev.peripherals if p.name == "SPX"]
    regs = {r.name: r for r in p.registers}
    assert [(f.name, f.lsb, f.width) for f in regs["CTR"].fields] == [("FSZ", 27, 4)]
    assert [f.name for f in regs["STS"].fields] == ["DONE"]
    # a field table titled for another register is not bound to this one, and says so
    assert "field row unread: ch40-spx SPX_STS 11 or 10<br>ALT" in problems
    assert regs["IER"].fields == []
    assert any(x.startswith("field table names another register:") and "SPX_IER" in x and "SPX_CTR" in x
               for x in problems)


DUP = """# Chapter 8: Clock Module (CLK)

|**Address**|**Register**|**Width**|**Access**|**Reset Value**|**Section/Page**|
|---|---|---|---|---|---|
|0xFC0C_0000|Control Register (CLK_CR)|32|R/W|0x0000_0000|8.2.1/8-3|
|0xFC0C_0008|Status Register (CLK_SR)|32|R/W|0x0000_0000|8.2.3/8-5|

Address: 0xFC0C_0000 (CLK_CR) Access: User read/write 31 30 R W Reset 0 0

**Figure 8-1. Control Register (CLK_CR)**

Address: 0xFC0C_0008 (CLK_SR) 

Access: User read/write

Address: 0xFC0C_0008 (CLK_SR) Access: User read/write 31 30 R W Reset 0 0

**Figure 8-3. Status Register (CLK_SR)**

**Table 8-4. CLK_SR Field Descriptions**

|**Field**|**Description**|
|---|---|
|31–4|Reserved|
|3<br>LOCK|Locked.|
"""


def test_a_repeated_address_line_does_not_hide_the_figures_fields(tmp_path):
    dev, problems = build_device(manual(tmp_path, ("ch08-clk.md", DUP)))
    (p,) = [p for p in dev.peripherals if p.name == "CLK"]
    regs = {r.name: r for r in p.registers}
    assert [f.name for f in regs["SR"].fields] == ["LOCK"]
    # a register whose figure gives no fields is reported
    assert any(x.startswith("no fields:") and "CLK_CR" in x for x in problems)
    assert not any(x.startswith("no fields:") and "CLK_SR" in x for x in problems)


EMPTY = """# Chapter 27: Number Generator (NG)

|**Address**|**Register**|**Width**|**Access**|**Reset Value**|**Section/Page**|
|---|---|---|---|---|---|
|0xFC0A_0000|Data Register (NG_DR)|64|R|0x0|27.3.1/27-3|
"""


def test_a_peripheral_left_without_registers_is_reported(tmp_path):
    dev, problems = build_device(manual(tmp_path, ("ch27-ng.md", EMPTY)))
    assert "NG" not in {p.name for p in dev.peripherals}
    assert "empty peripheral: NG 0xfc0a0000 has no registers; left out" in problems


NO_BITS = """# Chapter 32: Switch (SWX)

|**Address**|**Register**|**Width**|**Access**|**Reset Value**|**Section/Page**|
|---|---|---|---|---|---|
|0xFC0D_C000|Revision (SWX_REV)|32|R|0x0000_0000|32.3.1/32-5|

Address: 0xFC0D_C09C (SWX_P0BCT) Access: User read/write R W Reset
"""


def test_a_figure_only_register_without_bit_numbers_is_reported(tmp_path):
    dev, problems = build_device(manual(tmp_path, ("ch32-swx.md", NO_BITS)))
    (p,) = [p for p in dev.peripherals if p.name == "SWX"]
    assert {r.name: r.width for r in p.registers}["P0BCT"] == 32
    assert "no width: ch32-swx SWX_P0BCT 0xfc0dc09c figure has no bit numbering (32 assumed)" in problems


PREFIXED = """# Chapter 34: Motor Control Pulse Width Modulator (MCPWM)

|**Address**|**Register**|**Width**|**Access**|**Reset Value**|**Section/Page**|
|---|---|---|---|---|---|
|0xEC08_8000|Counter (PWM_CNT)|16|R|0x0000|34.3.1/34-5|
|0xEC08_8002|Control (PWM_CR)|16|R/W|0x0000|34.3.2/34-6|
|0xEC08_8004|Control (CR)|16|R/W|0x0000|34.3.3/34-7|
"""


def test_register_names_drop_their_peripheral_prefix(tmp_path):
    dev, problems = build_device(manual(tmp_path, ("ch40-spx.md", SPI), ("ch34-mcpwm.md", PREFIXED)))
    ps = {p.name: p for p in dev.peripherals}
    assert [r.name for r in ps["SPX"].registers] == ["CTR", "STS", "IER"]      # the group's own prefix
    assert {f.name for f in ps["SPX"].registers[0].fields} == {"FSZ"}
    # MCPWM's registers say PWM_ (a known module prefix); a name the stripped one would collide with keeps it
    assert [r.name for r in ps["MCPWM"].registers] == ["CNT", "PWM_CR", "CR"]
    assert "prefix kept: MCPWM.PWM_CR (CR exists)" in problems


ALT_FIGS = """# Chapter 41: Serial Modules

|**Base Address**|**Module**|**Base Address**|**Module**|
|---|---|---|---|
|0xFC06_0000|SER 0|0xEC06_4000|SER 1|

|**Offset**|**Register**|**Width**|**Access**|**Reset Value**|**Section/Page**|
|---|---|---|---|---|---|
|0x00|Mode Registers (SMR1_n_), (SMR2_n_)|8|R/W|0x00|41.3.1/41-5|

Address: 0xFC06_0000 (SMR10) 0xEC06_4000 (SMR11) Access: User read/write 7 6 R W Reset 0 0

**Table 41-3. SMR1 Field Descriptions**

|**Field**|**Description**|
|---|---|
|7<br>ONE|First mode register's bit.|

Address: 0xFC06_0000 (SMR20) 0xEC06_4000 (SMR21) Access: User read/write 7 6 R W Reset 0 0

**Table 41-4. SMR2 Field Descriptions**

|**Field**|**Description**|
|---|---|
|6<br>TWO|Second mode register's bit.|
"""


def test_register_numbers_tell_alternates_apart(tmp_path):
    # SMR10 is SMR1 of SER0 (its instance number appended), SMR20 is SMR2 of SER0: digits other than the instance's
    # own number count, so SMR2 takes SMR2's figure and not SMR1's
    dev, problems = build_device(manual(tmp_path, ("ch41-serial.md", ALT_FIGS)))
    for p in (p for p in dev.peripherals if p.group == "SER"):
        regs = {r.name: [f.name for f in r.fields] for r in p.registers}
        assert regs == {"SMR1": ["ONE"], "SMR2": ["TWO"]}, p.name
    assert not any(x.startswith("figure name differs") for x in problems)


ODD = """# Chapter 27: Widget (WG)

|**Address**|**Register**|**Access**|**Reset Value**|**Section/Page**|
|---|---|---|---|---|
|0xFC0A_0000|Widget control (WG_CR)|R/W|See Section|27.3.1/27-3|

Address: 0xFC0F_0000 (WG_FAR) Access: User read/write 7 6 R W Reset 0 0
"""


def test_a_map_row_without_any_width_is_reported(tmp_path):
    # no width cell, a reset value that gives none, and no figure: 32 bits assumed, and said so
    dev, problems = build_device(manual(tmp_path, ("ch27-wg.md", ODD)))
    (p,) = [p for p in dev.peripherals if p.name == "WG"]
    assert {r.name: r.width for r in p.registers} == {"CR": 32}
    assert "no width: ch27-wg WG_CR 0xfc0a0000 (32 assumed)" in problems


def test_a_figure_outside_the_chapters_blocks_says_it_is_left_out(tmp_path):
    dev, problems = build_device(manual(tmp_path, ("ch27-wg.md", ODD)))
    assert "no map row: ch27-wg WG_FAR 0xfc0f0000 outside the chapter's peripherals; left out" in problems
    assert not any(r.name.endswith("FAR") for p in dev.peripherals for r in p.registers)
