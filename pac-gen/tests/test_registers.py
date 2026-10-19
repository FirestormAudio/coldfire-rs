from rm2svd.registers import access_of, figures, memory_map_rows, norm_abbrev

TABLE = """**Table 5-2. Timer Memory Map**

|**Address**|||||
|---|---|---|---|---|
|**T0**<br>**T1**|**Register**<br>**Width**<br>**(bits)**|**Access**|**Reset Value**|**Section/Page**|
|0xFC08_0000<br>0xFC08_4000|Timer Control Register (TCR_n_)<br>16|R/W|0x0000|5.2.1/5-3|
"""

PICTURE = """<!-- Start of picture text -->
Address<br>T0 Width<br>Register Access Reset Value Section/Page<br>0xFC08_0002 Timer Modulus Register (TMR n ) 16 R/W 0xFFFF 5.2.2/5-5<br>0xFC08_4002<br>0xFC08_0004 Timer Count Register (TCNT n ) 16 R 0xFFFF 5.2.3/5-5<br>0xFC08_4004<br><!-- End of picture text -->
"""

FIG = """Address: 0xFC00_8000 (CSAR0) Access: User read/write 0xFC00_800C (CSAR1) 0xFC00_8018 (CSAR2) 31 30 R 0 0 BA W w1c w1c Reset 0 0
"""


def test_multi_instance_rows_give_one_peripheral_each():
    (r,) = memory_map_rows(TABLE)
    assert (r.abbrev, r.addresses, r.width, r.access, r.reset) == ("TCRn", [0xFC080000, 0xFC084000], 16, "read-write", 0)


def test_picture_text_rows_parse_like_table_rows():
    rows = memory_map_rows(PICTURE)
    assert [(r.abbrev, r.addresses, r.width, r.access, r.reset) for r in rows] == [
        ("TMRn", [0xFC080002, 0xFC084002], 16, "read-write", 0xFFFF),
        ("TCNTn", [0xFC080004, 0xFC084004], 16, "read-only", 0xFFFF)]


def test_figure_lists_every_instance_address():
    (f,) = figures(FIG)
    assert f.addresses == [(0xFC008000, "CSAR0"), (0xFC00800C, "CSAR1"), (0xFC008018, "CSAR2")]
    assert f.access == "read-write"


def test_abbrev_and_access_normalised():
    assert norm_abbrev("PCSR_n_") == norm_abbrev("PCSR n ") == "PCSRn" and norm_abbrev("EDMA_CR") == "EDMA_CR"
    assert (access_of("R/W"), access_of("R"), access_of("W"), access_of("R/W1C")) == \
        ("read-write", "read-only", "write-only", "read-write")


# --- real-manual shapes. Synthetic rows in the manual's table shapes. ---
from rm2svd.registers import instance_bases, instance_headers, memory_map_rows_with_problems  # noqa: E402

WIDTH_CELL = """|**Address**|**Register**|**Width**<br>**(bits)**|**Access**|**Reset Value**|**Section/Page**|
|---|---|---|---|---|---|
||**Supervisor Access Only Registers**||||
|0xFC0C_0000|Widget Control Register (WG_CR)|32|R/W|0x0000_0000|8.2.1/8-6|
|0xFC0C_0004|Widget Enable High Register (WG_ENH,<br>Channels 63-32)|8|W<sup>2</sup>|0x00|8.2.2/8-8|
|0xEC09_4048|WG_PAR||8|R/W<br>See Note|See Section|8.2.3/8-9|
"""


def test_width_in_its_own_cell():
    rows = memory_map_rows(WIDTH_CELL)
    assert [(r.abbrev, r.addresses, r.width, r.access, r.reset, r.section) for r in rows] == [
        ("WG_CR", [0xFC0C0000], 32, "read-write", 0, 8),
        ("WG_ENH", [0xFC0C0004], 8, "write-only", 0, 8),
        ("WG_PAR", [0xEC094048], 8, "read-write", None, 8)]


NO_WIDTH = """|**Address**|**Register**|**Access**|**Reset Value**|**Section/Page**|
|---|---|---|---|---|
|0xFC05_8000<br>0xFC03_8000|I<sup>2</sup>C Address Register (I2ADR_n_)|R/W|0x00|42.2.1/42-5|
|0xFC0A_C008|Port detect register (PD_DET1)|R/W|0x0000_----|28.3.3/28-6|
|0xFC0A_C00C|Port status register (PD_STAT)|R|See Section|28.3.4/28-7|
"""


def test_no_width_cell_takes_width_from_the_reset_value():
    rows = memory_map_rows(NO_WIDTH)
    assert [(r.abbrev, r.addresses, r.width, r.access) for r in rows] == [
        ("I2ADRn", [0xFC058000, 0xFC038000], 8, "read-write"),
        ("PD_DET1", [0xFC0AC008], 32, "read-write"),
        ("PD_STAT", [0xFC0AC00C], None, "read-only")]


EXTRA_CELLS = """|0xFC0B_0000|Identification Register (ID)<br>N|H/D|32|R|0xE241_FA05|24.3.1.1/24-8|
|0xFC0B_4004|General Parameters (HWGENERAL)|N|32|R|0x0000_07C5|24.3.1.2/24-9|
|0xFC02_0004<br>0xFC02_4004|Control Register<br>(CANCTRL_n_)|32|Y|N|R/W|0x0000_0000|33.3.2/33-9|
"""


def test_extra_cells_between_name_and_width_or_access():
    rows = memory_map_rows(EXTRA_CELLS)
    assert [(r.abbrev, r.addresses, r.width, r.access, r.reset) for r in rows] == [
        ("ID", [0xFC0B0000], 32, "read-only", 0xE241FA05),
        ("HWGENERAL", [0xFC0B4004], 32, "read-only", 0x07C5),
        ("CANCTRLn", [0xFC020004, 0xFC024004], 32, "read-write", 0)]


STRIDE = """|0xFC00_8000<br>+ (_n_0xC)|Chip-Select Address Register (CSAR_n_)<br>_n_= 0 – 2|32|R/W|0x0000_0000|20.3.1/20-6|
|0xFC05_C00C<br>+ (_n_ 0x04)<br>0xFC03_C00C<br>+ (_n_ 0x04)|Transfer attributes registers<br>(SP_x__CTAR_n_),_n_=0:1|32|R/W|0x7800_0000|40.3.3/40-9|
|0xFC04_8040 +_n_<br>(_n_=0:2)|Control Registers (ICR0_n_)|8|R/W|0x00|17.2.9/17-12|
|0xFC04_80E0 + 4_n_|Level_n_Acknowledge Registers (L_n_IACK0)|8|R|0x18|17.2.10/17-19|
|(_n_=1:2)||||||
||**Submodule****_n_ registers (****_n_  **|**= 0–1)**||||
|0xEC08_8000<br>+ (_n_ \uf0b40x50)|Counter register (PW_SM_n_CNT)|16|R|0x0000|34.3.1/34-7|
"""


def test_stride_rows_expand_to_one_row_per_index():
    rows = memory_map_rows(STRIDE)
    assert [(r.template, r.addresses) for r in rows] == [
        ("CSAR0", [0xFC008000]), ("CSAR1", [0xFC00800C]), ("CSAR2", [0xFC008018]),
        ("SP{x}_CTAR0", [0xFC05C00C, 0xFC03C00C]), ("SP{x}_CTAR1", [0xFC05C010, 0xFC03C010]),
        ("ICR00", [0xFC048040]), ("ICR01", [0xFC048041]), ("ICR02", [0xFC048042]),
        ("L1IACK0", [0xFC0480E4]), ("L2IACK0", [0xFC0480E8]),
        ("PW_SM0CNT", [0xEC088000]), ("PW_SM1CNT", [0xEC088050])]
    assert all(r.array for r in rows)


RELATIVE = """|**UART Base**<br>**Offset**|**Register**|**Width**<br>**(bit)**|**Access **|**Reset Value**|**Section/Page**|
|---|---|---|---|---|---|
|0x00|Mode Registers<sup>1</sup>(UMR1_n_), (UMR2_n_)|8|R/W|0x00|41.3.1/41-5|
|0x04|Status Register (USR_n_)|8|R|0x00|41.3.3/41-8|
||Clock Select Register<sup>1</sup>(UCSR_n_)|8|W|See Section|41.3.4/41-9|
"""


def test_offset_rows_are_relative_and_blank_address_rows_share_the_previous_address():
    rows = memory_map_rows(RELATIVE)
    assert [(r.template, r.addresses, r.relative, r.access) for r in rows] == [
        ("UMR1{n}", [0], True, "read-write"), ("UMR2{n}", [0], True, "read-write"),
        ("USR{n}", [4], True, "read-only"),
        ("UCSR{n}", [4], True, "write-only")]


RANGE = """|0xEC09_4000–<br>0xEC09_4002|Port output data registers (PODR_A – C)<br>8|R/W|0xFF|15.3.1/15-14|
|0xEC09_4030–<br>0xEC09_4034|Pull control registers (PCR_A – C)<br>16|R/W|See Section|15.3.5/15-17|
|0xFC0F_C000<br>–<br>0xFC0F_F8FF|SRAM buffer|64|R/W|—|22.4.1/22-15|
"""


def test_address_ranges_with_letter_ranges_expand():
    rows = memory_map_rows(RANGE)
    assert [(r.template, r.addresses, r.width) for r in rows] == [
        ("PODR_A", [0xEC094000], 8), ("PODR_B", [0xEC094001], 8), ("PODR_C", [0xEC094002], 8),
        ("PCR_A", [0xEC094030], 16), ("PCR_B", [0xEC094032], 16), ("PCR_C", [0xEC094034], 16)]


BASES = """|**Base Address**|**Module**|**Base Address**|**Module**|
|---|---|---|---|
|0xFC06_0000|SER 0|0xEC06_4000|SER 2|
|0xFC06_4000|SER 1|||
|**Number**|**Base Address**|
|---|---|
|CTL0|0xFC04_8000|
|Global Space<sup>1</sup>|0xFC05_4000|
|0xFC0D_4000|Configuration|Core control and status registers|
"""


def test_instance_base_tables():
    assert instance_bases(BASES) == [("SER0", 0xFC060000), ("SER2", 0xEC064000), ("SER1", 0xFC064000),
                                     ("CTL0", 0xFC048000)]


def test_instance_header_cells():
    assert instance_headers(TABLE) == ["T0", "T1"]
    assert instance_headers("|**Address**<br>**NET0**<br>**NET1**|**Register**|\n") == ["NET0", "NET1"]
    assert instance_headers("|**PIT 0**<br>**PIT 1**|**Register**|\n") == ["PIT0", "PIT1"]
    assert instance_headers("|**I**<sup>**2**</sup>**C0**<br>**...**<br>**I**<sup>**2**</sup>**C5**|x|\n") == []


FIG_BR = """Address: 0xFC08_0000 (PCSR0) Access: Supervisor<br>0xFC08_4000 (PCSR1) read/write<br>15 14 13 12 R 0 0 W Reset 0 0
"""


def test_figure_addresses_separated_by_line_breaks():
    (f,) = figures(FIG_BR)
    assert f.addresses == [(0xFC080000, "PCSR0"), (0xFC084000, "PCSR1")]
    assert f.access == "read-write" and f.width == 16
    (g,) = figures("Address: 0xFC04_801A (ICONFIG) 7 6 5 4 R 0 W Reset 0\n")
    assert g.access is None and g.width == 8


def test_stride_without_parentheses():
    rows = memory_map_rows("||**Port_n_ registers (_n_ = 0–1)**||||\n"
                           "|0xFC0D_C310<br>+_n_0x10|Port_n_congestion (SW_P_n_OQC)|32|R|0x0000_0000|32.3.35/32-29|\n")
    assert [(r.template, r.addresses) for r in rows] == [("SW_P0OQC", [0xFC0DC310]), ("SW_P1OQC", [0xFC0DC320])]


def test_numeric_ranges_in_the_abbreviation_expand_by_width():
    rows, problems = memory_map_rows_with_problems(
        "|0xFC0D_C0C0|Snoop registers (SW_PSNP1–3)|32|R/W|0x0000_0000|32.3.27/32-24|\n"
        "|0xFC02_0080<br>0xFC02_4080|Message Buffers 0–15 (MB0–15)|2048|N|N|R/W|—|33.3.9/33-19|\n")
    assert [(r.template, r.addresses, r.index) for r in rows] == [
        ("SW_PSNP1", [0xFC0DC0C0], "1"), ("SW_PSNP2", [0xFC0DC0C4], "2"), ("SW_PSNP3", [0xFC0DC0C8], "3")]
    assert any("MB0–15" in p for p in problems)


def test_stride_multiply_sign_in_symbol_font():
    # the converter keeps the PDF's Symbol-font multiply sign and space as private-use characters
    rows = memory_map_rows("|0xFC05_C00C<br>+ (_n_ 0x04)<br>0xFC03_C00C<br>+ (_n_ 0x04)|"
                           "Attributes (SP_x__CTAR_n_),_n_=0:1|32|R/W|0x7800_0000|40.3.3/40-9|\n")
    assert [(r.template, r.addresses) for r in rows] == [("SP{x}_CTAR0", [0xFC05C00C, 0xFC03C00C]),
                                                         ("SP{x}_CTAR1", [0xFC05C010, 0xFC03C010])]


def test_blank_address_rows_only_continue_the_row_right_above():
    rows, problems = memory_map_rows_with_problems(
        "|0x04|Status Register (USR_n_)|8|R|0x00|41.3.3/41-8|\n\nSome prose.\n\n"
        "|**A**|**B**|**C**|**D**|\n|---|---|---|---|\n||**SIM_RCR**<br>**[KILL]**|**SIM_CR**|R|0x00|\n")
    assert [r.template for r in rows] == ["USR{n}"] and problems == []


# --- fix round 1: nothing is dropped silently ---

def test_every_name_in_a_map_cell_is_a_row():
    rows = memory_map_rows("|0x00|Mode Registers<sup>1</sup>(UMR1_n_), (UMR2_n_)|8|R/W|0x00|41.3.1/41-5|\n")
    assert [(r.template, r.addresses) for r in rows] == [("UMR1{n}", [0]), ("UMR2{n}", [0])]


def test_hex_strides():
    rows, problems = memory_map_rows_with_problems(
        "|0xFC04_4100<br>+ hex(_n_)|Channel_n_Priority Register (DCPR_n_)<br>for_n_= 0 – 2|8|R/W|See Section|19.4.16/19-23|\n"
        "|0xFC04_5000<br>+ hex(32\uf0b4_n_)|Transfer Control Descriptor (TCD_n_)<br>for_n_= 0 – 63|256|R/W|See Section|19.4.17/19-24|\n")
    assert [(r.template, r.addresses, r.width) for r in rows] == [
        ("DCPR0", [0xFC044100], 8), ("DCPR1", [0xFC044101], 8), ("DCPR2", [0xFC044102], 8),
        ("TCD{n}", [0xFC045000], 256)]                  # left whole: build reports its width for the TCD cluster
    assert problems == []


def test_strides_not_understood_are_reported():
    rows, problems = memory_map_rows_with_problems(
        "|0xFC04_4100<br>+ off(_n_)|Thing Register (THG_n_)|8|R/W|0x00|19.4.16/19-23|\n"
        "|0xFC04_4200|Other Register (OTH_n_)<br>_n_= 0 – 3|8|R/W|0x00|19.4.16/19-23|\n")
    assert rows == []
    assert [p.split(":")[0] for p in problems] == ["stride not understood", "stride not understood"]
    assert "THG_n_" in problems[0] and "OTH_n_" in problems[1]


def test_address_rows_without_an_abbreviation_are_reported():
    rows, problems = memory_map_rows_with_problems(
        "|0xFC0E_0000<br>–<br>0xFC0E_3FFC|MAC address lookup table for the switch ports|32|RW|Undefined|32.3.45/32-35|\n"
        "|0xFC0B_80B8|Reserved|32|—|—|21.4.47/21-38|\n"
        "|0xFC0F_0000||||||\n"
        "|0xFC06_0000|UART 0|0xEC06_4000|UART 5|\n"
        "|0x8000_0000|Internal SRAM region|Yes|Yes|\n")              # a region list, no access column
    assert rows == []
    assert problems == ["unread map row: 0xfc0e0000 MAC address lookup table for",
                        "unread map row: 0xfc0f0000 no register in the row"]


TWO_FIGURES = """Address: 0xFC04_401F (EDMA_CDNE) Access: User write-only

||7|6|5|4|3|2|1|0|
|---|---|---|---|---|---|---|---|---|
|R|0|0|0|0|0|0|0|0|
|W|NOP|CADN|CDNE|||||
|Reset|0|0|0|0|0|0|0|0|

**Figure 19-15. Clear DONE Status Bit Register (EDMA_CDNE)**

**Table 19-14. EDMA_CDNE Field Descriptions**

|**Field**|**Description**|
|---|---|
|7<br>NOP|No operation.|

Offset 0xFC04_4020 (EDMA_INTH)

||7|6|5|4|3|2|1|0|
|---|---|---|---|---|---|---|---|---|
|R|INT7|INT6|INT5|INT4|INT3|INT2|INT1|INT0|
|W|w1c|w1c|w1c|w1c|w1c|w1c|w1c|w1c|
|Reset|0|0|0|0|0|0|0|0|

**Figure 19-17. eDMA Interrupt Request High Register (EDMA_INTH)**
"""


def test_w1c_cells_come_from_the_figures_own_diagram_only():
    f, g = figures(TWO_FIGURES)                           # the second is headed `Offset` (a figure too)
    assert f.addresses == [(0xFC04401F, "EDMA_CDNE")]
    assert f.one_to_clear_bits == set()
    assert g.addresses == [(0xFC044020, "EDMA_INTH")] and g.one_to_clear_bits == set(range(8))


def test_w1c_cells_without_a_caption_stop_at_the_field_table():
    f, _ = figures(TWO_FIGURES.replace("**Figure 19-15. Clear DONE Status Bit Register (EDMA_CDNE)**", ""))
    assert f.one_to_clear_bits == set()


NO_COLON = """Address: 0xFC05_C00C (SPI0_CTR) Access: User read/write 31 30 R W Reset 0 0

**Table 40-6. SPI_CTR Field Description**

|**Field**|**Description**|
|---|---|
|31<br>DBL|Doubler.|

Address 0xFC05_C02C (SPI0_STS) Access: User read/write<br>: 0xFC03_C02C (SPI1_STS) 31 30 R W Reset 0 0

**Table 40-7. SPI_STS Field Descriptions**

Offset 0xFC05_C030 (SPI0_IER) 31 30 R W Reset 0 0
"""


def test_address_without_a_colon_or_headed_offset_starts_a_figure():
    fs = figures(NO_COLON)
    assert [f.addresses for f in fs] == [[(0xFC05C00C, "SPI0_CTR")],
                                         [(0xFC05C02C, "SPI0_STS"), (0xFC03C02C, "SPI1_STS")],
                                         [(0xFC05C030, "SPI0_IER")]]
    assert "SPI_STS" not in fs[0].text_after                # the next figure ends this one's text


# --- deferred minors ---

def test_figure_access_words_in_any_case():
    for text, want in (("Access: User Read/Write", "read-write"), ("Access: Supervisor Write-only", "write-only"),
                       ("Access: User READ-ONLY", "read-only"), ("Access: User read only", "read-only"),
                       ("Access: Supervisor read-write", "read-write")):
        (f,) = figures(f"Address: 0xFC08_0000 (PCSR0) {text} 15 14 R 0 W Reset 0 0\n")
        assert f.access == want, text


def test_unknown_access_is_not_read_write():
    import pytest
    for s in ("—", "Undefined", "N"):
        with pytest.raises(ValueError, match="access"):
            access_of(s)
    assert access_of(" rw ") == "read-write"


def test_unknown_access_in_a_picture_row_is_reported():
    rows, problems = memory_map_rows_with_problems(
        "<!-- Start of picture text -->\n0xFC08_0000 Timer Mode Register (TMD) 16 R/Wx 0x0000 5.2.1/5-3<br>"
        "<!-- End of picture text -->\n")
    assert rows == []
    assert problems == ["access not understood: TMD 'R/Wx'"]


def test_single_instance_picture_row():
    (r,) = memory_map_rows("<!-- Start of picture text -->\nAddress Register Width Access Reset<br>"
                           "0xFC0A_0000 Widget Status Register (WG_SR) 32 R 0x0000_0001 27.3.1/27-3<br>"
                           "<!-- End of picture text -->\n")
    assert (r.abbrev, r.addresses, r.width, r.access, r.reset, r.array) == \
        ("WG_SR", [0xFC0A0000], 32, "read-only", 1, False)


def test_unread_rows_say_their_address():
    # a name in parentheses but no access cell this parser knows: the problem line gives the row's address
    _, problems = memory_map_rows_with_problems("|0xFC0A_0000|Widget Register (WG_R)|32|RO*|0x00|27.3.1/27-3|\n")
    assert problems == ["unread map row: 0xfc0a0000 WG_R"]


def test_rows_without_a_name_need_an_access_or_a_reset_cell():
    # no abbreviation and an access cell this parser doesn't know: a reset value still marks a register row (an 8-
    # or 16-bit one: a 32-bit value reads like the address of a base-address table)
    rows, problems = memory_map_rows_with_problems(
        "|0xFC0A_0000|Widget lookup table|16|RO*|0x0000|27.3.1/27-3|\n"
        "|0x8000_0000|Internal SRAM region|Yes|Yes|\n")
    assert rows == [] and problems == ["unread map row: 0xfc0a0000 Widget lookup table"]


def test_address_rows_with_nothing_else_are_reported():
    rows, problems = memory_map_rows_with_problems("|0xEC0A_0010<br>0xEC0A_4010|||||\n")
    assert rows == [] and problems == ["unread map row: 0xec0a0010 no register in the row"]
