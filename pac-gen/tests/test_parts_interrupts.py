import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))   # pac-gen/: `import rm2svd`
from rm2svd.interrupts import interrupts
from rm2svd.parts import availability

PARTS = """|**Module**|**MCF54410**|**MCF54415**|**MCF54416**|**MCF54417**|**MCF54418**|
|---|---|---|---|---|---|
|Timer block (TMR)|—|||||
|UARTs|6|10|10|10|10|
|Smart card (SIM)|—|2 ports|2 ports|2 ports|2 ports|
"""

INTC = """**Table 17-15. Interrupt Source Assignment For INTC0**

|**Source**|**Module**|**Flag**|**Source Description**|**Flag Clearing Mechanism**|
|---|---|---|---|---|
|0|||Not Used||
|1||EPFR0[EPF1]|Edge flag 1|Write 1|
|2|EPORT<br>0|EPFR0[EPF2]|Edge flag 2|Write 1|
|3||TSR[TXF]|Timer frame|Write 1|

**Table 17-16. Interrupt Source Assignment for INTC1**

|**Source**|**Module**|**Flag**|**Source Description**|**Flag Clearing Mechanism**|
|---|---|---|---|---|
|5|UART1|USR[TXF]|Transmit|Write|
"""


def test_availability_reads_dashes_blanks_counts_and_ports():
    a = availability(PARTS)
    assert a["mcf54410"]["Timer block (TMR)"] == 0 and a["mcf54418"]["Timer block (TMR)"] == 1
    assert a["mcf54410"]["UARTs"] == 6 and a["mcf54415"]["Smart card (SIM)"] == 1     # 2 ports: one module
    assert "---" not in a["mcf54410"]


def test_interrupt_names_are_unique_and_vectors_offset():
    irqs = interrupts(INTC)
    assert [(i.name, i.value) for i in irqs] == [("EPFR0_EPF1", 65), ("EPFR0_EPF2", 66), ("TSR_TXF", 67),
                                                 ("USR_TXF", 133)]
    assert len({i.value for i in irqs}) == len(irqs)


INTC_FIX = """**Table 17-15. Interrupt Source Assignment For INTC0**

|**Source**|**Module**|**Flag**|**Source Description**|**Flag Clearing Mechanism**|
|---|---|---|---|---|
|0|DTIM0|DTER0 register|Timer 0|Write 1|
|1|Flex|ERRSTAT0<br>[BOFFINT]|Bus-off|Write 1|
|2|EPORT<br>0|EPFR0[EPF2]|Edge|Write 1|
|3||ERRSTAT0[TXWRN<br>or RXWRN]|Error|Write 1|
|4|DMA<br>56–63|EDMA_INTR<br>[INT56–63]|DMA OR|Write|
|5||FOO BAR|Mystery|Write|
|6|Gone||Not Used||

**Table 17-16. Interrupt Source Assignment for INTC1**

|**Source**|**Module**|**Flag**|**Source Description**|**Flag Clearing Mechanism**|
|---|---|---|---|---|
|0||TSR[TXF]|Timer|Write 1|
"""


def test_br_split_register_only_and_range_flags_are_named():
    irqs = {i.name: i for i in interrupts(INTC_FIX)}
    assert irqs["DTER0"].value == 64 and irqs["DTER0"].peripheral == "DTIM0"
    assert irqs["ERRSTAT0_BOFFINT"].value == 65 and irqs["ERRSTAT0_BOFFINT"].peripheral == "Flex"
    assert irqs["ERRSTAT0_TXWRN"].value == 67          # "TXWRN<br>or RXWRN" uses its first field
    assert irqs["EDMA_INTR_INT56_63"].value == 68 and irqs["EDMA_INTR_INT56_63"].peripheral == "DMA56–63"


def test_unnameable_flag_is_reported_as_problem():
    from rm2svd.interrupts import interrupts_with_problems
    irqs, problems = interrupts_with_problems(INTC_FIX)
    assert "FOO BAR" in " ".join(problems)
    assert len(problems) == 1                          # the Not Used row and the named rows are not problems
    assert 69 not in {i.value for i in irqs}           # the unnameable row is not an interrupt


def test_module_resets_between_intc_tables():
    irqs = {i.name: i for i in interrupts(INTC_FIX)}
    assert irqs["TSR_TXF"].value == 128 and irqs["TSR_TXF"].peripheral == ""


INTC_CONT = """**Table 17-15. Interrupt Source Assignment For INTC0**

|**Source**|**Module**|**Flag**|**Source Description**|**Flag Clearing Mechanism**|
|---|---|---|---|---|
|0|Gone|DTER0 register|Timer 0|Write 1|

**Table 17-15. Interrupt Source Assignment For INTC0 (continued)**

|**Source**|**Module**|**Flag**|**Source Description**|**Flag Clearing Mechanism**|
|---|---|---|---|---|
|1||EPFR0[EPF1]|Edge|Write 1|
|2||USB_STS<br>|Status|Write 1|
"""


def test_module_label_carries_across_continued_header_of_same_intc():
    irqs = {i.name: i for i in interrupts(INTC_CONT)}
    assert irqs["EPFR0_EPF1"].value == 65 and irqs["EPFR0_EPF1"].peripheral == "Gone"


def test_single_identifier_flag_names_the_interrupt():
    irqs = {i.name: i for i in interrupts(INTC_CONT)}
    assert irqs["USB_STS"].value == 66 and irqs["USB_STS"].peripheral == "Gone"


def test_availability_is_keyed_by_the_parts_maps_short_names():
    from rm2svd.parts import keyed
    groups = {"TMR": {"match": "(TMR)"}, "UART": {"match": "UARTs"}, "SIM": {"match": "(SIM)"},
              "GONE": {"match": "Gone"}}
    a, problems = keyed(availability(PARTS + "|I<sup>2</sup>C bus|4|6|6|6|6|\n"), groups)
    assert a["mcf54410"] == {"TMR": 0, "UART": 6, "SIM": 0} and a["mcf54418"]["SIM"] == 1
    # the manual's row labels are never copied: an unmatched row is reported by its position
    assert problems == ["availability: Table 1-1 row 4 matches no parts_map.json group",
                        "availability: parts_map.json group GONE matches no Table 1-1 row"]


def test_availability_cells_that_are_not_counts():
    from rm2svd.parts import availability_with_problems
    a, problems = availability_with_problems(PARTS + "|Static RAM (SRAM)|||64 Kbytes|||\n"
                                                     "|Package|196<br>MAPBGA||2<br>MAP|56<br>BGA||\n"
                                                     "|Timer block (TMQ)|–|||||\n"
                                                     "|Clock (CLK)|\uf0b7|\uf0b7|\uf0b7|\uf0b7|\uf0b7|\n")
    # a number with a unit (ports, Kbytes, a package name) counts something else: the module is present (1)
    assert a["mcf54415"]["Smart card (SIM)"] == 1 and a["mcf54416"]["Static RAM (SRAM)"] == 1
    assert [a[p]["Package"] for p in ("mcf54410", "mcf54415", "mcf54416", "mcf54417")] == [1, 1, 1, 1]
    assert a["mcf54410"]["Timer block (TMQ)"] == 0                         # an en dash is absent too
    assert a["mcf54410"]["UARTs"] == 6 and a["mcf54410"]["Clock (CLK)"] == 1       # a checkmark: present, not reported
    assert problems == ["availability: Table 1-1 row 3: cells that aren't a count taken as present",
                        "availability: Table 1-1 row 4: cells that aren't a count taken as present",
                        "availability: Table 1-1 row 5: cells that aren't a count taken as present"]


def test_availability_without_table_1_1_is_reported():
    from rm2svd.parts import availability_with_problems
    a, problems = availability_with_problems("# Chapter 1: Overview\n\nNo table here.\n")
    assert all(v == {} for v in a.values()) and problems == ["availability: Table 1-1 not found"]


def test_availability_reads_every_page_of_table_1_1():
    a = availability(PARTS + "\nPage footer.\n\n" + PARTS.splitlines()[0] + "\n|---|---|---|---|---|---|\n"
                     "|Watchdog timer||||||\n")
    assert a["mcf54418"]["Watchdog timer"] == 1 and a["mcf54410"]["UARTs"] == 6


INTC_DUP = """**Table 17-15. Interrupt Source Assignment For INTC0**

|**Source**|**Module**|**Flag**|**Source Description**|**Flag Clearing Mechanism**|
|---|---|---|---|---|
|1|TMR|TSR[TXF]|Timer frame|Write 1|
|2|TMR|TSR[TXF]|Timer frame again|Write 1|
"""


def test_a_repeated_flag_gets_its_vector_as_suffix():
    assert [(i.name, i.value) for i in interrupts(INTC_DUP)] == [("TSR_TXF", 65), ("TSR_TXF_66", 66)]
