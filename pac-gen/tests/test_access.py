"""A field never has more access than its register, unless the figure's W (or R) row shows cells for it."""
from rm2svd.build import build_device
from rm2svd.fields import figure_rows


def chapter(rows, figures):
    """rows: [(address, name, map access)] (16-bit, one instance each); figures: the text after the memory map."""
    maps = "\n".join(f"|{a}|{n} Register ({n})<br>16|{acc}|0x0000|7.2.1/7-3|" for a, n, acc in rows)
    return f"""# Chapter 7: Gauges (GAU)

**Table 7-2. Gauge Memory Map**

|**Address**|||||
|---|---|---|---|---|
|**G0**|**Register**<br>**Width**<br>**(bits)**|**Access**|**Reset Value**|**Section/Page**|
{maps}

{figures}
"""


def fig(addr, name, diagram, field, access="User read/write"):
    return f"""Address: {addr} ({name}) Access: {access} 15 14 {diagram}

**Table 7-3. {name} Field Descriptions**

|**Field**|**Description**|
|---|---|
|0<br>{field}|x|

"""


def build(tmp_path, rows, figures):
    (tmp_path / "ch07-gauges.md").write_text(chapter(rows, figures))
    (tmp_path / "ch01-overview.md").write_text("")
    (tmp_path / "ch17-interrupt-controller-modules-intc.md").write_text("")
    dev, problems = build_device(tmp_path)
    regs = {r.name: r for p in dev.peripherals for r in p.registers}
    return regs, problems


def test_figure_rows_picture_text_and_table():
    diagram = "15 14 R 0 A B W w1c Reset 0 0"
    assert figure_rows(diagram) == (["0", "A", "B"], ["w1c"])
    table = "||15|14|\n|---|---|---|\n|R<br>(X_n_)|A|0|\n|W<br>(Y_n_)||B|\n|Reset:|0|0|"
    assert figure_rows(table) == (["(X_n_)", "A", "0"], ["(Y_n_)", "B"])
    assert figure_rows("no rows here") is None


def test_blank_w_row_makes_the_field_read_only(tmp_path):
    regs, problems = build(tmp_path, [("0xFC10_0000", "FLG", "R")],
                           fig("0xFC10_0000", "FLG", "R 0 FLG W Reset 0 0", "FLG"))
    assert regs["FLG"].access == "read-only"
    assert [f.access for f in regs["FLG"].fields] == ["read-only"]
    assert any(p.startswith("access:") and "FLG" in p for p in problems)


def test_w_row_cells_make_the_register_read_write(tmp_path):
    regs, problems = build(tmp_path, [("0xFC10_0000", "FLG", "R")],
                           fig("0xFC10_0000", "FLG", "R 0 FLG W w1c Reset 0 0", "FLG"))
    assert regs["FLG"].access == "read-write"
    assert [f.access for f in regs["FLG"].fields] == ["read-write"]
    assert any(p.startswith("access:") and "FLG" in p for p in problems)


def test_a_write_only_figure_gives_a_write_only_register(tmp_path):
    regs, _ = build(tmp_path, [("0xFC10_0000", "ENA", "R")],
                    fig("0xFC10_0000", "ENA", "R 0 0 W E N Reset 0 0", "ENA", "User write-only"))
    assert regs["ENA"].access == "write-only"
    assert [f.access for f in regs["ENA"].fields] == ["write-only"]


def test_a_write_only_register_with_zero_read_cells_has_write_only_fields(tmp_path):
    regs, _ = build(tmp_path, [("0xFC10_0000", "CLR", "W")],
                    fig("0xFC10_0000", "CLR", "R 0 0 W CLR Reset 0 0", "CLR"))
    assert regs["CLR"].access == "write-only"
    assert [f.access for f in regs["CLR"].fields] == ["write-only"]


def test_read_cells_make_a_write_only_register_read_write(tmp_path):
    regs, _ = build(tmp_path, [("0xFC10_0000", "CLR", "W")],
                    fig("0xFC10_0000", "CLR", "R 0 CLR W CLR Reset 0 0", "CLR"))
    assert regs["CLR"].access == "read-write"


def test_the_tail_of_a_label_wrapped_onto_the_w_row_is_not_a_cell(tmp_path):
    regs, _ = build(tmp_path, [("0xFC10_0000", "REC", "R")],
                    fig("0xFC10_0000", "REC", "R 0 SW HASH W PORT Reset 0 0", "SWPORT"))
    assert regs["REC"].access == "read-only"
    assert [f.access for f in regs["REC"].fields] == ["read-only"]


def test_a_status_mask_pair_sharing_an_address_keeps_each_access(tmp_path):
    rows = [("0xFC10_0000", "STA", "R"), ("", "MSK", "W")]
    table = """Address: 0xFC10_0000 (STA0) Access: User read/write 15 14

||15|14|
|---|---|---|
|R<br>(STA_n_)|DB|0|
|W<br>(MSK_n_)|DB|0|
|Reset:|0|0|

**Table 7-3. STA/MSK Field Descriptions**

|**Field**|**Description**|
|---|---|
|15<br>DB|x|
"""
    regs, _ = build(tmp_path, rows, table)
    assert regs["STA"].access == "read-only" and regs["MSK"].access == "write-only"
    assert [f.access for f in regs["STA"].fields] == ["read-only"]
    assert [f.access for f in regs["MSK"].fields] == ["write-only"]


def test_a_figure_without_rows_clamps_the_fields_and_says_so(tmp_path):
    regs, problems = build(tmp_path, [("0xFC10_0000", "FLG", "R")],
                           fig("0xFC10_0000", "FLG", "FLG Reset 0 0", "FLG"))
    assert [f.access for f in regs["FLG"].fields] == ["read-only"]
    assert any(p.startswith("access:") and "rows" in p for p in problems)
