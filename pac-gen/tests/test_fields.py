from rm2svd.fields import fields_after, figure_w1c_bits, label_is_name, w1c_bits

TABLE = """**Table 5-3. TCR Field Descriptions**

|**Field**|**Description**|
|---|---|
|15–12|Reserved, must be cleared.|
|11–8<br>PRE|Prescaler.<br>0000 Divide by 1<br>0001 Divide by 2<br>0010 Divide by 4|
|7–3|Reserved|
|2<br>PIF_n_|Interrupt flag. Write 1 to clear.|
|1<br>RLD|Reload.<br>0 Free running<br>1 Reload from TMR|
|0<br>EN|Enable.|

**Table 5-4. TMR Field Descriptions**
"""


def test_field_ranges_and_reserved_rows():
    fs = {f.name: f for f in fields_after(TABLE, 16, "read-write")}
    assert sorted(fs) == ["EN", "PIFn", "PRE", "RLD"]
    assert (fs["PRE"].lsb, fs["PRE"].width) == (8, 4) and (fs["EN"].lsb, fs["EN"].width) == (0, 1)


def test_enumerations_become_values():
    fs = {f.name: f for f in fields_after(TABLE, 16, "read-write")}
    assert fs["RLD"].enum == {"FREE_RUNNING": 0, "RELOAD_FROM_TMR": 1}
    assert fs["PRE"].enum == {"DIVIDE_BY_1": 0, "DIVIDE_BY_2": 1, "DIVIDE_BY_4": 2}


def test_only_the_first_table_is_read():
    # the next register's table has rows of its own: none of them is read
    after = TABLE + "\n|**Field**|**Description**|\n|---|---|\n|15–0<br>MOD|Modulus.|\n"
    assert [f.name for f in fields_after(after, 16, "read-write")] == ["PRE", "PIFn", "RLD", "EN"]


def test_w1c_columns_from_the_bit_diagram():
    diag = "15 14 13 12 11 10 9 8 7 6 5 4 3 2 1 0 R 0 0 W w1c Reset 0 0"
    # a lone w1c under the W row with all other bits plain: position by count of preceding cells is unknowable,
    # so w1c bits come from the field descriptions instead (see fields_after)
    assert w1c_bits(diag, 16) == set()


def test_write_one_to_clear_from_the_description():
    fs = {f.name: f for f in fields_after(TABLE, 16, "read-write")}
    assert fs["PIFn"].one_to_clear and not fs["EN"].one_to_clear


TWO_TABLES = """**Table 9-1. AAA Field Descriptions**

|**Field**|**Description**|
|---|---|
|1<br>ONE|First.|

**Table 9-2. BBB Field Descriptions**

|**Field**|**Description**|
|---|---|
|0<br>TWO|Second table.|
"""

CONTINUED = """**Table 9-1. AAA Field Descriptions**

|**Field**|**Description**|
|---|---|
|3<br>AA|First page.|

Page footer text.

**Table 9-1. AAA Field Descriptions (continued)**

|**Field**|**Description**|
|---|---|
|2<br>BB|Second page.|

**Table 9-2. CCC Field Descriptions**

|**Field**|**Description**|
|---|---|
|0<br>CC|Next register.|
"""


def test_second_field_descriptions_table_is_not_read():
    assert [f.name for f in fields_after(TWO_TABLES, 8, "read-write")] == ["ONE"]


def test_continued_table_is_followed_across_the_page():
    assert [f.name for f in fields_after(CONTINUED, 8, "read-write")] == ["AA", "BB"]


def test_w1c_bits_positive_when_w_cells_match_width():
    diag = "3 2 1 0 R 0 0 0 0 W w1c 0 0 w1c Reset 0 0 0 0"
    assert w1c_bits(diag, 4) == {3, 0}


def test_w1c_bits_empty_when_cell_count_differs():
    assert w1c_bits("3 2 1 0 R 0 0 0 0 W w1c w1c Reset 0 0 0 0", 4) == set()


RANGES = """**Table 1-1. X Field Descriptions**

|**Field**|**Description**|
|---|---|
|15-12<br>HI|Above an 8-bit register.|
|8–4<br>STRAD|Straddles bit 8 of an 8-bit register.|
|7-4<br>HYPH|Hyphen range.|
|3|Reserved|
|2<br>Reserved|Named reserved.|
|1<br>RESERVED_X|Name starts with reserved.|
|0<br>LOW|Low bit.|
"""


def test_range_forms_and_skips():
    problems = []
    fs = {f.name: f for f in fields_after(RANGES, 8, "read-write", problems)}
    # hyphen range parsed; single-bit and named Reserved rows skipped; a name that only starts with Reserved is a
    # field; hi >= width rows are left out and reported
    assert sorted(fs) == ["HYPH", "LOW", "RESERVED_X"]
    assert (fs["HYPH"].lsb, fs["HYPH"].width) == (4, 4)
    assert problems == ["field beyond width: HI bits 15-12 in a 8-bit register",
                        "field beyond width: STRAD bits 8-4 in a 8-bit register"]


def test_hyphen_range_alone():
    fs = fields_after("**T. Field Descriptions**\n|11-8<br>PAV|x|\n", 16, "read-write")
    assert [(f.name, f.lsb, f.width) for f in fs] == [("PAV", 8, 4)]


def test_wider_register_keeps_rows_the_narrow_one_skips():
    problems = []
    names = [f.name for f in fields_after(RANGES, 16, "read-write", problems)]
    assert names == ["HI", "STRAD", "HYPH", "RESERVED_X", "LOW"] and problems == []


SPLIT = """**Table 15-5. PODR x_ Field Descriptions**

|**Field**|**Description**|
|---|---|
|15–8|Reserved, must be cleared.|
|7–1|Port output data bits.|
|PODRx_|0 Drives 0<br>1 Drives 1|
|0|Enable.|
|EN||
"""


def test_bit_range_and_name_on_consecutive_rows():
    # the converter often puts the bit range in one row and the field name in the next
    fs = {f.name: (f.lsb, f.width) for f in fields_after(SPLIT, 16, "read-write")}
    assert fs == {"PODRx": (1, 7), "EN": (0, 1)}


ENDS = """**Table 39-7. CNTR Field Descriptions**

|**Field**|**Description**|
|---|---|
|31–0<br>CNT|Counter.|

## **39.3 Functional Description**

**Table 39-8. Valid Mode Values**

|**Field**|**Description**|
|---|---|
|15–8<br>PS|Not this register's.|
"""


def test_a_heading_or_another_table_ends_the_field_table():
    assert [f.name for f in fields_after(ENDS, 32, "read-write")] == ["CNT"]
    other = ENDS.replace("## **39.3 Functional Description**\n", "")
    assert [f.name for f in fields_after(other, 32, "read-write")] == ["CNT"]


W1C_TABLE = """Address: 0xFC0B_0144 (XSTS)

Access: User read/write

||31|30|29|28|27|26|25|24|23|22|21|20|19|18|17|16|
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
|R|0|0|0|0|0|0|TI1|TI0|0|0|0|0|UPI|UAI|0|NAKI|
|W|||||||w1c|w1c|||||||||
|Reset|0|0|0|0|0|0|0|0|0|0|0|0|0|0|0|0|
||15|14|13|12|11|10|9|8|7|6|5|4|3|2|1|0|
|R|AS|PS|RCL|HCH|0|ULPII|0|SLI|SRI|URI|AAI|SEI|FRI|PCI|UEI|UI|
|W||||||||w1c|w1c|w1c|w1c|w1c|w1c|w1c|w1c|w1c|
|Reset|0|0|0|0|0|0|0|0|1|0|0|0|0|0|0|0|
"""


def test_w1c_bits_from_a_figure_table_follow_the_bit_number_rows():
    assert figure_w1c_bits(W1C_TABLE, 32) == {25, 24, *range(9)}


def test_w1c_bits_from_a_figure_table_need_every_bit_numbered():
    mangled = W1C_TABLE.replace("||15|14|13|", "||15|14 13|")   # a merged header cell: positions unknowable
    assert figure_w1c_bits(mangled, 32) == {25, 24}


def test_w1c_bits_from_picture_text_still_count_cells():
    assert figure_w1c_bits("3 2 1 0 R 0 0 0 0 W w1c 0 0 w1c Reset 0 0 0 0", 4) == {3, 0}


PROSE_ENUMS = """**Table 1-1. X Field Descriptions**

|**Field**|**Description**|
|---|---|
|2<br>PD|Power down.<br>0 Normal<br>1 Immediately power down converter A; the results of a scan using converter A are lost|
|1<br>MODE|Mode.<br>0 Keep running in the current mode when the flag is set<br>1 Stop|
|0<br>EN|Enable.<br>0 Module disabled<br>1 Module enabled|
"""


def test_enum_labels_that_are_sentences_drop_the_fields_enum():
    fs = {f.name: f for f in fields_after(PROSE_ENUMS, 8, "read-write")}
    assert fs["PD"].enum == {}                           # one label cut at the length cap
    assert fs["MODE"].enum == {}                         # one label longer than 24 characters and 4 words
    assert fs["EN"].enum == {"DISABLED": 0, "ENABLED": 1}


def test_label_rule():
    assert label_is_name("Divide by 1") and label_is_name("Reload from TMR")
    assert label_is_name("Reload from timer register")               # 26 characters but 4 words
    assert not label_is_name("Keep running in the current mode when the flag is set")
    assert not label_is_name("Short", truncated=True)


def test_label_words_are_counted_as_the_constant_spells_them():
    assert not label_is_name("On-chip 32 kHz oscillator")          # ON_CHIP_32_KHZ_OSCILLATOR: 5 words, 25 characters


def test_a_table_titled_field_description_is_read():
    fs = fields_after(TABLE.replace("Field Descriptions", "Field Description"), 16, "read-write")
    assert sorted(f.name for f in fs) == ["EN", "PIFn", "PRE", "RLD"]


ODD_ROWS = """**Table 21-9. CTL Field Descriptions**

|**Field**|**Description**|
|---|---|
|31–28<br>RDBAS<br>E|A name the converter split over two lines.|
|27–24<br>DLY_<br>CTRL|Split after an underscore.|
|23<br>BUSY/STAR<br>T|A slash in the name, split too.|
|22<br>16BIT|A name that starts with a digit.|
|21<br>2Hz|Lower case after the digit.|
|20<br>FIRST<br>SECOND|Two names that don't join.|
|19<br>FIFO[3:0]|A bit range after the name.|
|18–17, 16–15<br>EN_n_|One field per listed range, numbered high to low.|
|14<br>LPB<br>**Field**|A header cell's text after the name.|
|13<br>CFA1<br>12<br>CFA0|Two fields in one cell.|
|11 or 10<br>ALT|Alternative positions.|
|9<br>RDF<br>8|The next row's bit number after the name.|
|3–2<br>HRS<br>Hours counted in binary or in decimal|Text run into the name cell.|
|7, 6,|A list of bits continued on the next row.|
|5, 4<br>PU_n_|Its name.|
|0x00|0x10|A value table's row, not a field.|
"""


def test_odd_field_rows_are_read_or_reported():
    problems: list[str] = []
    fs = {f.name: (f.lsb, f.width) for f in fields_after(ODD_ROWS, 32, "read-write", problems)}
    assert fs == {"RDBASE": (28, 4), "DLY_CTRL": (24, 4), "BUSY_START": (23, 1), "_16BIT": (22, 1),
                  "_2HZ": (21, 1), "FIFO": (19, 1), "EN1": (17, 2), "EN0": (15, 2), "LPB": (14, 1),
                  "CFA1": (13, 1), "CFA0": (12, 1), "RDF": (9, 1),
                  "PU3": (7, 1), "PU2": (6, 1), "PU1": (5, 1), "PU0": (4, 1)}
    assert problems == ["field row unread: 20<br>FIRST<br>SECOND", "field row unread: 11 or 10<br>ALT",
                        "field row unread: 3–2<br>HRS<br>…"]       # the run-in text is not copied


W1C_OTHERS = """**Table 25-12. CTL Field Descriptions**

|**Field**|**Description**|
|---|---|
|24<br>RSTA|Reset all. Bits of type ROC, RW1C and RWAC are cleared.|
|7–4<br>CLDOK|Clear load okay.<br>0 No effect<br>1 Write a 1 to clear the corresponding LDOK.|
|3<br>FLG|Flag. Write 1 to clear.|
|2<br>ERR|Error flag (w1c).|
"""


def test_write_one_to_clear_means_this_field_not_another():
    fs = {f.name: f.one_to_clear for f in fields_after(W1C_OTHERS, 32, "read-write")}
    assert fs == {"RSTA": False, "CLDOK": False, "FLG": True, "ERR": True}


def test_em_dash_and_minus_sign_ranges():
    fs = fields_after("**T. Field Descriptions**\n|15—12<br>EMD|x|\n|11−8<br>MIN|x|\n|7 — 4<br>SPC|x|\n", 16,
                      "read-write")
    assert [(f.name, f.lsb, f.width) for f in fs] == [("EMD", 12, 4), ("MIN", 8, 4), ("SPC", 4, 4)]


def test_enum_codes_spell_every_bit_of_the_field():
    text = ("**T. Field Descriptions**\n"
            "|7<br>FAST|Clock. Runs at<br>10 MHz when set.|\n"                       # prose, not a value list
            "|6–5<br>MODE|Mode.<br>00 Off<br>01 Slow<br>10 Fast|\n"
            "|4–1<br>MASK|Per-bit mask.<br>0 Output normal<br>1 Output masked|\n"     # one bit's values, 4-bit field
            "|0<br>ONE|One value only.<br>1 Enabled|\n")
    fs = {f.name: f.enum for f in fields_after(text, 8, "read-write")}
    assert fs == {"FAST": {}, "MODE": {"OFF": 0, "SLOW": 1, "FAST": 2}, "MASK": {}, "ONE": {}}


def test_write_one_to_clear_wordings():
    text = ("**T. Field Descriptions**\n"
            "|5<br>A|Flag. Cleared by writing a 1 to it.|\n"
            "|4<br>B|Flag. Writing a one to this bit clears it.|\n"
            "|3<br>C|Flag. Write one to each bit to clear it.|\n"
            "|2<br>D|Flag. Writing 1 clears this bit.|\n"
            "|1<br>E|Writing a 1 clears the corresponding bit of F.|\n"
            "|0<br>F|Force. Write a 1 to this bit to force the endpoint.|\n"
            "|7<br>G|Flag. The user must write a 1<br>to clear this bit.|\n")      # a line break inside the words
    fs = {f.name: f.one_to_clear for f in fields_after(text, 8, "read-write")}
    assert fs == {"A": True, "B": True, "C": True, "D": True, "E": False, "F": False, "G": True}


from rm2svd import fields


def test_enum_names_drop_the_words_every_value_starts_with():
    desc = "Enables the thing.<br>0 PIF interrupt requests disabled<br>1 PIF interrupt requests enabled"
    assert fields._enum(desc, 1) == {"DISABLED": 0, "ENABLED": 1}


def test_enum_names_keep_a_word_and_never_start_with_a_digit():
    assert fields._enum("<br>0 Divide by 1<br>1 Divide by 2", 1) == {"DIVIDE_BY_1": 0, "DIVIDE_BY_2": 1}
    assert fields._enum("<br>00 Mode A<br>01 Mode B<br>10 Mode", 2) == {"MODE_A": 0, "MODE_B": 1, "MODE": 2}


def test_one_bit_field_is_bool_unless_both_values_are_names():
    desc = "<br>0 Disabled<br>1 Writing the modulus register replaces the counter value immediately and keeps counting"
    assert fields._enum(desc, 1) == {}
