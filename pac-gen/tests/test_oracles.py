"""extract.py's bit oracle: digitakt's register bit constants, each with the register it belongs to."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "oracles"))
import extract  # noqa: E402

LIB = """pub mod pit {
    pub const PIT0: usize = 0xFC08_0000;
    pub const PCSR: usize = 0x0;
    pub const PIF: u16 = 1 << 2;
    pub const fn pre(p: u16) -> u16 { (p & 0xF) << 8 }
}
pub mod dspi {
    pub const DSPI0: usize = 0xFC05_C000;
    pub const CTAR0: usize = 0x0C;
    pub const CTAR1: usize = 0x10;
    pub const SR: usize = 0x2C;
    /// 8-bit frames.
    pub const CTAR0_FLASH: u32 = 0xBE01_0020;
    /// CTAR0_FLASH with 16-bit frames (FMSZ = 15).
    pub const CTAR1_FLASH16: u32 = 0xFE01_0020;
    pub const SR_EOQF: u32 = 1 << 28;
}
pub mod uart {
    pub const UART9: usize = 0xEC07_4000;
    pub const USR_UCSR: usize = 0x04;
    /// OE | PE | FE.
    pub const USR_ERRORS: u8 = 0x70;
    pub const UART9_SOURCE: usize = 53;
}
pub mod clocks {
    pub const SLOT_PIT0: u8 = 32;
}
"""
USB = """pub const BASE: usize = 0xFC0B_0000;
pub const USBCMD: usize = BASE + 0x140;
pub const PORTSC1: usize = BASE + 0x184;
pub mod cmd {
    pub const RS: u32 = 1;
}
pub mod portsc {
    /// PSPD 27:26.
    pub const PSPD_SHIFT: u32 = 26;
    pub const PSPD_HIGH: u32 = 0b10;
}
"""


def test_bit_constants_become_field_checks(tmp_path):
    (tmp_path / "rust/mcf5441x/src").mkdir(parents=True)
    (tmp_path / "rust/mcf5441x-usb/src").mkdir(parents=True)
    (tmp_path / "rust/mcf5441x/src/lib.rs").write_text(LIB)
    (tmp_path / "rust/mcf5441x-usb/src/regs.rs").write_text(USB)
    bits = {b["name"]: b for b in extract.bits(tmp_path)}
    assert bits["pit::PIF"] == {"name": "pit::PIF", "register": "pit::PIT0+PCSR", "address": 0xFC080000,
                                "value": 4, "fields": [{"name": "PIF", "mask": 4}]}
    assert bits["pit::pre"]["fields"] == [{"name": "PRE", "mask": 0xF00}]
    assert bits["dspi::CTAR1_FLASH16"]["address"] == 0xFC05C010
    assert bits["dspi::CTAR1_FLASH16"]["fields"] == [{"name": "FMSZ", "value": 15}]
    assert bits["dspi::CTAR0_FLASH"]["fields"] == []                         # bits covered, names unknown
    assert bits["dspi::SR_EOQF"]["fields"] == [{"name": "EOQF", "mask": 1 << 28}]
    assert bits["uart::USR_ERRORS"]["fields"] == [{"name": n, "within": True} for n in ("OE", "PE", "FE")]
    assert bits["usb::cmd::RS"]["address"] == 0xFC0B0140
    assert bits["usb::portsc::PSPD_SHIFT"]["fields"] == [{"name": "PSPD", "lsb": 26}]
    # constants that aren't register bits are kept, marked, with the reason
    assert bits["usb::portsc::PSPD_HIGH"]["excluded"] and bits["usb::portsc::PSPD_HIGH"]["note"]
    assert bits["clocks::SLOT_PIT0"]["excluded"]
    assert "uart::UART9_SOURCE" not in bits                                    # usize: an address, not bits


def test_doc_values_need_a_number_and_indexed_names_are_one_bit_of_a_field(tmp_path):
    (tmp_path / "rust/mcf5441x/src").mkdir(parents=True)
    (tmp_path / "rust/mcf5441x-usb/src").mkdir(parents=True)
    (tmp_path / "rust/mcf5441x/src/lib.rs").write_text(LIB.replace(
        "    pub const SR_EOQF: u32 = 1 << 28;\n",
        "    pub const SR_EOQF: u32 = 1 << 28;\n    pub const PUSHR: usize = 0x34;\n"
        "    pub const PUSHR_PCS1: u32 = 1 << 17;\n    /// PUSHR DTW = 4-bit.\n    pub const PUSHR_DTW_4BIT: u32 = 1 << 1;\n"))
    (tmp_path / "rust/mcf5441x-usb/src/regs.rs").write_text(USB)
    bits = {b["name"]: b for b in extract.bits(tmp_path)}
    assert bits["dspi::PUSHR_PCS1"]["fields"] == [{"name": "PCS1", "within": True}]
    assert {"name": "DTW", "value": 4} not in bits["dspi::PUSHR_DTW_4BIT"]["fields"]


def test_a_checkout_without_the_usb_crate_is_refused_with_the_reason(tmp_path):
    # digitakt's rust/mcf5441x-usb moved into embassy_mcf5441x::usb: the oracle's USB constants come from a
    # checkout made before that, and a later one must say so rather than fail on a missing file
    (tmp_path / "rust/mcf5441x/src").mkdir(parents=True)
    (tmp_path / "rust/mcf5441x/src/lib.rs").write_text(LIB)
    import pytest
    with pytest.raises(SystemExit, match="before the USB driver moved"):
        extract.check_source(tmp_path)
