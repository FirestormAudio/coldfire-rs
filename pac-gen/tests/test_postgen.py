"""postgen.py: part gates come from parts_map.json's absent lists, checked against Table 1-1's counts."""
import json
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "mcf5441x-pac"))
import postgen  # noqa: E402

PARTS = postgen.PARTS
LIB = """#![no_std]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Interrupt {
    #[doc = "64 - UART0"]
    UART0 = 64,
    #[doc = "66 - UART2"]
    UART2 = 66,
}
unsafe impl cortex_m::interrupt::InterruptNumber for Interrupt {
}
#[cfg(feature = "rt")]
mod _vectors {
}
#[cfg(feature = "rt")]
pub use cortex_m_rt::interrupt;
#[cfg(feature = "rt")]
pub use Interrupt as interrupt;
""" + "".join(f"pub const {n}: x::X = unsafe {{ x::X::from_ptr(0usize as _) }};\n"
              for n in ["UART0", "UART1", "UART2", "UART3", "TMR", *(f"PAD{i}" for i in range(40))])

LIB += """pub mod common {
    impl<T: Copy, A: Read> Reg<T, A> {
        pub fn read(&self) -> T {
            unsafe { (self.ptr as *mut T).read_volatile() }
        }
    }
    impl<T: Copy, A: Write> Reg<T, A> {
        pub fn write_value(&self, val: T) {
            unsafe { (self.ptr as *mut T).write_volatile(val) }
        }
    }
}
"""


def svd(counts: dict[str, int]) -> str:
    parts = "".join(f'<part name="{p}">' + "".join(f'<module group="{g}" instances="{n if p == "mcf54410" else 4}"/>'
                                                     for g, n in counts.items()) + "</part>" for p in PARTS)
    return f"<device><vendorExtensions>{parts}</vendorExtensions></device>"


SVD = """<device><peripherals>
<peripheral><name>UART0</name><interrupt><name>UART0</name><value>64</value></interrupt></peripheral>
<peripheral derivedFrom="UART0"><name>UART2</name><interrupt><name>UART2</name><value>66</value></interrupt></peripheral>
</peripherals></device>"""


def run(tmp_path, uart_count, absent, silent=None, svd_text=SVD):
    (tmp_path / "lib.rs").write_text(LIB)
    (tmp_path / "mcf5441x.svd").write_text(svd_text)
    (tmp_path / "raw.svd").write_text(svd({"UART": uart_count, "TMR": 1}))
    pm = {"groups": {"UART": {"match": "UARTs", "peripherals": ["UART"], "instances": True},
                     "TMR": {"match": "(TMR)", "peripherals": ["TMR"]}},
          "absent": {p: absent if p == "mcf54410" else [] for p in PARTS}, "silent": silent or {}}
    (tmp_path / "pm.json").write_text(json.dumps(pm))
    postgen.main(*(str(tmp_path / f) for f in ("lib.rs", "raw.svd", "pm.json", "out.rs", "mcf5441x.svd")))
    return (tmp_path / "out.rs").read_text()


def test_absent_instances_are_gated_and_the_rest_stay(tmp_path):
    out = run(tmp_path, 3, ["UART2"])
    others = ", ".join(f'feature = "{p}"' for p in PARTS[1:])
    assert f"#[cfg(any({others}))]\npub const UART2:" in out
    assert "\npub const UART3:" in out and "\npub const UART0:" in out        # present on every part: no gate


def test_a_count_the_absent_list_does_not_explain_is_an_error(tmp_path):
    with pytest.raises(SystemExit, match="UART"):
        run(tmp_path, 2, ["UART2"])                    # Table 1-1 says 2, the lists leave 3


def test_a_silent_mismatch_is_accepted_only_when_listed(tmp_path):
    out = run(tmp_path, 2, ["UART2"], {"mcf54410": {"UART": "the manual doesn't say which"}})
    assert "\npub const UART3:" in out


def test_an_absent_name_that_is_no_instance_is_an_error(tmp_path):
    with pytest.raises(SystemExit, match="UART7"):
        run(tmp_path, 3, ["UART7"])


def test_interrupts_of_absent_instances_are_gated_like_them(tmp_path):
    out = run(tmp_path, 3, ["UART2"])
    others = ", ".join(f'feature = "{p}"' for p in PARTS[1:])
    assert f'    #[cfg(any({others}))]\n    #[doc = "66 - UART2"]\n    UART2 = 66,' in out
    assert '{\n    #[doc = "64 - UART0"]\n    UART0 = 64,' in out


def test_an_interrupt_no_peripheral_raises_is_an_error(tmp_path):
    with pytest.raises(SystemExit, match="UART2"):
        run(tmp_path, 3, ["UART2"], svd_text=SVD.replace("<interrupt><name>UART2</name><value>66</value></interrupt>", ""))


TWO = SVD.replace("<peripherals>", "<peripherals><peripheral><name>UART3</name><interrupt><name>UART2</name>"
                                   "<value>66</value></interrupt></peripheral>")


def test_an_interrupt_two_peripherals_raise_is_present_where_either_is(tmp_path):
    # UART2 is absent on mcf54410 but UART3, which also raises the interrupt, is not
    out = run(tmp_path, 3, ["UART2"], svd_text=TWO)
    assert '    #[doc = "66 - UART2"]\n    UART2 = 66,' in out
    assert "#[cfg(any(feature" not in out.split("pub enum Interrupt")[1].split("}")[0]


def test_an_interrupt_is_absent_only_where_all_its_peripherals_are(tmp_path):
    out = run(tmp_path, 2, ["UART2", "UART3"], svd_text=TWO)
    others = ", ".join(f'feature = "{p}"' for p in PARTS[1:])
    assert f'    #[cfg(any({others}))]\n    #[doc = "66 - UART2"]\n    UART2 = 66,' in out


PAC_LIB = Path(__file__).resolve().parents[2] / "mcf5441x-pac" / "src" / "lib.rs"


def test_route_mmio_replaces_both_volatile_accesses():
    out = postgen.route_mmio(LIB)
    assert "read_volatile" not in out and "write_volatile" not in out
    for f in ("r8", "r16", "r32", "w8", "w16", "w32"):
        assert f"coldfire::mmio::{f}(" in out


def test_route_mmio_refuses_unexpected_input():
    with pytest.raises(AssertionError):
        postgen.route_mmio("pub mod common {}")


def test_generated_pac_registers_go_through_mmio():
    src = PAC_LIB.read_text()
    common = src[src.index("pub mod common {"):]
    common = common[:common.index("\npub mod ", 1)]
    assert "read_volatile" not in common and "write_volatile" not in common
    assert "coldfire::mmio::r32(" in common and "coldfire::mmio::w32(" in common


def test_rt_reexports_the_attribute_and_the_enum():
    src = PAC_LIB.read_text()
    assert "pub use coldfire_rt::interrupt;" in src and "pub use Interrupt as interrupt;" in src
