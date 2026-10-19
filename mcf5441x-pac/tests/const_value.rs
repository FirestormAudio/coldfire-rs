//! Register values compose in const context (chiptool's setters are const fns).
use mcf5441x_pac::pit0::regs::Pcsr;
use mcf5441x_pac::pit0::vals::Pie;

const CFG: Pcsr = {
    let mut w = Pcsr(0);
    w.set_pre(2);
    w.set_pie(Pie::Enabled);
    w.set_rld(true);
    w
};

#[test]
fn a_register_value_composes_in_const() {
    assert_eq!(CFG.0, 0x0200 | 0x0008 | 0x0002);
}
