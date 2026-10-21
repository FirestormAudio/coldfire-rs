//! Dispatch: one handler per [`Op`], in a table indexed by the `Op`'s discriminant.

use crate::cpu::{Cpu, Decoded, ops};
use crate::isa::Op;
use crate::mem::{Fault, Memory};
use crate::stop::Stop;

/// Executes one decoded instruction. A `Fault` ends the instruction (and the run) where it happened.
pub type Handler = fn(&mut Cpu, &mut Memory, &Decoded) -> Result<Flow, Fault>;

/// What happens after an instruction.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Flow {
    /// Continue with the next instruction: the caller advances PC by the instruction's length.
    Next,
    /// The handler has set PC.
    Jump,
    /// Stop without completing the instruction; PC stays on it.
    Stop(Stop),
}

/// The handler for an instruction this core does not model (yet): it stops the run, and the caller reports the
/// form's mnemonic.
pub fn unmodelled(_: &mut Cpu, _: &mut Memory, _: &Decoded) -> Result<Flow, Fault> {
    Ok(Flow::Stop(Stop::Unmodelled))
}

/// The handler for `op`.
pub const fn handler(op: Op) -> Handler {
    use ops::{arith, moves};
    match op {
        Op::Move => moves::move_,
        Op::Movea => moves::movea,
        Op::Moveq => moves::moveq,
        Op::Mvs => moves::mvs,
        Op::Mvz => moves::mvz,
        Op::Illegal => moves::illegal,
        Op::Addq => arith::addq,
        Op::Subq => arith::subq,
        _ => unmodelled,
    }
}

const COUNT: usize = Op::ALL.len();

/// Every handler, indexed by `op as usize`.
pub static HANDLERS: [Handler; COUNT] = {
    let mut table = [unmodelled as Handler; COUNT];
    let mut i = 0;
    while i < COUNT {
        table[Op::ALL[i] as usize] = handler(Op::ALL[i]);
        i += 1;
    }
    table
};

/// Run one decoded instruction.
#[inline(always)]
pub fn execute(cpu: &mut Cpu, mem: &mut Memory, d: &Decoded) -> Result<Flow, Fault> {
    HANDLERS[d.op as usize](cpu, mem, d)
}
