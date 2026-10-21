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
    /// Stop without completing the instruction; PC stays on it and the registers are as they were.
    Stop(Stop),
    /// The instruction takes exception `vector` instead of completing: the registers are restored to their state
    /// before it (RM 3.3.4.16), and the machine stops or vectors according to its
    /// [`ExceptionPolicy`](crate::cpu::ExceptionPolicy).
    Exception(u8),
    /// As [`Flow::Exception`], but when the exception is taken (not stopped on) the instruction's register updates
    /// stand: JSR and BSR push their return address before the target takes an address error (RM 3.3.4.2).
    ExceptionAfterPush(u8),
    /// STOP: the instruction is complete (PC past it) and the core waits for an interrupt.
    Wait,
    /// The instruction is complete and the run stops after it (HALT).
    StopAfter(Stop),
}

/// The handler for an instruction this core does not model (yet): it stops the run, and the caller reports the
/// form's mnemonic.
pub fn unmodelled(_: &mut Cpu, _: &mut Memory, _: &Decoded) -> Result<Flow, Fault> {
    Ok(Flow::Stop(Stop::Unmodelled))
}

/// The handler for `op`.
pub const fn handler(op: Op) -> Handler {
    use ops::{arith, bits, emac, flow, isac, logic, moves, muldiv, shift, system};
    match op {
        Op::Move => moves::move_,
        Op::Movea => moves::movea,
        Op::Moveq => moves::moveq,
        Op::Mvs => moves::mvs,
        Op::Mvz => moves::mvz,
        Op::Addq => arith::addq,
        Op::Subq => arith::subq,
        Op::AddEaDn => arith::add_ea_dn,
        Op::AddDnEa => arith::add_dn_ea,
        Op::SubEaDn => arith::sub_ea_dn,
        Op::SubDnEa => arith::sub_dn_ea,
        Op::Addi => arith::addi,
        Op::Subi => arith::subi,
        Op::Adda => arith::adda,
        Op::Suba => arith::suba,
        Op::Addx => arith::addx,
        Op::Subx => arith::subx,
        Op::Cmp => arith::cmp,
        Op::Cmpi => arith::cmpi,
        Op::Cmpa => arith::cmpa,
        Op::Neg => arith::neg,
        Op::Negx => arith::negx,
        Op::AndEaDn => logic::and_ea_dn,
        Op::AndDnEa => logic::and_dn_ea,
        Op::OrEaDn => logic::or_ea_dn,
        Op::OrDnEa => logic::or_dn_ea,
        Op::Eor => logic::eor,
        Op::Andi => logic::andi,
        Op::Ori => logic::ori,
        Op::Eori => logic::eori,
        Op::Not => logic::not,
        Op::Clr => logic::clr,
        Op::Tst => logic::tst,
        Op::Tas => logic::tas,
        Op::Ext => logic::ext,
        Op::Extb => logic::extb,
        Op::Swap => logic::swap,
        Op::AsrImm => shift::asr_imm,
        Op::AsrReg => shift::asr_reg,
        Op::AslImm => shift::asl_imm,
        Op::AslReg => shift::asl_reg,
        Op::LsrImm => shift::lsr_imm,
        Op::LsrReg => shift::lsr_reg,
        Op::LslImm => shift::lsl_imm,
        Op::LslReg => shift::lsl_reg,
        Op::BtstImmEa => bits::btst_imm,
        Op::BtstDnEa => bits::btst_dn,
        Op::BsetImmEa => bits::bset_imm,
        Op::BsetDnEa => bits::bset_dn,
        Op::BclrImmEa => bits::bclr_imm,
        Op::BclrDnEa => bits::bclr_dn,
        Op::BchgImmEa => bits::bchg_imm,
        Op::BchgDnEa => bits::bchg_dn,
        Op::Scc => bits::scc,
        Op::MulsW => muldiv::muls_w,
        Op::MuluW => muldiv::mulu_w,
        Op::MulL => muldiv::mul_l,
        Op::DivsW => muldiv::divs_w,
        Op::DivuW => muldiv::divu_w,
        Op::DivL => muldiv::div_l,
        Op::Mov3q => isac::mov3q,
        Op::Sats => isac::sats,
        Op::Ff1 => isac::ff1,
        Op::Bitrev => isac::bitrev,
        Op::Byterev => isac::byterev,
        Op::Tpf => isac::tpf,
        Op::Nop => flow::nop,
        Op::Bra => flow::bra,
        Op::Bsr => flow::bsr,
        Op::Bcc => flow::bcc,
        Op::Jmp => flow::jmp,
        Op::Jsr => flow::jsr,
        Op::Rts => flow::rts,
        Op::Lea => flow::lea,
        Op::Pea => flow::pea,
        Op::Link => flow::link,
        Op::Unlk => flow::unlk,
        Op::MovemRegsEa => flow::movem_regs_ea,
        Op::MovemEaRegs => flow::movem_ea_regs,
        Op::MoveFromSr => system::move_from_sr,
        Op::MoveToSr => system::move_to_sr,
        Op::MoveFromCcr => system::move_from_ccr,
        Op::MoveToCcr => system::move_to_ccr,
        Op::MoveToUsp => system::move_to_usp,
        Op::MoveFromUsp => system::move_from_usp,
        Op::Movec => system::movec,
        Op::Strldsr => system::strldsr,
        Op::Trap => system::trap,
        Op::Illegal => system::illegal,
        Op::Rte => system::rte,
        Op::Stop => system::stop,
        Op::Halt => system::halt,
        Op::Pulse => system::pulse,
        Op::Wddata => system::wddata,
        Op::Wdebug => system::wdebug,
        Op::Cpushl | Op::Intouch => system::cache_hint,
        Op::Mac => emac::mac,
        Op::MacLoad => emac::mac_load,
        Op::MoveToAcc => emac::move_to_acc,
        Op::MoveAccAcc => emac::move_acc_acc,
        Op::MoveFromAcc => emac::move_from_acc,
        Op::Movclr => emac::movclr,
        Op::MoveToMacsr => emac::move_to_macsr,
        Op::MoveFromMacsr => emac::move_from_macsr,
        Op::MoveMacsrCcr => emac::move_macsr_ccr,
        Op::MoveToAccext01 => emac::move_to_accext01,
        Op::MoveFromAccext01 => emac::move_from_accext01,
        Op::MoveToAccext23 => emac::move_to_accext23,
        Op::MoveFromAccext23 => emac::move_from_accext23,
        Op::MoveToMask => emac::move_to_mask,
        Op::MoveFromMask => emac::move_from_mask,
        Op::FpuOp | Op::Fbcc | Op::Fsave | Op::Frestore => system::line_f,
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
