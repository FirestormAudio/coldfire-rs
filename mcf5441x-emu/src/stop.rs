//! Why and where execution stopped, and what bounds a run.

/// Why execution stopped.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Stop {
    /// The program counter reached a requested address.
    UntilPc,
    /// An instruction was about to access the range [`Limit::until_access`] watches.
    UntilAccess,
    /// The instruction limit was reached.
    InstructionLimit,
    /// The time limit was reached.
    TimeLimit,
    /// The core halted with no interrupt that could wake it.
    HaltedForever,
    /// An illegal instruction with no usable handler.
    Illegal,
    /// An access to unmapped memory.
    Unmapped,
    /// A polling loop on a device register was detected.
    Poll,
    /// A device or instruction that is not modelled.
    Unmodelled,
    /// An exception with the given vector number stopped execution.
    Exception(u8),
    /// A device asked to halt, with its reason.
    Halt(&'static str),
}

/// The outcome of [`Machine::run`](crate::Machine::run).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct StopReason {
    pub kind: Stop,
    /// The program counter at the stop: the instruction that did not complete, or the next one to run.
    pub pc: u32,
    /// Human-readable context (the mnemonic of an unmodelled instruction, the faulting address, ...).
    pub detail: String,
}

/// Which accesses [`Limit::until_access`] stops at.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AccessKind {
    Read,
    Write,
    /// A read or a write.
    Any,
}

impl AccessKind {
    /// Whether an access that writes when `write` is of this kind.
    pub fn matches(self, write: bool) -> bool {
        match self {
            AccessKind::Read => !write,
            AccessKind::Write => write,
            AccessKind::Any => true,
        }
    }
}

/// Bounds for one [`Machine::run`](crate::Machine::run); the default has none.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Limit {
    /// Stop before executing the instruction at this address.
    pub until_pc: Option<u32>,
    /// Stop after this many instructions of this run.
    pub max_instructions: Option<u64>,
    /// An access breakpoint: `(base, len, kind)`. Stop with [`Stop::UntilAccess`] before the first access of
    /// `kind` by the core (an instruction's operand access, or an exception's stacking and vector fetch) that
    /// touches a byte of `base..base + len`, RAM or device. Nothing of it is performed: the instruction does not
    /// complete, PC stays on it and the registers are as before it, as for a refused device access (an access the
    /// same instruction made before it, such as the read of a read-modify-write when `kind` is
    /// [`AccessKind::Write`], or an earlier register of a MOVEM, stands). The detail names the access and its
    /// address. Instruction fetches and decoding, bus masters, the host's accesses and accesses through a mirror
    /// of the range's RAM ([`Memory::map_mirror`](crate::Memory::map_mirror)) are not watched. `len` 0 watches
    /// nothing; a range past the end of the address space stops there. The watch lasts for its run only: between
    /// runs every access sees the range as it is. `None` (the default) costs nothing.
    pub until_access: Option<(u32, u32, AccessKind)>,
}
