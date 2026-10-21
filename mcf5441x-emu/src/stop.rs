//! Why and where execution stopped, and what bounds a run.

/// Why execution stopped.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Stop {
    /// The program counter reached a requested address.
    UntilPc,
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

/// Bounds for one [`Machine::run`](crate::Machine::run); the default has none.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Limit {
    /// Stop before executing the instruction at this address.
    pub until_pc: Option<u32>,
    /// Stop after this many instructions of this run.
    pub max_instructions: Option<u64>,
}
