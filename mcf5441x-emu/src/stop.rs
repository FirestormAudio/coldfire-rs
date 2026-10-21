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
