//! Decoded blocks: straight-line runs of instructions decoded once and executed many times.
//!
//! A block is keyed by the PC execution entered it at. It ends after a control-flow or SR-writing instruction, after
//! [`MAX_LEN`] instructions, or before the first instruction that starts in the next 4 KiB page, cannot be decoded,
//! or is not wholly in RAM (code is only decoded ahead in RAM: a device read has side effects). An instruction that
//! straddles a page boundary belongs to both pages.
//!
//! Every page holding a block's bytes is marked in [`Memory`]; any write into a marked page (a store, an exception
//! frame, [`Memory::load`]) reports the page and ends the current run of instructions, and the machine drops the
//! page's blocks before the next instruction. So a store is seen by the next instruction fetched, as when every
//! instruction is decoded where it is executed.

use std::collections::{BTreeMap, HashMap};

use crate::cpu::{Decoded, decode};
use crate::isa::{self, Op};
use crate::mem::{Memory, PAGE_SHIFT};

/// The most instructions in one block.
pub const MAX_LEN: usize = 64;

/// Entries in the direct-mapped table in front of the block map.
const FAST: usize = 4096;
/// A start address no block has (blocks start at even addresses).
const EMPTY: u32 = 1;

/// One decoded block.
#[derive(Debug)]
pub(crate) struct Block {
    pub(crate) start: u32,
    /// The address after its last instruction.
    pub(crate) end: u32,
    pub(crate) insns: Vec<Decoded>,
    /// Times execution entered it.
    pub(crate) count: u64,
}

impl Block {
    /// How many of its instructions come before the one at `pc`; all of them when none starts there.
    pub(crate) fn index_of(&self, pc: u32) -> usize {
        let mut at = self.start;
        for (i, d) in self.insns.iter().enumerate() {
            if at == pc {
                return i;
            }
            at = at.wrapping_add(d.len as u32);
        }
        self.insns.len()
    }
}

/// Whether a block ends after `op`: it may change PC, or it writes SR (the trace bit and the interrupt mask are
/// looked at between blocks).
fn ends_block(op: Op) -> bool {
    matches!(
        op,
        Op::Bra
            | Op::Bsr
            | Op::Bcc
            | Op::Jmp
            | Op::Jsr
            | Op::Rts
            | Op::Rte
            | Op::Trap
            | Op::Illegal
            | Op::Stop
            | Op::Halt
            | Op::MoveToSr
            | Op::Strldsr
    )
}

/// The block cache.
pub(crate) struct Blocks {
    /// Blocks by slot; a dropped block's slot is empty and on `free`.
    slots: Vec<Block>,
    free: Vec<u32>,
    /// Start address to slot.
    by_pc: HashMap<u32, u32>,
    /// (start, slot) at `(start >> 1) % FAST`, or [`EMPTY`].
    fast: Box<[(u32, u32)]>,
    /// The slots of the blocks with bytes in each page. Only looked up, never iterated.
    by_page: HashMap<u32, Vec<u32>>,
    /// Entry counts of dropped blocks, while profiling.
    dropped: BTreeMap<u32, u64>,
    profiling: bool,
}

impl Default for Blocks {
    fn default() -> Self {
        Blocks {
            slots: Vec::new(),
            free: Vec::new(),
            by_pc: HashMap::new(),
            fast: vec![(EMPTY, 0); FAST].into_boxed_slice(),
            by_page: HashMap::new(),
            dropped: BTreeMap::new(),
            profiling: false,
        }
    }
}

impl std::fmt::Debug for Blocks {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Blocks")
            .field("blocks", &self.by_pc.len())
            .field("profiling", &self.profiling)
            .finish_non_exhaustive()
    }
}

#[inline(always)]
fn fast_index(pc: u32) -> usize {
    (pc as usize >> 1) & (FAST - 1)
}

impl Blocks {
    /// The block starting at `pc`, decoding it if needed; `None` when no instruction at `pc` can go in a block (it
    /// is not in RAM or does not decode), so it runs on its own.
    #[inline(always)]
    pub(crate) fn get(&mut self, mem: &mut Memory, pc: u32) -> Option<u32> {
        let (start, slot) = self.fast[fast_index(pc)];
        if start == pc {
            return Some(slot);
        }
        self.lookup(mem, pc)
    }

    #[inline(never)]
    fn lookup(&mut self, mem: &mut Memory, pc: u32) -> Option<u32> {
        let slot = match self.by_pc.get(&pc) {
            Some(&slot) => slot,
            None => self.build(mem, pc)?,
        };
        self.fast[fast_index(pc)] = (pc, slot);
        Some(slot)
    }

    pub(crate) fn block_mut(&mut self, slot: u32) -> &mut Block {
        &mut self.slots[slot as usize]
    }

    /// Decode the block at `pc` and register it.
    fn build(&mut self, mem: &mut Memory, pc: u32) -> Option<u32> {
        let page = pc >> PAGE_SHIFT;
        let mut insns = Vec::new();
        let mut at = pc;
        while insns.len() < MAX_LEN && at >> PAGE_SHIFT == page {
            let opword = match mem.peek(at, 2) {
                Some(b) => u16::from_be_bytes([b[0], b[1]]),
                None => break,
            };
            let Some(form) = isa::decode(opword) else {
                break;
            };
            if !mem.in_ram(at, form.length(opword)) {
                break;
            }
            let Ok((d, _)) = decode(mem, at) else {
                break;
            };
            insns.push(d);
            at = at.wrapping_add(d.len as u32);
            if ends_block(d.op) {
                break;
            }
        }
        if insns.is_empty() {
            return None;
        }
        let last = at.wrapping_sub(1) >> PAGE_SHIFT;
        mem.mark_code(page, last);
        let block = Block {
            start: pc,
            end: at,
            insns,
            count: 0,
        };
        let slot = match self.free.pop() {
            Some(slot) => {
                self.slots[slot as usize] = block;
                slot
            }
            None => {
                self.slots.push(block);
                self.slots.len() as u32 - 1
            }
        };
        self.by_pc.insert(pc, slot);
        for p in page..=last {
            self.by_page.entry(p).or_default().push(slot);
        }
        Some(slot)
    }

    /// Drop every block with bytes in `page`.
    pub(crate) fn drop_page(&mut self, page: u32) {
        let Some(slots) = self.by_page.remove(&page) else {
            return;
        };
        for slot in slots {
            let b = std::mem::replace(
                &mut self.slots[slot as usize],
                Block {
                    start: EMPTY,
                    end: EMPTY,
                    insns: Vec::new(),
                    count: 0,
                },
            );
            self.by_pc.remove(&b.start);
            if self.fast[fast_index(b.start)].0 == b.start {
                self.fast[fast_index(b.start)] = (EMPTY, 0);
            }
            if self.profiling && b.count != 0 {
                *self.dropped.entry(b.start).or_default() += b.count;
            }
            // A block in two pages is listed under the other one too.
            for p in b.start >> PAGE_SHIFT..=b.end.wrapping_sub(1) >> PAGE_SHIFT {
                if p != page
                    && let Some(list) = self.by_page.get_mut(&p)
                {
                    list.retain(|&s| s != slot);
                }
            }
            self.free.push(slot);
        }
    }

    /// Count block entries from now on (`true`, starting from zero) or stop.
    pub(crate) fn profile(&mut self, on: bool) {
        self.profiling = on;
        self.reset_counts();
    }

    fn reset_counts(&mut self) {
        for b in &mut self.slots {
            b.count = 0;
        }
        self.dropped.clear();
    }

    /// Entries per block start address since profiling started or the profile was last taken, sorted by address;
    /// empty when not profiling.
    pub(crate) fn take_profile(&mut self) -> Vec<(u32, u64)> {
        if !self.profiling {
            return Vec::new();
        }
        let mut counts = std::mem::take(&mut self.dropped);
        for b in &self.slots {
            if b.count != 0 {
                *counts.entry(b.start).or_default() += b.count;
            }
        }
        self.reset_counts();
        counts.into_iter().collect()
    }
}
