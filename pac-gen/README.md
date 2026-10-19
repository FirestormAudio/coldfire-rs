# pac-gen

Generates the MCF5441x peripheral access crate from the SVD.

## Tools

`pac-gen/tools.sh` installs svdtools and chiptool into `build/tools/` (git-ignored).
Nothing is installed system-wide. A tool already at its pinned version (svdtools' `--version`, the revision on
chiptool's line of `cargo install --list`) is left alone; any other version, or a missing binary, is reinstalled.
An install needs the network (crates.io for svdtools, GitHub for chiptool); a run with both tools pinned doesn't.

## Pins

- svdtools 0.5.0 (crates.io, `--version =0.5.0`)
- chiptool at git commit `bcf538a2e7b8584ae874ee9ab72efb1576fc6152`
  (`https://github.com/embassy-rs/chiptool`, `--rev` pinned in `tools.sh`); the crate there is chiptool v0.1.0

## Generate

`chiptool generate` builds the PAC directly from an SVD:

```
build/tools/bin/chiptool generate --svd <file.svd> [--transform <file>] --output <dir>
```

- `--svd <SVD>`: SVD file path (required)
- `--transform <TRANSFORM>`: transforms file path (a path to a YAML file, not inline YAML)
- `--output <OUTPUT>`: output directory of the PAC files
- `--namespaces none|block|block-with-regs-vals` (default `block-with-regs-vals`)
- `--no-defmt` / `--yes-defmt` / `--defmt-feature <FEATURE>`: defmt support
- `--skip-no-std`: don't emit `#![no_std]`

Other subcommands: `extract-all`, `extract-peripheral`, `transform`, `fmt`, `check`,
`gen-block`, `gen-common`.

## Patch and verify

`pac-gen/regen.sh` runs the pipeline:

```
RM=<markdown dir> CF_PYTHON=<python> pac-gen/regen.sh
```

1. only when `RM` is set: `python -m rm2svd $RM -o mcf5441x-pac/svd/raw.svd --coverage mcf5441x-pac/svd/coverage.txt`;
2. `svdtools patch --post-validate mcf5441x-pac/svd/patches/mcf5441x.yaml mcf5441x-pac/svd/mcf5441x.svd`
   (svdtools 0.5.0's usage is `svdtools patch <YAML> [OUT]`: without `OUT` it writes `<input>.patched`.
   The YAML's `_svd: ../raw.svd` is relative to the YAML file);
3. `python pac-gen/verify.py mcf5441x.svd pac-gen/oracles/*.json`: every oracle address must be a register
   (or a peripheral base) of the SVD, with the same width as one of the registers there when both know it, and
   every oracle bit constant must lie in that register's fields (below). It prints the exclusion counts, then
   `verify: ok` or one line per finding, and exits non-zero on any finding.

The oracles (`pac-gen/oracles/`, written by `extract.py <digitakt checkout>` and committed):

- `uboot-linux.json`: module bases and registers from digitakt's `re/peripherals/mcf54415.json`
  (U-Boot and Linux headers).
- `digitakt-mcf5441x.json`: the address constants of digitakt's `mcf5441x` and `mcf5441x-usb` crates. The width
  of each register is the accessor width (`r8`/`w8`, `r16`/`w16`, `r32`/`w32`) that digitakt's drivers use on it.
  `mcf5441x-usb` became `embassy_mcf5441x::usb` on 2026-10-18, so `extract.py` needs a digitakt checkout from before
  then (e.g. commit 821fd7b); it refuses a later one.

`digitakt-mcf5441x.json` also lists digitakt's bit constants under `bits` (PIT `PIF`, UART `USR_*`, eDMA `CSR_*`,
DSPI `SR_*`/`PUSHR_*` and the CTAR values, eSDHC `IRQSTAT`/`PRSSTAT`/`SYSCTL` bits, SSI `CR_*`, USB `cmd`/`sts`/
`portsc`/`epcr`, …). `verify.py` checks each against the SVD's fields at that address: against the register
digitakt names where one of the registers there is it (`UART0+UIMR`, not the UISR sharing its address), else any
alternate there: every set bit lies in a field; a constant named like a field (`SR_EOQF`) is exactly that field;
names its doc comment lists (`OE | PE | FE`) are fields holding some of its bits; `NAME = n` in its doc comment
(`FMSZ = 15` for `CTAR1_FLASH16`) is the field's value in it; a `_SHIFT` is the field's first bit. Field names
are compared without underscores or case, but with their index: `PCS1` is not `PCS2`, and an indexed field
(`PCSn`) holds any index.

`extract.py` places each bit constant in its register by hand tables (`BIT_REGISTERS`, `ESDHC_BITS`); a constant
under a prefix they don't know is kept, excluded with the note "no register known for this constant", and
counted apart in verify's summary line, so a new digitakt constant left unchecked shows there until the table
learns it.

An entry judged wrong keeps its place, with `"excluded": true` and a `note` giving the reason (`EXCLUDE` in
`extract.py`); `verify.py` skips it. An entry whose width alone is wrong (`EXCLUDE_WIDTH`) is
`"excluded": "width"`: its address must still be a register.

## Peripheral names

The patches rename peripherals where rm2svd had only a chapter title's first word, or a label:

| raw.svd | mcf5441x.svd | why |
|---|---|---|
| DMATIMER0-3 | DTIM0-3 | RM ch. 39 acronym; the interrupt labels say DTIM |
| ROBUST | RTC | RM ch. 37 register prefix and interrupt label |
| ETHERNET | ESW | RM ch. 32 register prefix |
| CLOCK | PLL | RM ch. 8 register prefix and interrupt label |
| PIN_MULTIPLEXING | GPIO | ports and pin assignment; U-Boot and Linux name |
| INTC (0xFC05_4000) | IACK | it holds only the global IACK registers |

Some register blocks are shared by several chapters: reset, CCM, SBF and LPCR at 0xEC09_0000, and power
management and SCM at 0xFC04_0000. Their offsets don't overlap, so rm2svd merges each block into one
peripheral: `CCM` (the largest chapter) and `SCM` (`SHARED_BLOCK_NAMES`: the SCM chapter owns that block). This
matches the U-Boot and Linux structures. It also gives chiptool one block per address.

Repeated instances are `derivedFrom` the first one: DTIM, PIT, DAC, DSPI, I2C, UART, FLEXCAN, MACNET and SSI.
Register names drop their module prefix (`DSPI_MCR` → `MCR`, `EDMA_SERQ` → `SERQ`, `PWM_SM0CR` → `SM0CR`), so
`DSPI0.mcr()` reads like `PIT0.pcsr()`. The prefix is the peripheral's group name, or the chapter's register prefix
where they differ (`MODULE_PREFIXES` in `rm2svd/build.py`: DDR, ENET, PWM, ESW, PLL, RTC). A register whose
stripped name another register already has keeps its prefix (`prefix kept:` in `coverage.txt`). Pin-multiplexing
names (`PAR_UART0`, `PODR_A`) keep theirs: they name a port, not the module.

## Part availability

rm2svd reads RM Table 1-1 into `raw.svd`'s `<vendorExtensions>` (svdtools drops them, so `postgen.py` reads
`raw.svd`). Each row is named by a short key from `parts_map.json` (`groups`): the key whose `match` text the row's
label contains, so the manual's row labels are not stored. A row matching no key, or several, is a problem line in
`coverage.txt`. `parts_map.json` also holds the per-part `absent` lists the PAC's `#[cfg]` gates come from, and the
`silent` cases where Table 1-1 counts fewer instances than the lists leave and the manual doesn't say which.

## coverage.txt

One line per peripheral, then every problem rm2svd met, by kind: `no figure`, `no fields` (a figure, but no field
table read), `field row unread` (a field table row starting with a bit number that couldn't be read; run-in
description text is shown as `…`), `field beyond width` (a row above the register's width, left out), `field table
names another register` (kept only when the figure's own diagram shows the table's fields), `figure name differs`,
`no map row` (a figure without a memory-map row; `left out` when it lies outside the chapter's blocks), `no width`,
`empty peripheral`, `prefix kept`, `availability` (Table 1-1 rows by position: a row no `parts_map.json` group
matches, or cells that aren't a count, such as `2 ports` or a package name, taken as present), and the map-row
kinds (`unread map row` with the row's address, including rows with an address and nothing else; `access not
understood`; strides and ranges), and `access` (a field whose figure gives it more access than the register's
map row: the figure's R and W rows decide. A W row with cells makes the register writable, taking the figure's
`Access:` when it says write-only; a blank one makes the field read-only; likewise R cells and a write-only
register. A read and a write register sharing an address and a figure (UISR/UIMR) keep their own direction).

A register's figure is the one naming it; the figure's name may carry the instance's number (`DAC0_CR` for DAC_CR
in DAC0, `UMR10` for UMR1 in UART0), but no other digit may differ (UMR2's figure isn't UMR1's). Enumerated values
come from `code label` lines whose code spells every bit of the field, two or more of them.
