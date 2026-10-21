# mcf5441x-emu

An exact emulator of the MCF5441x (ColdFire V4e) for running firmware on the host.

Status: core only. The crate currently provides the memory map (big-endian RAM regions and faulting
accesses); the CPU, peripherals and block cache come later.
