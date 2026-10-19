// Spot checks against registers a board drives in hardware-proven code.
#[test]
fn hardware_proven_registers_are_where_the_unit_has_them() {
    assert_eq!(mcf5441x_pac::PIT0.pcsr().as_ptr() as usize, 0xFC08_0000);
    assert_eq!(mcf5441x_pac::PIT0.pmr().as_ptr() as usize, 0xFC08_0002);
    assert_eq!(mcf5441x_pac::UART9.as_ptr() as usize, 0xEC07_4000);
    // register names carry no module prefix: DSPI0.mcr(), not dspi_mcr()
    assert_eq!(mcf5441x_pac::DSPI0.mcr().as_ptr() as usize, 0xFC05_C000);
    assert_eq!(mcf5441x_pac::DSPI0.popr().as_ptr() as usize, 0xFC05_C038);
    assert_eq!(mcf5441x_pac::EDMA.serq().as_ptr() as usize, 0xFC04_4018);
}

// DSPI0 CTAR: FMSZ (bits 30-27) is a field.
#[test]
fn ctar_frame_size_is_a_field() {
    let ctar1 = mcf5441x_pac::dspi0::regs::Ctar0(0x7802_0010);
    assert_eq!(ctar1.fmsz(), 15);
    assert_eq!(mcf5441x_pac::dspi0::regs::Ctar0(0x3802_0010).fmsz(), 7);
}
