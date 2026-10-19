//! Getting the app part into DDR. [`plan`] works out where its bytes are (pure, host-tested); `fetch` copies them in
//! the boot part, before anything runs from DDR.
use super::config::AppSource;

/// Where the app part's first byte is, once the boot part knows the app part's offset in the image.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum FetchPlan {
    /// Memory-mapped: copy from `src`.
    Copy { src: u32 },
    /// SPI flash: read from byte `offset` with DSPI0's CTAR0 = `ctar`.
    Spi { offset: u32, ctar: u32 },
    /// NAND, boot-page format: from page `first_page`, skipping `skip` payload bytes of it.
    Nand { first_page: u32, skip: u32 },
}

/// NAND pages the NFC reads at boot (RM 22.4.4); the app pages follow them.
pub const BOOT_PAGES: u32 = 4;
/// Payload bytes of a boot-format page (1056 bytes, 60 of them ECC).
pub const PAYLOAD: u32 = 996;

/// `app_offset`: the app part's offset from image offset 0 (serial, NOR) or in the NAND payload stream.
pub const fn plan(src: &AppSource, app_offset: u32) -> FetchPlan {
    match *src {
        AppSource::Mapped(base) => FetchPlan::Copy { src: base + app_offset },
        AppSource::SpiFlash { offset, ctar } => FetchPlan::Spi { offset: offset + app_offset, ctar },
        AppSource::Nand => FetchPlan::Nand { first_page: BOOT_PAGES + app_offset / PAYLOAD, skip: app_offset % PAYLOAD },
    }
}

/// Copy `len` bytes (a multiple of 4) of the app part to `dst` as `p` says. Runs before DDR code exists: word loop,
/// no library calls.
#[cfg(target_arch = "m68k")]
#[unsafe(link_section = ".boot.text.coldfire_rt_fetch")]
#[inline(never)]
pub(crate) unsafe fn fetch(p: FetchPlan, dst: u32, len: u32) {
    match p {
        FetchPlan::Copy { src } => {
            let mut i = 0;
            while i < len {
                unsafe { ((dst + i) as *mut u32).write_volatile(((src + i) as *const u32).read_volatile()) };
                i += 4;
            }
        }
        FetchPlan::Spi { offset, ctar } => unsafe { super::spi::read(offset, ctar, dst, len) },
        FetchPlan::Nand { first_page, skip } => unsafe { super::nand::read(first_page, skip, dst, len) },
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mapped_copies_from_base_plus_the_app_offset() {
        assert_eq!(plan(&AppSource::Mapped(0x8000_0000), 0x2000), FetchPlan::Copy { src: 0x8000_2000 });
    }

    #[test]
    fn spi_reads_from_the_image_offset_plus_the_app_offset() {
        let p = plan(&AppSource::SpiFlash { offset: 0x1_0007, ctar: 0x3800_0001 }, 0x2000);
        assert_eq!(p, FetchPlan::Spi { offset: 0x1_2007, ctar: 0x3800_0001 });
    }

    #[test]
    fn nand_starts_on_the_page_after_the_four_boot_pages() {
        assert_eq!(plan(&AppSource::Nand, 0), FetchPlan::Nand { first_page: 4, skip: 0 });
        assert_eq!(plan(&AppSource::Nand, 996 + 8), FetchPlan::Nand { first_page: 5, skip: 8 });
    }
}
