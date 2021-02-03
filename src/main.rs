#![no_std]
#![no_main]
#![feature(asm)]

use core::sync::atomic::{AtomicUsize, Ordering};
use cortex_m_rt::entry;
use defmt::*;
use defmt_rtt as _;
use pac::{watchdog, xosc};
use panic_probe as _;
use rp2040_pac as pac;
use rp2040_pac::io_bank0::gpio0_ctrl::FUNCSEL_A;
use core::borrow::{BorrowMut, Borrow};
use rp2040_pac::pads_bank0::RegisterBlock;
use core::ops::Deref;

mod pll;
mod resets;

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER;

#[defmt::timestamp]
fn timestamp() -> u64 {
    static COUNT: AtomicUsize = AtomicUsize::new(0);
    // NOTE(no-CAS) `timestamps` runs with interrupts disabled
    let n = COUNT.load(Ordering::Relaxed);
    COUNT.store(n + 1, Ordering::Relaxed);
    n as u64
}

fn init(
    resets: pac::RESETS,
    watchdog: pac::WATCHDOG,
    clocks: pac::CLOCKS,
    xosc: pac::XOSC,
    pll_sys: pac::PLL_SYS,
    pll_usb: pac::PLL_USB,
) {
    // Now reset all the peripherals, except QSPI and XIP (we're using those
    // to execute from external flash!)

    let resets = resets::Resets::new(resets);

    // Reset everything except:
    // - QSPI (we're using it to run this code!)
    // - PLLs (it may be suicide if that's what's clocking us)
    resets.reset(!(resets::IO_QSPI | resets::PADS_QSPI | resets::PLL_SYS | resets::PLL_USB));

    resets.unreset_wait(
        resets::ALL
            & !(resets::ADC
                | resets::RTC
                | resets::SPI0
                | resets::SPI1
                | resets::UART0
                | resets::UART1
                | resets::USBCTRL),
    );

    // xosc 12 mhz
    watchdog
        .tick
        .write(|w| unsafe { w.cycles().bits(XOSC_MHZ as u16).enable().set_bit() });

    clocks.clk_sys_resus_ctrl.write(|w| unsafe { w.bits(0) });

    // Enable XOSC
    // TODO extract to HAL module
    const XOSC_MHZ: u32 = 12;
    xosc.ctrl.write(|w| w.freq_range()._1_15mhz());
    let startup_delay = (((XOSC_MHZ * 1_000_000) / 1000) + 128) / 256;
    xosc.startup
        .write(|w| unsafe { w.delay().bits(startup_delay as u16) });
    xosc.ctrl
        .write(|w| w.freq_range()._1_15mhz().enable().enable());
    while !xosc.status.read().stable().bit_is_set() {}

    // Before we touch PLLs, switch sys and ref cleanly away from their aux sources.
    clocks.clk_sys_ctrl.modify(|_, w| w.src().clk_ref());
    while clocks.clk_sys_selected.read().bits() != 1 {}
    clocks.clk_ref_ctrl.modify(|_, w| w.src().rosc_clksrc_ph());
    while clocks.clk_ref_selected.read().bits() != 1 {}

    resets.reset(resets::PLL_SYS | resets::PLL_USB);
    resets.unreset_wait(resets::PLL_SYS | resets::PLL_USB);

    pll::PLL::new(pll_sys).configure(1, 1500_000_000, 6, 2);
    pll::PLL::new(pll_usb).configure(1, 480_000_000, 5, 2);
}

#[entry]
fn main() -> ! {
    info!("Hello World!");

    let p = pac::Peripherals::take().unwrap();

    init(p.RESETS, p.WATCHDOG, p.CLOCKS, p.XOSC, p.PLL_SYS, p.PLL_USB);

    // i2c crap, TODO: generalize and extract to HAL
    // Pads should be configured for
    //  pull-up enabled
    //  slew rate limited
    //  schmitt trigger enabled
    p.PADS_BANK0.gpio16.write(|w| {
        w.schmitt().bit(true);
        w.pue().bit(true);
        w.slewfast().bit(false);
        w
    });
    p.PADS_BANK0.gpio17.write(|w| {
        w.schmitt().bit(true);
        w.pue().bit(true);
        w.slewfast().bit(false);
        w
    });
    // Select gpio i2c fn
    p.IO_BANK0.gpio17_ctrl.write(|w| {
        w.funcsel().i2c0_scl(); // Set this pin to SCL
        w
    });
    p.IO_BANK0.gpio16_ctrl.write(|w| {
        w.funcsel().i2c0_sda(); // Set this pin to SDA
        w
    });
    // Enable i2c
    p.I2C0.ic_enable.write(|w| {
        w.enable().bit(true); // Enable the i2c peripheral
        w
    });
    // Set i2c master, speed, and addressing mode
    let master = true;
    let ten_bit_addressing = false;
    p.I2C0.ic_con.write(|w| {
        // Bits 0 and 6 should always be set the same.
        w.master_mode().bit(master);      // Bit 0. Enable master mode.
        w.ic_slave_disable().bit(master); // Bit 6. Disable slave mode.
        w.ic_10bitaddr_master().bit(ten_bit_addressing);
        w.speed().standard();
        w
    });
    // 7-bit target addressing. Set target addr to 0x110_1000_1 <- one at end for read
    // CPU only needs to write here to initiate an i2c bus exchange.
    p.I2C0.ic_tar.write(|w| unsafe {
        w.bits(0x110_1000_1);
        w
    });
    // Read ACK
    let noack = p.I2C0.ic_tx_abrt_source.read().abrt_7b_addr_noack().bit();
    if !noack {
        info!("Acknoledge!");
        for x in 0..10 { // flash rapidly if we are connected
            cortex_m::asm::delay(100_000);
            p.IO_BANK0.gpio25_ctrl.write(|w| {
                w.oeover().enable();
                w.outover().high();
                w
            });
            cortex_m::asm::delay(100_000);
            p.IO_BANK0.gpio25_ctrl.write(|w| {
                w.oeover().enable();
                w.outover().low();
                w
            });
        }
    } else {
        info!("NoAck!");
        p.IO_BANK0.gpio25_ctrl.write(|w| {
            w.oeover().enable();
            w.outover().high();
            w
        });
        cortex_m::asm::delay(1_000_000);
        p.IO_BANK0.gpio25_ctrl.write(|w| {
            w.oeover().enable();
            w.outover().low();
            w
        });
    }

    cortex_m::asm::delay(5_000_000);
    loop {
        info!("on!");
        p.IO_BANK0.gpio25_ctrl.write(|w| {
            w.oeover().enable();
            w.outover().high();
            w
        });

        cortex_m::asm::delay(1_000_000);

        info!("off!");
        p.IO_BANK0.gpio25_ctrl.write(|w| {
            w.oeover().enable();
            w.outover().low();
            w
        });

        cortex_m::asm::delay(1_000_000);
    }
}
