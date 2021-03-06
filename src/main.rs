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

mod pll;
mod resets;
mod uart;

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

static GREETING: &str = "\n\r/ Hello fellow rustaceans! Now I talk to \\\r
\\ you from a Raspberry Pico board!       /\r
 -----------------------------------------\r
        \\\r
         \\\r
            _~^~^~_\r
        \\) /  o o  \\ (/\r
          '_   -   _'\r
          / '-----' \\\n\n\n\n\r";

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
                | resets::USBCTRL
                | resets::IO_BANK0),
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

    // Activate peripheral clock
    clocks.clk_peri_ctrl.write(|w| w.enable().set_bit());
}

fn uart_set_gpios(p: &pac::IO_BANK0) {
    // todo funcsel in pac not implemented for generic access to gpio
    // for UART, the funcsel is 2 for all pins, so calling uart0_tx
    // on each gp does the trick but is very confusing

    // set GP0 to UART0_TX
    p.gpio[0].gpio_ctrl.write(|w| w.funcsel().uart0_tx());
    // set GP1 to UART0_RX
    p.gpio[1].gpio_ctrl.write(|w| w.funcsel().uart0_tx());
    // set GP4 to UART1_TX
    p.gpio[4].gpio_ctrl.write(|w| w.funcsel().uart0_tx());
    // set GP5 to UART1_RX
    p.gpio[5].gpio_ctrl.write(|w| w.funcsel().uart0_tx());
}

#[entry]
fn main() -> ! {
    info!("Hello World!");

    let p = pac::Peripherals::take().unwrap();

    init(p.RESETS, p.WATCHDOG, p.CLOCKS, p.XOSC, p.PLL_SYS, p.PLL_USB);

    uart_set_gpios(&p.IO_BANK0);
    // Peripheral clock is attached to sys clk
    const PERI_CLK: u32 = 5_000_000;

    let uart0 = uart::UART::new(p.UART0, PERI_CLK);
    uart0.configure(115200);
    let uart1 = uart::UART::new(p.UART1, PERI_CLK);
    uart1.configure(115200);

    loop {
        info!("on!");
        p.IO_BANK0.gpio[25].gpio_ctrl.write(|w| {
            w.oeover().enable();
            w.outover().high();
            w
        });

        cortex_m::asm::delay(1_000_000);

        info!("off!");
        p.IO_BANK0.gpio[25].gpio_ctrl.write(|w| {
            w.oeover().enable();
            w.outover().low();
            w
        });

        cortex_m::asm::delay(1_000_000);

        uart0.write_blocking(&GREETING.as_bytes());
        uart1.write_blocking(&GREETING.as_bytes());
    }
}
