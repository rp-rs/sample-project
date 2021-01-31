use rp2040_pac as pac;

pub use pac::resets::fields::Peripherals;

pub const ALL_PERIPHERALS: Peripherals = Peripherals::from_bits(0x01ffffff);

pub struct Resets {}

impl Resets {
    pub fn new() -> Self {
        Self {}
    }

    pub fn reset(&self, peris: Peripherals) {
        unsafe {
            pac::RESETS.reset().write_value(peris);
        }
    }

    pub fn unreset_wait(&self, peris: Peripherals) {
        unsafe {
            // TODO use the "atomic clear" register version
            pac::RESETS
                .reset()
                .modify(|v| *v = Peripherals::from_bits(v.to_bits() & !peris.to_bits()));
            while ((!pac::RESETS.reset_done().read().to_bits()) & peris.to_bits()) != 0 {}
        }
    }
}
