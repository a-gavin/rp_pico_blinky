//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

use cortex_m::delay::Delay;
use cortex_m_rt::entry;
use embedded_hal::digital::v2::ToggleableOutputPin;

use rp_pico as bsp;
use bsp::hal::{
    clocks::{init_clocks_and_plls, ClockSource},
    gpio::pin::bank0::{Gpio16, Gpio18, Gpio25},
    gpio::pin::{Pin, PullUpInput, PushPullOutput},
    gpio::Interrupt::EdgeLow,
    pac,
    pac::interrupt,
    sio::Sio,
    watchdog::Watchdog,
};

use defmt::*;
use defmt_rtt as _;
use panic_probe as _;

use core::cell::RefCell;
use critical_section::Mutex;

// Types for method to transfer ownership of LED and button pins to interrupt handler
type LedPin = Pin<Gpio25, PushPullOutput>;
type ButtonPin = Pin<Gpio16, PullUpInput>;
type PulsePin = Pin<Gpio18, PushPullOutput>;
type LedPulseAndButton = (LedPin, PulsePin, ButtonPin);

static GLOBAL_PINS: Mutex<RefCell<Option<LedPulseAndButton>>> = Mutex::new(RefCell::new(None));

pub struct PicoDevice {
    delay: Delay,
}

impl PicoDevice {
    pub fn new() -> PicoDevice {
        debug!("Initializing device");

        // External high-speed crystal on the pico board is 12Mhz
        let external_xtal_freq_hz = 12_000_000u32;

        let mut pac = pac::Peripherals::take().unwrap();
        let core = pac::CorePeripherals::take().unwrap();
        let mut watchdog = Watchdog::new(pac.WATCHDOG);
        let sio = Sio::new(pac.SIO);

        let clocks = init_clocks_and_plls(
            external_xtal_freq_hz,
            pac.XOSC,
            pac.CLOCKS,
            pac.PLL_SYS,
            pac.PLL_USB,
            &mut pac.RESETS,
            &mut watchdog,
        )
        .ok()
        .unwrap();

        let pins = bsp::Pins::new(
            pac.IO_BANK0,
            pac.PADS_BANK0,
            sio.gpio_bank0,
            &mut pac.RESETS,
        );

        // Init pins
        let led_pin = pins.led.into_push_pull_output();
        let pulse_pin = pins.gpio18.into_push_pull_output();
        let sw1_pin = pins.gpio16.into_pull_up_input();

        // Enable interrupts
        sw1_pin.set_interrupt_enabled(EdgeLow, true);

        // Transfer ownership of pins into `GLOBAL_PINS` variable
        // for sole use by the interrupt handler.
        critical_section::with(|cs| {
            GLOBAL_PINS.borrow(cs).replace(
                Some((led_pin, pulse_pin, sw1_pin))
            );
        });

        // Init delay struct
        let delay: Delay = Delay::new(
            core.SYST, 
            clocks.system_clock.get_freq().to_Hz()
        );

        // Enable interrupts
        unsafe {
            pac::NVIC::unmask(pac::Interrupt::IO_IRQ_BANK0);
        }

        PicoDevice {
            delay: delay,
        }
    }

    fn delay_ms(&mut self, ms: u32) {
        self.delay.delay_ms(ms)
    }

}

#[entry]
fn main() -> ! {
    debug!("Program start");
    let mut dev = PicoDevice::new();

    loop {
        dev.delay_ms(1000);
        debug!("Heartbeat");
    }
}

#[interrupt]
fn IO_IRQ_BANK0() {
    // The `#[interrupt]` attribute covertly converts this to `&'static mut Option<LedAndButton>`
    static mut LED_AND_BUTTON: Option<LedPulseAndButton> = None;

    // This is one-time lazy initialisation. We steal the variables given to us
    // via `GLOBAL_PINS`.
    if LED_AND_BUTTON.is_none() {
        critical_section::with(|cs| {
            *LED_AND_BUTTON = GLOBAL_PINS.borrow(cs).take();
        });
    }

    if let Some(gpios) = LED_AND_BUTTON {
        let (led, pulse_pin, button) = gpios;

        if button.interrupt_status(EdgeLow) {
            _ = led.toggle();
            _ = pulse_pin.toggle();
            button.clear_interrupt(EdgeLow);
        }
    }
}