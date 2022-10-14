//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

use cortex_m::asm::wfi;
use cortex_m_rt::entry;
use embedded_hal::digital::v2::ToggleableOutputPin;
use fugit::MillisDurationU32;

use bsp::hal::{
    gpio::pin::bank0::{Gpio16, Gpio18, Gpio25},
    gpio::pin::{Pin, PullUpInput, PushPullOutput},
    gpio::Interrupt::EdgeLow,
    pac,
    pac::interrupt,
    sio::Sio,
    timer::{Alarm, Alarm0, Timer},
};
use rp_pico as bsp;

use defmt::*;
use defmt_rtt as _;
use panic_probe as _;

use core::cell::RefCell;
use critical_section::Mutex;

const DEBOUNCE_DELAY: MillisDurationU32 = MillisDurationU32::millis(100);

struct InterruptPeripherals {
    led_pin: Pin<Gpio25, PushPullOutput>,
    button_pin: Pin<Gpio16, PullUpInput>,
    pulse_pin: Pin<Gpio18, PushPullOutput>,
    alarm: Alarm0,
}
static INTERRUPT_PERIPHERALS: Mutex<RefCell<Option<InterruptPeripherals>>> =
    Mutex::new(RefCell::new(None));


#[entry]
fn main() -> ! {
    debug!("Initializing device");

    let mut pac = pac::Peripherals::take().unwrap();
    let sio = Sio::new(pac.SIO);

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut timer = Timer::new(pac.TIMER, &mut pac.RESETS);
    let mut alarm = timer.alarm_0().unwrap();

    // Init GPIO pins
    let led_pin = pins.led.into_push_pull_output();
    let pulse_pin = pins.gpio18.into_push_pull_output();
    let button_pin = pins.gpio16.into_pull_up_input();

    // Enable interrupt for button pin and alarm (won't trigger until bank enabled)
    button_pin.set_interrupt_enabled(EdgeLow, true);
    alarm.enable_interrupt();

    // Transfer ownership of pins into `GLOBAL_PINS` variable
    // for sole use by the interrupt handler.
    let interrupt_periphs = InterruptPeripherals {
        led_pin,
        button_pin,
        pulse_pin,
        alarm,
    };
    critical_section::with(|cs| {
        INTERRUPT_PERIPHERALS
            .borrow(cs)
            .replace(Some(interrupt_periphs));
    });

    // Enable interrupt and timer
    unsafe {
        pac::NVIC::unmask(pac::Interrupt::IO_IRQ_BANK0);
        pac::NVIC::unmask(pac::Interrupt::TIMER_IRQ_0);
    }

    // Loop forever doing nothing
    loop {
        wfi();
    }
}

#[interrupt]
fn IO_IRQ_BANK0() {
    critical_section::with(|cs| {
        // Temporarily take peripherals in protected critical section
        if let Some(mut periphs) = INTERRUPT_PERIPHERALS.borrow(cs).take() {
            if periphs.button_pin.interrupt_status(EdgeLow) {
                if periphs.alarm.finished() {
                    _ = periphs.led_pin.toggle();
                    _ = periphs.pulse_pin.toggle();
                    _ = periphs.alarm.schedule(DEBOUNCE_DELAY);
                }

                periphs.button_pin.clear_interrupt(EdgeLow);
            }
        INTERRUPT_PERIPHERALS
            .borrow(cs)
            .replace_with(|_| Some(periphs));
        }
    });
}

#[interrupt]
fn TIMER_IRQ_0() {
    critical_section::with(|cs| {
        // Temporarily take peripherals in protected critical section
        if let Some(mut periphs) = INTERRUPT_PERIPHERALS.borrow(cs).take() {
            if periphs.alarm.finished() {
                periphs.alarm.clear_interrupt();
            }

        INTERRUPT_PERIPHERALS
            .borrow(cs)
            .replace_with(|_| Some(periphs));
        }
    });
}