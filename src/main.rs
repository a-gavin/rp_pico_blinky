//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

use cortex_m::{asm::wfi, delay::Delay};
use cortex_m_rt::entry;
use embedded_hal::digital::v2::ToggleableOutputPin;
use fugit::{MillisDurationU32, RateExtU32};

use bsp::hal::{
    clocks::{init_clocks_and_plls, ClockSource},
    gpio,
    gpio::pin::bank0::{Gpio16, Gpio18, Gpio25},
    gpio::pin::{Pin, PullUpInput, PushPullOutput},
    gpio::Interrupt::EdgeLow,
    pac,
    pac::interrupt,
    sio::Sio,
    timer::{Alarm, Alarm0, Timer},
    watchdog::Watchdog,
    I2C,
};
use rp_pico as bsp;

use core::cell::RefCell;
use critical_section::Mutex;
use defmt::*;
use defmt_rtt as _;
use lcd_i2c::{Backlight, Lcd};
use panic_probe as _;

const DEBOUNCE_DELAY: MillisDurationU32 = MillisDurationU32::millis(100);
const LCD_ADDRESS: u8 = 0x27;

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

    // Init delay struct, timer, alarm
    let mut delay: Delay = Delay::new(core.SYST, clocks.system_clock.get_freq().to_Hz());

    let mut timer = Timer::new(pac.TIMER, &mut pac.RESETS);
    let mut alarm = timer.alarm_0().unwrap();

    // Init GPIO pins
    let led_pin = pins.led.into_push_pull_output();
    let pulse_pin = pins.gpio18.into_push_pull_output();
    let button_pin = pins.gpio16.into_pull_up_input();

    // Init I2C pins
    let sda_pin = pins.gpio2.into_mode::<gpio::FunctionI2C>();
    let scl_pin = pins.gpio3.into_mode::<gpio::FunctionI2C>();

    let mut i2c = I2C::i2c1(
        pac.I2C1,
        sda_pin,
        scl_pin,
        100.kHz(),
        &mut pac.RESETS,
        &clocks.system_clock,
    );

    // Init LCD, takes ownership of I2C
    let mut lcd = Lcd::new(&mut i2c, Backlight::Off)
        .address(LCD_ADDRESS)
        .cursor_on(true)
        .rows(4)
        .init(&mut delay)
        .unwrap();

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

    debug!("Starting print, backlight on");
    _ = lcd.backlight(Backlight::On);

    for _ in 1..1000 {
        _ = lcd.return_home(&mut delay);
        _ = delay.delay_ms(10);
        _ = lcd.write_str("we hurrr");
        _ = delay.delay_ms(2000);

        //_ = lcd.clear();
        _ = lcd.return_home(&mut delay);
        _ = delay.delay_ms(10);
        _ = lcd.write_str("we there");
        _ = delay.delay_ms(2000);
    }

    _ = lcd.backlight(Backlight::Off);
    debug!("Ending print, backlight off");

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