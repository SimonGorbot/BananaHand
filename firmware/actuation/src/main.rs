#![no_std]
#![no_main]

mod fmt;

#[cfg(not(feature = "defmt"))]
use panic_halt as _;
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

use embassy_executor::Spawner;
use embassy_stm32::{
    Config,
    gpio::Pin,
    pac::{HRTIM1, hrtim},
};
use embassy_stm32::{
    gpio::{Level, Output, Speed},
    pac,
};
use embassy_time::{Duration, Timer};
use fmt::info;

// Timer Index:
const TIM_A: usize = 0;

// Channel Index:
const CH1: usize = 0;
const CH2: usize = 1;

// Comparator Index:
const CMP1: usize = 0;
const CMP2: usize = 1;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());
    let mut led = Output::new(p.PA5, Level::High, Speed::Low);

    let rcc = pac::RCC;

    rcc.ahb2enr().modify(|w| w.set_gpioaen(true));

    rcc.apb2enr().modify(|w| w.set_hrtim1en(true));

    let gpioa = pac::GPIOA;

    gpioa.moder().modify(|w| {
        w.set_moder(8, pac::gpio::vals::Moder::ALTERNATE);
        w.set_moder(9, pac::gpio::vals::Moder::ALTERNATE);
    });

    gpioa.pupdr().modify(|w| {
        w.set_pupdr(8, pac::gpio::vals::Pupdr::FLOATING);
        w.set_pupdr(9, pac::gpio::vals::Pupdr::FLOATING);
    });

    gpioa.ospeedr().modify(|w| {
        w.set_ospeedr(8, pac::gpio::vals::Ospeedr::VERY_HIGH_SPEED);
        w.set_ospeedr(9, pac::gpio::vals::Ospeedr::VERY_HIGH_SPEED);
    });

    // AFRH index 0 -> pin 8, index 1 -> pin 9. AF13 for both.
    gpioa.afr(1).modify(|w| {
        w.set_afr(0, 13);
        w.set_afr(1, 13);
    });

    let hrtim1 = pac::HRTIM1;
    // Set Timer A (idx 0)  to operate in continuous mode.
    hrtim1.tim(TIM_A).cr().modify(|w| w.set_cont(true));

    // The HRTIM clocked by what appears as a  5.376Ghz clock (168Mhz x 32 = 5.376Ghz).
    // The counting period of the clock is selected by writing to a 16-bit register using the formula:
    // PER = T_count / T_hrtim
    // Example: Want a period of 10us? PER = 10us / 0.0002us = 50,000

    // Set Timer A (idx 0) period to 10us.
    hrtim1.tim(TIM_A).per().modify(|w| w.set_per(0xB400));

    // The X% duty cycle is obtained by multiplying the period by the duty cycle: PER x DC.

    // Set Timer A Compare 1 to 50% of the Timer A period (Will result in 50% DC)
    hrtim1.tim(TIM_A).cmp(CMP1).modify(|w| w.set_cmp(0x5A00));

    // Set Timer A Compare 1 to 25% of the Timer A period (Will result in 25% DC)
    hrtim1.tim(TIM_A).cmp(CMP2).modify(|w| w.set_cmp(0x2D00));

    hrtim1.tim(TIM_A).setr(CH1).modify(|w| w.set_per(true)); // Tim A Ch1 set on Tim A period
    hrtim1
        .tim(TIM_A)
        .rstr(CH1)
        .modify(|w| w.set_cmp(CMP1, true)); // Tim A Ch1 reset on Tim A CMP1 event

    hrtim1.tim(TIM_A).setr(CH2).modify(|w| w.set_per(true)); // Tim A Ch2 set on Tim A period
    hrtim1
        .tim(TIM_A)
        .rstr(CH2)
        .modify(|w| w.set_cmp(CMP2, true)); // Tim A Ch2 reset on Tim A CMP2 event

    hrtim1.mcr().modify(|w| w.set_mcen(true)); // Enable hrtim globally
    hrtim1.mcr().modify(|w| w.set_tcen(TIM_A, true)); // Start Tim A
    hrtim1.oenr().modify(|w| w.set_t1oen(TIM_A, true)); // Enable Tim A Ch1 output
    hrtim1.oenr().modify(|w| w.set_t2oen(TIM_A, true)); // Enable Tim A Ch2 output

    loop {
        info!("Hello, World!");
        led.set_high();
        Timer::after(Duration::from_millis(500)).await;
        led.set_low();
        Timer::after(Duration::from_millis(500)).await;
    }
}
