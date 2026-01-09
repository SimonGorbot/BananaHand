#![no_std]
#![no_main]

mod fmt;
mod force_sensor;

#[cfg(not(feature = "defmt"))]
use panic_halt as _;
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

use embassy_executor::Spawner;
use embassy_stm32::{
    adc::{Adc, AdcChannel, SampleTime},
    gpio::{Level, Output, Speed},
};
use embassy_time::{Duration, Timer};
use fmt::info;
use force_sensor::ForceSensor;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());
    let mut led = Output::new(p.PB7, Level::High, Speed::Low);
    let mut adc = Adc::new(p.ADC1);

    let mut force_sensor = ForceSensor::new(p.PA0.degrade_adc(), SampleTime::CYCLES480);
    loop {
        let force_v = force_sensor.read_voltage_blocking(&mut adc);
        info!("Force sensor voltage: {} V", force_v);
        led.set_high();
        Timer::after(Duration::from_millis(500)).await;
        led.set_low();
        Timer::after(Duration::from_millis(500)).await;
    }
}