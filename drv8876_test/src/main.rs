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
    adc::{Adc, SampleTime},
    gpio::{Level, Output, OutputType, Speed},
    peripherals::TIM3,
    time::khz,
    timer::{
        Ch2,
        simple_pwm::{PwmPin, SimplePwm},
    },
};
use embassy_time::Timer;
use fmt::info;

// const VREF_POS_UPPER: f32 = 3.3;
// const VREF_POS_LOWER: f32 = 0.0;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut config = Config::default();
    {
        use embassy_stm32::rcc::*;
        config.rcc.pll = Some(Pll {
            source: PllSource::HSI,
            prediv: PllPreDiv::DIV4,
            mul: PllMul::MUL85,
            divp: None,
            divq: None,
            // Main system clock at 170 MHz
            divr: Some(PllRDiv::DIV2),
        });
        config.rcc.mux.adc12sel = mux::Adcsel::SYS;
        config.rcc.sys = Sysclk::PLL1_R;
    }
    let mut p = embassy_stm32::init(config);
    info!("Peripherals initialized.");

    let mut adc = Adc::new(p.ADC1);
    adc.set_sample_time(SampleTime::CYCLES12_5);
    info!("ADC initialized.");

    // let motor_dir_pin: Output<'_> = Output::new(p.PA9, Level::High, Speed::High);
    // info!("Motor direction pin initialized.");

    let motor_pwm_pin: PwmPin<'_, TIM3, Ch2> = PwmPin::new(p.PC7, OutputType::PushPull); // PWM/D9 On nucleo
    let mut pwm = SimplePwm::new(
        p.TIM3,
        None,
        Some(motor_pwm_pin),
        None,
        None,
        khz(10),
        Default::default(),
    );
    let mut ch2 = pwm.ch2();
    ch2.enable();
    info!("PWM initialized.");
    info!("PWM max duty {}", ch2.max_duty_cycle());

    let mut led = Output::new(p.PA5, Level::High, Speed::Low);

    info!("Entering test loop.");
    loop {
        let measured = adc_read_to_volts(adc.blocking_read(&mut p.PA0)); // A0 on nucleo
        ch2.set_duty_cycle_fraction(1, 4);
        led.set_high();
        info!("Set PWM: 25%, LED: On, ADC Measurement: {}", measured);
        Timer::after_millis(1000).await;

        let measured = adc_read_to_volts(adc.blocking_read(&mut p.PA0)); // A0 on nucleo
        ch2.set_duty_cycle_fraction(1, 2);
        led.set_low();
        info!("Set PWM: 50%, LED: Off, ADC Measurement: {}", measured);
        Timer::after_millis(1000).await;

        let measured = adc_read_to_volts(adc.blocking_read(&mut p.PA0)); // A0 on nucleo
        ch2.set_duty_cycle_fraction(3, 4);
        led.set_high();
        info!("Set PWM: 75%, LED: On, ADC Measurement: {}", measured);
        Timer::after_millis(1000).await;

        let measured = adc_read_to_volts(adc.blocking_read(&mut p.PA0)); // A0 on nucleo
        ch2.set_duty_cycle(ch2.max_duty_cycle() - 1);
        led.set_low();
        info!("Set PWM: 99%, LED: Off, ADC Measurement: {}", measured);
        Timer::after_millis(1000).await;
    }
}

fn adc_read_to_volts(adc_reading: u16) -> f32 {
    const U12_MAX: u16 = 4095;
    const ADC_VREF: f32 = 3.3; // Based on the nucleo schematic it should be 3.25V, but 3.3V seems to give more accurate readings. Need to investigate further. 
    (adc_reading as f32 / U12_MAX as f32) * ADC_VREF
}
