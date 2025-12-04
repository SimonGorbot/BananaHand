#![no_std]
#![no_main]

mod actuator;
mod fmt;

#[cfg(not(feature = "defmt"))]
use panic_halt as _;
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

use actuator::{Pq12P, SimplePq12P};
use embassy_executor::Spawner;
use embassy_stm32::{
    Config,
    adc::{Adc, AdcChannel, SampleTime},
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
    let p = embassy_stm32::init(config);

    info!("Peripherals initialized.");

    let mut adc = Adc::new(p.ADC1);
    adc.set_sample_time(SampleTime::CYCLES247_5);
    info!("ADC initialized.");

    let motor_dir_pin: Output<'_> = Output::new(p.PA9, Level::High, Speed::High); // D8 on nucleo
    info!("Motor direction pin initialized.");

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

    let mut actuator = SimplePq12P::new(3.3, 0.0, ch2, motor_dir_pin, p.PA0.degrade_adc());

    info!("Entering test loop.");
    loop {
        actuator.move_to_pos(1.0, 90, &mut adc);
        led.toggle();
        Timer::after_secs(1).await;
        actuator.move_to_pos(10.0, 90, &mut adc);
        led.toggle();
        Timer::after_secs(1).await;
        actuator.move_to_pos(19.0, 90, &mut adc);
        led.toggle();
        Timer::after_secs(1).await;
    }
}
