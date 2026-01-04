#![no_std]
#![no_main]

mod fmt;

#[cfg(not(feature = "defmt"))]
use panic_halt as _;
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

use embassy_executor::Spawner;
use embassy_stm32::{
    adc::SampleTime,
    gpio::{Level, Output, Speed},
    hrtim::{AdvancedPwm, ChA, ComplementaryPwmPin, PwmPin},
    peripherals::{ADC1, DMA1_CH1, HRTIM1, TIM1},
    timer::{GeneralInstance4Channel, simple_pwm::SimplePwmChannel},
};
use embassy_time::{Duration, Ticker, Timer};
use fmt::info;

use core::sync::atomic::{AtomicU16, AtomicU32, Ordering};

pub struct Pq12PCal;
impl Pq12PCal {
    pub const STROKE_LENGTH: f32 = 20.0;
    pub const ADC_MAX_RAW: u16 = 4095;

    pub fn raw_to_mm(raw: u16) -> f32 {
        (raw as f32) * (Self::STROKE_LENGTH / Self::ADC_MAX_RAW as f32)
    }
}

pub struct Pq12PDrive<'a, T: GeneralInstance4Channel> {
    pwm_1: SimplePwmChannel<'a, T>,
    pwm_2: SimplePwmChannel<'a, T>,
}

impl<'a, T: GeneralInstance4Channel> Pq12PDrive<'a, T> {
    /// Set motor to coast (PWM1 = 0 PWM2 = 0).
    pub fn coast(&mut self) {
        self.pwm_1.set_duty_cycle_fully_off();
        self.pwm_2.set_duty_cycle_fully_off();
    }

    /// Set motor to brake (PWM1 = 1 PWM2 = 1).
    pub fn brake(&mut self) {
        self.pwm_1.set_duty_cycle_fully_on();
        self.pwm_2.set_duty_cycle_fully_on();
    }

    /// Move PQ12 plunger out (PWM1 = DC% PWM2 = 0).
    pub fn move_out(&mut self, duty_cycle_percent: u8) {
        let dcp = duty_cycle_percent.clamp(0, 100);
        self.pwm_1.set_duty_cycle_fully_on();
        self.pwm_2.set_duty_cycle_percent(100 - dcp);
    }

    /// Move PQ12 plunger in (PWM1 = 0 PWM2 = DC%).
    pub fn move_in(&mut self, duty_cycle_percent: u8) {
        let dcp = duty_cycle_percent.clamp(0, 100);
        self.pwm_1.set_duty_cycle_percent(100 - dcp);

        self.pwm_2.set_duty_cycle_fully_on();
    }
}

pub const NUM_MOTORS: usize = 5;
pub static POSITION_RAW: [AtomicU16; NUM_MOTORS] = [
    AtomicU16::new(0),
    AtomicU16::new(0),
    AtomicU16::new(0),
    AtomicU16::new(0),
    AtomicU16::new(0),
];

pub static POSITION_SEQ: AtomicU32 = AtomicU32::new(0);

#[embassy_executor::task]
pub async fn adc_sampler_task(
    mut adc: embassy_stm32::adc::Adc<'static, ADC1>,
    mut dma: embassy_stm32::Peri<'static, DMA1_CH1>,
    mut pos_ch: [embassy_stm32::adc::AnyAdcChannel<ADC1>; NUM_MOTORS],
) -> ! {
    let mut buf = [0u16; NUM_MOTORS];
    let mut ticker = Ticker::every(Duration::from_hz(400)); // sample faster than control

    loop {
        // Read all 5 position channels in one shot
        adc.read(
            dma.reborrow(),
            pos_ch.iter_mut().map(|ch| (ch, SampleTime::CYCLES12_5)),
            &mut buf,
        )
        .await;

        for (i, &raw) in buf.iter().enumerate() {
            POSITION_RAW[i].store(raw, Ordering::Relaxed);
        }
        POSITION_SEQ.fetch_add(1, Ordering::Release);

        ticker.next().await;
    }
}

#[derive(Clone, Copy)]
pub struct MotorGoal {
    pub target_mm: f32,
    pub duty: u8,
    pub end_in_brake: bool,
}

#[embassy_executor::task]
pub async fn motor_control_task(
    mut motors: [Pq12PDrive<'static, TIM1>; NUM_MOTORS],
    // receive goals from UART tasks via a Channel (not shown)
) -> ! {
    use embassy_time::{Duration, Ticker};

    const CONTROL_HZ: u64 = 200;
    const TOL_MM: f32 = 0.1;

    let mut goals = [MotorGoal {
        target_mm: 10.0,
        duty: 40,
        end_in_brake: true,
    }; NUM_MOTORS];
    let mut last_seq = POSITION_SEQ.load(Ordering::Acquire);

    let mut ticker = Ticker::every(Duration::from_hz(CONTROL_HZ));
    loop {
        // (1) pull any pending goal updates from a channel here

        // (2) optional: freshness check
        let seq = POSITION_SEQ.load(Ordering::Acquire);
        let fresh = seq != last_seq;
        last_seq = seq;

        for i in 0..NUM_MOTORS {
            let raw = POSITION_RAW[i].load(Ordering::Relaxed) as u16;
            let pos_mm = Pq12PCal::raw_to_mm(raw);
            let g = goals[i];

            if !fresh {
                // if ADC stalled, fail-safe (optional)
                motors[i].coast();
                continue;
            }

            if pos_mm + TOL_MM < g.target_mm {
                motors[i].move_out(g.duty);
            } else if pos_mm - TOL_MM > g.target_mm {
                motors[i].move_in(g.duty);
            } else {
                if g.end_in_brake {
                    motors[i].brake();
                } else {
                    motors[i].coast();
                }
            }
        }

        ticker.next().await;
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());
    let mut led = Output::new(p.PA5, Level::High, Speed::Low);

    let ch_a_1 = PwmPin::new_cha(p.PA8);
    let ch_a_2 = ComplementaryPwmPin::new_cha(p.PA9);

    let pwm = AdvancedPwm::new(
        p.HRTIM1,
        Some(ch_a_1),
        Some(ch_a_2),
        None,
        None,
        None,
        None,
        None,
        None,
        None,
        None,
        None,
        None,
    );

    loop {
        info!("Hello, World!");
        led.set_high();
        Timer::after(Duration::from_millis(500)).await;
        led.set_low();
        Timer::after(Duration::from_millis(500)).await;
    }
}
