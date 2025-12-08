//! High-level driver for an Actuonix PQ12-P linear actuator driven by a TI DRV8876 H-bridge,
//! built on top of `embassy-stm32` for `no_std` environments.
//!
//! # Overview
//!
//! This crate provides two actuator abstractions for the PQ12-P + DRV8876 combination:
//!
//! - [`Pq12P`] – uses *dual-PWM* drive (DRV8876 in 2×PWM mode, `IN1`/`IN2` both PWM-capable).
//!   Exposes explicit `coast`, `brake`, `move_out`, and `move_in` helpers, plus current sensing
//!   via the DRV8876 IPROPI pin.
//! - [`SimplePq12P`] – uses a *single PWM* output plus a direction GPIO. Provides simple
//!   open-loop speed control and basic position feedback using the PQ12-P’s internal potentiometer,
//!   including a blocking [`SimplePq12P::move_to_pos`] primitive in millimetres.
//!
//! Both drivers are generic over `embassy-stm32` peripheral instances:
//!
//! - PWM: [`SimplePwmChannel`] on a [`GeneralInstance4Channel`] timer.
//! - ADC: [`Adc`] with [`AnyAdcChannel`] for position and/or current feedback.
//!
//! # Hardware assumptions
//!
//! - DRV8876 is wired in either:
//!   - 2×PWM mode (`IN1`/`IN2` both driven by timers), or
//!   - 1×PWM + direction GPIO mode.
//! - `nSLEEP` is held high during normal operation.
//! - ADC reference is 3.3 V and 12-bit (0–4095), matching `ADC_VREF` and `ADC_MAX_RAW`.
//! - The PQ12-P feedback pin is connected to an ADC channel, and (for [`Pq12P`]) the DRV8876
//!   IPROPI pin is also connected to an ADC channel if current measurement is desired.

#![no_std]

use embassy_stm32::{
    adc::{Adc, AnyAdcChannel, Instance},
    gpio::Output,
    timer::{simple_pwm::SimplePwmChannel, GeneralInstance4Channel},
};

/// Actuator Struct for the PQ12-P linear actuator paired with a DRV8876 for driving in 2 x PWM input mode.
///
/// DRV8876 H-bridge truth table (PWM mode, PMODE = 1).
///
/// In this mode `IN1` and `IN2` are independent logic/PWM inputs.
/// `nSLEEP = 1` is assumed during normal operation.
///
/// Motor / H-bridge behaviour:
///
/// | IN1 | IN2 | OUT1 | OUT2 | Description                          |
/// |-----|-----|------|------|--------------------------------------|
/// | 0   | 0   | Hi-Z | Hi-Z | Coast (both outputs high-impedance)  |
/// | 0   | 1   |  L   |  H   | Reverse drive (Plunger In)           |
/// | 1   | 0   |  H   |  L   | Forward drive (Plunger Out)          |
/// | 1   | 1   |  L   |  L   | Brake (slow-decay, both low-side on) |
pub struct Pq12P<'a, T: GeneralInstance4Channel, C: Instance> {
    vref_position: (f32, f32),
    pwm_1: SimplePwmChannel<'a, T>,
    pwm_2: SimplePwmChannel<'a, T>,
    pos_adc: AnyAdcChannel<C>,
    i_adc: AnyAdcChannel<C>,
}

// TODO: Add doc comments.

impl<'a, T: GeneralInstance4Channel, C: Instance> Pq12P<'a, T, C> {
    pub const STROKE_LENGTH: f32 = 20.0;
    const ADC_MAX_RAW: u16 = 4096;
    const ADC_VREF: f32 = 3.3;
    const R_IPROPI: f32 = 2490.0;

    pub fn new(
        vref_position: (f32, f32),
        pwm_1: SimplePwmChannel<'a, T>,
        pwm_2: SimplePwmChannel<'a, T>,
        pos_adc: AnyAdcChannel<C>,
        i_adc: AnyAdcChannel<C>,
    ) -> Self {
        Pq12P {
            vref_position,
            pwm_1,
            pwm_2,
            pos_adc,
            i_adc,
        }
    }

    pub fn coast(&mut self) {
        self.pwm_1.set_duty_cycle_fully_off();
        self.pwm_2.set_duty_cycle_fully_off();
    }

    pub fn brake(&mut self) {
        self.pwm_1.set_duty_cycle_fully_on();
        self.pwm_2.set_duty_cycle_fully_on();
    }

    pub fn move_out(&mut self, duty_cycle_percent: u8) {
        let dcp: u8;
        if duty_cycle_percent > 100 {
            dcp = 100;
        } else {
            dcp = duty_cycle_percent;
        }
        self.pwm_1.set_duty_cycle_percent(dcp);
        self.pwm_2.set_duty_cycle_fully_off();
    }

    pub fn move_in(&mut self, duty_cycle_percent: u8) {
        let dcp: u8;
        if duty_cycle_percent > 100 {
            dcp = 100;
        } else {
            dcp = duty_cycle_percent;
        }
        self.pwm_1.set_duty_cycle_fully_off();
        self.pwm_2.set_duty_cycle_percent(dcp);
    }

    // TODO: Allow non-snake case name
    pub fn read_current_V(&mut self, adc: &mut Adc<'_, C>) -> f32 {
        let raw_reading = adc.blocking_read(&mut self.i_adc);
        (raw_reading as f32 / Self::ADC_MAX_RAW as f32) * Self::ADC_VREF
    }

    pub fn read_current_mA(&mut self, adc: &mut Adc<'_, C>) -> f32 {
        let volts = self.read_current_V(adc);
        (volts / Self::R_IPROPI) * 1e6
    }

    pub fn read_position_v(&mut self, adc: &mut Adc<'_, C>) -> f32 {
        let raw_reading = adc.blocking_read(&mut self.pos_adc);
        (raw_reading as f32 / Self::ADC_MAX_RAW as f32) * Self::ADC_VREF
    }
}

/// Actuator Struct for the PQ12-P linear actuator paired with a DRV8876 for driving in 1 x PWM input mode.
pub struct SimplePq12P<'a, T: GeneralInstance4Channel, C: Instance> {
    pub vref_position_upper: f32,
    pub vref_position_lower: f32,
    pub pwm: SimplePwmChannel<'a, T>,
    pub dir_select: Output<'a>,
    pub adc_pin: AnyAdcChannel<C>,
}

impl<'a, T: GeneralInstance4Channel, C: Instance> SimplePq12P<'a, T, C> {
    const STROKE_LENGTH: f32 = 20.0;
    const ADC_VREF: f32 = 3.3;
    const ADC_MAX_RAW: u16 = 4096;

    pub fn new(
        vref_position_upper: f32,
        vref_position_lower: f32,
        pwm: SimplePwmChannel<'a, T>,
        dir_select: Output<'a>,
        adc_pin: AnyAdcChannel<C>,
    ) -> Self {
        SimplePq12P {
            vref_position_upper,
            vref_position_lower,
            pwm,
            dir_select,
            adc_pin,
        }
    }

    pub fn set_direction_in(&mut self) {
        self.dir_select.set_low();
    }

    pub fn set_direction_out(&mut self) {
        self.dir_select.set_high();
    }

    pub fn toggle_direction(&mut self) {
        self.dir_select.toggle();
    }

    pub fn set_speed(&mut self, percent: u8) {
        self.pwm.set_duty_cycle_percent(percent);
    }

    pub fn read_position_v(&mut self, adc: &mut Adc<'_, C>) -> f32 {
        let raw_reading = adc.blocking_read(&mut self.adc_pin);
        let pos_as_v = (raw_reading as f32 / Self::ADC_MAX_RAW as f32) * Self::ADC_VREF;
        pos_as_v
    }

    pub fn read_position_mm(&mut self, adc: &mut Adc<'_, C>) -> f32 {
        let pos_as_v = self.read_position_v(adc);
        let pos_in_mm = Self::STROKE_LENGTH * (pos_as_v - self.vref_position_lower)
            / (self.vref_position_upper - self.vref_position_lower);
        pos_in_mm
    }

    pub fn move_to_pos(
        &mut self,
        target_position_mm: f32,
        duty_cycle_percent: u8,
        adc: &mut Adc<'_, C>,
    ) {
        const TOLERANCE_MM: f32 = 0.1;

        let target = target_position_mm.clamp(1.0, 19.0);
        let mut current = self.read_position_mm(adc);

        let direction = if current + TOLERANCE_MM < target {
            self.set_direction_out();
            Some(1.0_f32)
        } else if current - TOLERANCE_MM > target {
            self.set_direction_in();
            Some(-1.0_f32)
        } else {
            None
        };

        if direction.is_none() {
            self.set_speed(0);
            return;
        }

        self.set_speed(duty_cycle_percent);

        loop {
            current = self.read_position_mm(adc);

            if (current - target).abs() <= TOLERANCE_MM {
                break;
            }
        }
        self.set_speed(0);
    }
}
