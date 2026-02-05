//! Expose *simple* **but unsafe** PWM interface for Hrtim
use embassy_stm32::{self, hrtim, pac, peripherals::HRTIM1};

use paste::paste;
#[cfg_attr(feature = "defmt", derive(::defmt::Format))]
#[derive(Debug, Copy, Clone)]
pub enum HrtimError {
    GpioInitFailed,
    ChannelNotSetup,
}

/// HRTIM Subtimer.
#[repr(usize)]
#[derive(Debug, Copy, Clone)]
pub enum HrtimSubTimer {
    TimA = 0,
    TimB = 1,
    TimC = 2,
    TimD = 3,
    TimE = 4,
    TimF = 5,
}

/// Prescaler applied from HRTIM clock frequency to sub timer.
#[allow(unused)]
#[repr(u8)]
#[derive(Debug, Copy, Clone)]
pub enum Prescaler {
    DIV1 = 0,
    DIV2 = 1,
    DIV4 = 2,
    DIV8 = 3,
    DIV16 = 4,
    DIV32 = 5,
    DIV64 = 6,
    DIV128 = 7,
}

/// Letter group of pin (e.g. A, B, C, ...)
#[repr(usize)]
#[derive(Debug, Copy, Clone)]
pub enum PinPort {
    A = 0,
    B = 1,
    C = 2,
    D = 3,
    E = 4,
}

impl TryFrom<u8> for PinPort {
    type Error = u8;

    fn try_from(v: u8) -> Result<Self, Self::Error> {
        match v {
            0 => Ok(PinPort::A),
            1 => Ok(PinPort::B),
            2 => Ok(PinPort::C),
            3 => Ok(PinPort::D),
            4 => Ok(PinPort::E),
            other => Err(other),
        }
    }
}

/// Subtimer X channel
#[repr(usize)]
#[derive(Debug, Copy, Clone)]
pub enum HrtimXChannel {
    Ch1 = 0,
    Ch2 = 1,
}

/// Subtimer X comparator (a.k.a crossbar)
#[repr(usize)]
#[derive(Debug, Copy, Clone)]
pub enum HrtimXCompare {
    Cmp1 = 0,
    Cmp2 = 1,
}

macro_rules! gpio_x_setup {
    ($port:ident, $pin:expr, $af:expr) => {{
        paste! {
            // Enable GPIO clock: set_gpioaen / set_gpioben / ...
            pac::RCC.ahb2enr().modify(|w| w.[<set_gpio $port:lower en>](true));

            // Pick GPIOA / GPIOB / ...
            let gpio = pac::[<GPIO $port:upper>];

            gpio.moder().modify(|w| w.set_moder($pin, pac::gpio::vals::Moder::ALTERNATE));
            gpio.pupdr().modify(|w| w.set_pupdr($pin, pac::gpio::vals::Pupdr::FLOATING));
            gpio.ospeedr().modify(|w| w.set_ospeedr($pin, pac::gpio::vals::Ospeedr::VERY_HIGH_SPEED));

            // Set appropriate alternate function. AFRL: pins 0..=7, AFRH: pins 8..=15
            match $pin {
                0..=7 => { gpio.afr(0).modify(|w| w.set_afr($pin, $af)); Ok(()) }
                8..=15 => { gpio.afr(1).modify(|w| w.set_afr($pin - 8, $af)); Ok(()) }
                _ => Err(HrtimError::GpioInitFailed),
            }
        }
    }};
}

pub struct NoTimer;
pub struct NoChannel;

/// Zero-sized marker returned when a timer is absent but we still call `activate()`.
pub struct NoTimerActive;

/// Trait for anything that can be turned into an active, PWM-capable handle.
pub trait Activate {
    type Active;
    fn activate(self) -> Result<Self::Active, HrtimError>;
}

// Marker traits to know at whether a channel is setup (present)
trait Ch1MarkerA {
    const PRESENT: bool;
    fn setup_gpio(&self) -> Result<(), HrtimError>;
}

trait Ch2MarkerA {
    const PRESENT: bool;
    fn setup_gpio(&self) -> Result<(), HrtimError>;
}

impl Ch1MarkerA for NoChannel {
    const PRESENT: bool = false;
    fn setup_gpio(&self) -> Result<(), HrtimError> {
        Ok(())
    }
}

impl Ch2MarkerA for NoChannel {
    const PRESENT: bool = false;
    fn setup_gpio(&self) -> Result<(), HrtimError> {
        Ok(())
    }
}

impl<P> Ch1MarkerA for P
where
    P: hrtim::ChannelAPin<HRTIM1>,
{
    const PRESENT: bool = true;
    fn setup_gpio(&self) -> Result<(), HrtimError> {
        let alt_func_num = self.af_num();
        let pin_num = self.pin() as usize;

        match PinPort::try_from(self.port()) {
            Ok(PinPort::A) => gpio_x_setup!(a, pin_num, alt_func_num)?,
            Ok(PinPort::B) => gpio_x_setup!(b, pin_num, alt_func_num)?,
            Ok(PinPort::C) => gpio_x_setup!(c, pin_num, alt_func_num)?,
            Ok(PinPort::D) => gpio_x_setup!(d, pin_num, alt_func_num)?,
            Ok(PinPort::E) => gpio_x_setup!(e, pin_num, alt_func_num)?,
            Err(v) => panic!("Invalid port value: {v}"),
        }
        Ok(())
    }
}

impl<P> Ch2MarkerA for P
where
    P: hrtim::ChannelAComplementaryPin<HRTIM1>,
{
    const PRESENT: bool = true;
    fn setup_gpio(&self) -> Result<(), HrtimError> {
        let alt_func_num = self.af_num();
        let pin_num = self.pin() as usize;

        match PinPort::try_from(self.port()) {
            Ok(PinPort::A) => gpio_x_setup!(a, pin_num, alt_func_num)?,
            Ok(PinPort::B) => gpio_x_setup!(b, pin_num, alt_func_num)?,
            Ok(PinPort::C) => gpio_x_setup!(c, pin_num, alt_func_num)?,
            Ok(PinPort::D) => gpio_x_setup!(d, pin_num, alt_func_num)?,
            Ok(PinPort::E) => gpio_x_setup!(e, pin_num, alt_func_num)?,
            Err(v) => panic!("Invalid port value: {v}"),
        }
        Ok(())
    }
}

#[derive(Debug)]
pub struct HrtimCore<A = NoTimer, B = NoTimer> {
    a: A,
    b: B,
}

impl HrtimCore<NoTimer, NoTimer> {
    pub fn new() -> Self {
        Self {
            a: NoTimer,
            b: NoTimer,
        }
    }
}

// Subtimer A Config

#[derive(Debug)]
pub struct SubTimerAConfig<Ch1 = NoChannel, Ch2 = NoChannel> {
    ch1: Ch1,
    ch2: Ch2,
    period: u16,
    clock_prescaler: Prescaler,
}

impl SubTimerAConfig<NoChannel, NoChannel> {
    pub fn new(period: u16, clock_prescaler: Prescaler) -> Self {
        Self {
            ch1: NoChannel,
            ch2: NoChannel,
            period,
            clock_prescaler,
        }
    }
}

impl<Ch2> SubTimerAConfig<NoChannel, Ch2> {
    pub fn with_ch1<P1: hrtim::ChannelAPin<HRTIM1>>(self, ch1: P1) -> SubTimerAConfig<P1, Ch2> {
        SubTimerAConfig {
            ch1,
            ch2: self.ch2,
            period: self.period,
            clock_prescaler: self.clock_prescaler,
        }
    }
}

impl<Ch1> SubTimerAConfig<Ch1, NoChannel> {
    pub fn with_ch2<P2: hrtim::ChannelAComplementaryPin<HRTIM1>>(
        self,
        ch2: P2,
    ) -> SubTimerAConfig<Ch1, P2> {
        SubTimerAConfig {
            ch1: self.ch1,
            ch2,
            period: self.period,
            clock_prescaler: self.clock_prescaler,
        }
    }
}

// Methods to add subtimer A
impl<B> HrtimCore<NoTimer, B> {
    pub fn add_tim_a<T>(self, timer: T) -> HrtimCore<T, B> {
        HrtimCore {
            a: timer,
            b: self.b,
        }
    }

    /// Add subtimer config with channel 1 present.
    pub fn add_tim_a_ch1<P1>(
        self,
        ch1: P1,
        period: u16,
        clock_prescaler: Prescaler,
    ) -> HrtimCore<SubTimerAConfig<P1, NoChannel>, B>
    where
        P1: hrtim::ChannelAPin<HRTIM1>,
    {
        self.add_tim_a(SubTimerAConfig::new(period, clock_prescaler).with_ch1(ch1))
    }

    /// Add subtimer config with channel 2 present.
    pub fn add_tim_a_ch2<P2>(
        self,
        ch2: P2,
        period: u16,
        clock_prescaler: Prescaler,
    ) -> HrtimCore<SubTimerAConfig<NoChannel, P2>, B>
    where
        P2: hrtim::ChannelAComplementaryPin<HRTIM1>,
    {
        self.add_tim_a(SubTimerAConfig::new(period, clock_prescaler).with_ch2(ch2))
    }

    /// Add subtimer config with channel 1 and channel 2 present.
    pub fn add_tim_a_ch1_ch2<P1, P2>(
        self,
        ch1: P1,
        ch2: P2,
        period: u16,
        clock_prescaler: Prescaler,
    ) -> HrtimCore<SubTimerAConfig<P1, P2>, B>
    where
        P1: hrtim::ChannelAPin<HRTIM1>,
        P2: hrtim::ChannelAComplementaryPin<HRTIM1>,
    {
        self.add_tim_a(
            SubTimerAConfig::new(period, clock_prescaler)
                .with_ch1(ch1)
                .with_ch2(ch2),
        )
    }
}

// Subtimer A Pwm Handle

pub struct SubtimerAPwm<Ch1, Ch2> {
    ch1: Ch1,
    ch2: Ch2,
    period: u16,
    clock_prescaler: Prescaler,
}

impl<Ch1, Ch2> SubTimerAConfig<Ch1, Ch2>
where
    Ch1: Ch1MarkerA,
    Ch2: Ch2MarkerA,
{
    /// Consume the config, perform hardware configuration, and return a PWM-capable handle.
    pub fn activate(self) -> Result<SubtimerAPwm<Ch1, Ch2>, HrtimError> {
        // GPIO + output compare setup only for present channels
        if Ch1::PRESENT {
            self.ch1.setup_gpio()?;
            configure_channel(HrtimSubTimer::TimA, HrtimXChannel::Ch1, HrtimXCompare::Cmp1);
        }
        if Ch2::PRESENT {
            self.ch2.setup_gpio()?;
            configure_channel(HrtimSubTimer::TimA, HrtimXChannel::Ch2, HrtimXCompare::Cmp2);
        }

        configure_timer(HrtimSubTimer::TimA, self.clock_prescaler, self.period);

        Ok(SubtimerAPwm {
            ch1: self.ch1,
            ch2: self.ch2,
            period: self.period,
            clock_prescaler: self.clock_prescaler,
        })
    }
}

impl<Ch1, Ch2> SubtimerAPwm<Ch1, Ch2>
where
    Ch1: hrtim::ChannelAPin<HRTIM1>,
{
    pub fn ch1_en(&self) {
        enable_output(HrtimSubTimer::TimA, HrtimXChannel::Ch1);
    }

    pub fn ch1_set_dc_percent(&self, dc: u8) {
        set_cmp_for_dc(HrtimSubTimer::TimA, HrtimXCompare::Cmp1, self.period, dc);
    }
}

impl<Ch1, Ch2> SubtimerAPwm<Ch1, Ch2>
where
    Ch2: hrtim::ChannelAComplementaryPin<HRTIM1>,
{
    pub fn ch2_en(&self) {
        enable_output(HrtimSubTimer::TimA, HrtimXChannel::Ch2);
    }

    pub fn ch2_set_dc_percent(&self, dc: u8) {
        set_cmp_for_dc(HrtimSubTimer::TimA, HrtimXCompare::Cmp2, self.period, dc);
    }
}

impl<Ch1, Ch2> SubtimerAPwm<Ch1, Ch2> {
    pub fn get_period(&self) -> u16 {
        self.period
    }

    pub fn get_clock_prescaler(&self) -> Prescaler {
        self.clock_prescaler
    }

    pub fn set_period(&self, period: u16) {
        configure_timer(HrtimSubTimer::TimA, self.clock_prescaler, period);
    }

    pub fn set_clock_prescaler(&self, prescaler: Prescaler) {
        configure_timer(HrtimSubTimer::TimA, prescaler, self.period);
    }
}

// HrtimCore General Impls

impl<A, B> HrtimCore<A, B> {
    /// Consume the `HrtimCore` and return the configured subtimers so the caller
    /// can `activate()` and use PWM (set duty, enable outputs, etc.) on configured channels.
    pub fn split(self) -> (A, B) {
        (self.a, self.b)
    }
}

impl<A: Activate, B: Activate> HrtimCore<A, B> {
    /// Consume the `HrtimCore` and return activated subtimers for those that were configured
    /// and `NoTimerActive` for absent timers become.
    pub fn split_active(self) -> Result<(A::Active, B::Active), HrtimError> {
        Ok((self.a.activate()?, self.b.activate()?))
    }
}

// Activate Impls

impl Activate for NoTimer {
    type Active = NoTimerActive;
    fn activate(self) -> Result<Self::Active, HrtimError> {
        Ok(NoTimerActive)
    }
}

impl<Ch1, Ch2> Activate for SubTimerAConfig<Ch1, Ch2>
where
    Ch1: Ch1MarkerA,
    Ch2: Ch2MarkerA,
{
    type Active = SubtimerAPwm<Ch1, Ch2>;
    fn activate(self) -> Result<Self::Active, HrtimError> {
        if Ch1::PRESENT {
            self.ch1.setup_gpio()?;
            configure_channel(HrtimSubTimer::TimA, HrtimXChannel::Ch1, HrtimXCompare::Cmp1);
        }
        if Ch2::PRESENT {
            self.ch2.setup_gpio()?;
            configure_channel(HrtimSubTimer::TimA, HrtimXChannel::Ch2, HrtimXCompare::Cmp2);
        }

        configure_timer(HrtimSubTimer::TimA, self.clock_prescaler, self.period);

        Ok(SubtimerAPwm {
            ch1: self.ch1,
            ch2: self.ch2,
            period: self.period,
            clock_prescaler: self.clock_prescaler,
        })
    }
}

// PAC helpers used above

fn configure_timer(timer: HrtimSubTimer, prescaler: Prescaler, period: u16) {
    pac::HRTIM1
        .tim(timer as usize)
        .cr()
        .modify(|w| w.set_cont(true));

    pac::HRTIM1
        .tim(timer as usize)
        .cr()
        .modify(|w| w.set_ckpsc(prescaler as u8));

    pac::HRTIM1
        .tim(timer as usize)
        .per()
        .modify(|w| w.set_per(period));

    pac::HRTIM1
        .mcr()
        .modify(|w| w.set_tcen(timer as usize, true));
}

fn configure_channel(timer: HrtimSubTimer, channel: HrtimXChannel, comparator: HrtimXCompare) {
    pac::HRTIM1
        .tim(timer as usize)
        .setr(channel as usize)
        .modify(|w| w.set_per(true));

    pac::HRTIM1
        .tim(timer as usize)
        .rstr(channel as usize)
        .modify(|w| w.set_cmp(comparator as usize, true));
}

fn enable_output(timer: HrtimSubTimer, channel: HrtimXChannel) {
    match channel {
        HrtimXChannel::Ch1 => pac::HRTIM1
            .oenr()
            .modify(|w| w.set_t1oen(timer as usize, true)),
        HrtimXChannel::Ch2 => pac::HRTIM1
            .oenr()
            .modify(|w| w.set_t2oen(timer as usize, true)),
    }
}

fn set_cmp_for_dc(timer: HrtimSubTimer, cmp: HrtimXCompare, period: u16, percent: u8) {
    let cmp_set = (period as f32 * (percent as f32 / 100.0)) as u16;
    pac::HRTIM1
        .tim(timer as usize)
        .cmp(cmp as usize)
        .modify(|w| w.set_cmp(cmp_set));
}
