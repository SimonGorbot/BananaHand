#![no_std]
#![no_main]

mod fmt;
mod c0_reader;
mod shared_force;

#[cfg(not(feature = "defmt"))]
use panic_halt as _;
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

use embassy_executor::Spawner;
use embassy_stm32::{
    Config,
    usart::Config as UartConfig,
};
use embassy_stm32::usart::UartRx;
use embassy_time::{Duration, Timer};
use fmt::info;
use shared_force::SharedForceData;

static SHARED_FORCE: SharedForceData = SharedForceData::new();

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

    let mut uart_config = UartConfig::default();
    uart_config.baudrate = 115_200;
    let uart_rx = UartRx::new_blocking(p.USART1, p.PA10, uart_config).unwrap();
    _spawner
        .spawn(c0_reader::c0_reader_task(uart_rx, &SHARED_FORCE))
        .unwrap();

    let mut latest = [0u16; 10];
    loop {
        if SHARED_FORCE.read_frame(&mut latest) {
            info!("Force raw (ch0): {}", latest[0]);
        }
        Timer::after(Duration::from_millis(10)).await;
    }
}
