#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_stm32::bind_interrupts;
use embassy_stm32::usart::{Config, Uart};
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    USART2 => embassy_stm32::usart::InterruptHandler<embassy_stm32::peripherals::USART2>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());

    let mut config = Config::default();
    config.baudrate = 115_200;

    // UART2 pin mapping:
    //   PA3 = RX
    //   PA2 = TX
    //
    // DMA mapping (verified by compiler errors):
    //   TX DMA -> DMA1_CH6
    //   RX DMA -> DMA1_CH5
    let mut uart = Uart::new(
        p.USART2,
        p.PA3,          // RX pin
        p.PA2,          // TX pin
        Irqs, 
        p.DMA1_CH6,     // TX DMA  (MUST BE FIRST)
        p.DMA1_CH5,     // RX DMA
        config,
    ).unwrap();

    defmt::info!("UART2 initialized!");

    let mut buf = [0u8; 64];

    loop {
        uart.read_until_idle(&mut buf).await.unwrap();
        defmt::info!("Received bytes: {:?}", &buf);
        uart.write(&buf).await.unwrap();
    }
}
