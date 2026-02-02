use embassy_stm32::mode::Blocking;
use embassy_stm32::usart::UartRx;

use crate::shared_force::SharedForceData;

const HEADER: [u8; 2] = [0xAA, 0x55];
const PAYLOAD_LEN: usize = 20;
const FRAME_LEN: usize = PAYLOAD_LEN + 1;

#[embassy_executor::task]
pub async fn c0_reader_task(
    mut rx: UartRx<'static, Blocking>,
    shared: &'static SharedForceData,
) {
    let mut frame = [0u8; FRAME_LEN];
    let mut readings = [0u16; 10];

    loop {
        let mut byte = [0u8; 1];
        if rx.blocking_read(&mut byte).is_err() {
            continue;
        }
        if byte[0] != HEADER[0] {
            continue;
        }

        if rx.blocking_read(&mut byte).is_err() {
            continue;
        }
        if byte[0] != HEADER[1] {
            continue;
        }

        if rx.blocking_read(&mut frame).is_err() {
            continue;
        }

        let checksum = frame[PAYLOAD_LEN];
        let sum = frame[..PAYLOAD_LEN]
            .iter()
            .fold(0u8, |acc, &b| acc.wrapping_add(b));
        if sum != checksum {
            continue;
        }

        for idx in 0..10 {
            let lo = frame[idx * 2];
            let hi = frame[idx * 2 + 1];
            readings[idx] = u16::from_le_bytes([lo, hi]);
        }

        shared.write_frame(&readings);
    }
}
