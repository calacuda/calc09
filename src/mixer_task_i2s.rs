use core::mem;

use alloc::{sync::Arc, vec::Vec};
use embassy_futures::select::{Either, select};
use embassy_rp::{
    Peri,
    peripherals::{DMA_CH2, PIN_2, PIN_3, PIN_4, PIO0},
    pio::{Common, StateMachine},
    pio_programs::i2s::{PioI2sOut, PioI2sOutProgram},
};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Receiver};
use static_cell::StaticCell;

use crate::Irqs;

pub type Sample = i32;

pub static SAMPLE_RATE: u32 = 48_000;
const BIT_DEPTH: u32 = 32;

#[derive(Clone, Debug)]
pub enum MixerCMD {
    Play {
        /// audio file to play
        drum_sound: Arc<[Sample]>,
        /// volume
        volume: f32,
        /// a unique-ish id to allow for stopping playback early
        id: u8,
        // /// which channel to send the audio to.
        // output_channel: usize
    },
    Stop {
        /// the id to stop playing, it cooresponds to the id feild of the Play command
        id: u8,
    },
}

#[derive(Clone, Debug, Default)]
pub struct PlayHead {
    /// audio file to play
    pub drum_sound: Arc<[Sample]>,
    /// volume
    pub volume: f32,
    /// a unique-ish id to allow for stopping playback early
    pub id: u8,
    pub i: usize,
}

#[embassy_executor::task]
pub async fn mixer_task(
    // left: Peri<'static, PIN_26>,
    // right: Peri<'static, PIN_27>,
    // slice5: Peri<'static, PWM_SLICE5>,
    bit_clock_pin: Peri<'static, PIN_2>,
    left_right_clock_pin: Peri<'static, PIN_3>,
    data_pin: Peri<'static, PIN_4>,
    mut common: Common<'static, PIO0>,
    sm0: StateMachine<'static, PIO0, 0>,
    dma: Peri<'static, DMA_CH2>,
    cmds: Receiver<'static, CriticalSectionRawMutex, MixerCMD, 4>,
) {
    let mut play_heads: Vec<PlayHead> = Vec::new();
    let program = PioI2sOutProgram::new(&mut common);
    let mut i2s = PioI2sOut::new(
        &mut common,
        sm0,
        dma,
        Irqs,
        data_pin,
        bit_clock_pin,
        left_right_clock_pin,
        SAMPLE_RATE,
        BIT_DEPTH,
        &program,
    );
    i2s.start();
    // let mut sample = 0.0;
    // create two audio buffers (back and front) which will take turns being
    // filled with new audio data and being sent to the pio fifo using dma
    const BUFFER_SIZE: usize = 64;
    static DMA_BUFFER: StaticCell<[u32; BUFFER_SIZE * 2]> = StaticCell::new();
    let dma_buffer = DMA_BUFFER.init_with(|| [0u32; BUFFER_SIZE * 2]);
    let (mut back_buffer, mut front_buffer) = dma_buffer.split_at_mut(BUFFER_SIZE);

    loop {
        match select(i2s.write(front_buffer), cmds.receive()).await {
            Either::First(_) => {
                mem::swap(&mut back_buffer, &mut front_buffer);

                for s in back_buffer.iter_mut() {
                    let sample: Sample = {
                        let samples = play_heads.iter_mut().map(|head| {
                            let sample = head.drum_sound[head.i];
                            head.i += 1;
                            sample
                        });

                        samples.sum()
                    };
                    play_heads.retain(|head| head.i < head.drum_sound.len());
                    // let sample = libm::tanhf(sample * 0.8);
                    let sample = (sample * 4) / 5;
                    // let sample = sample as i64 + i32::MIN.abs() as i64;
                    // let sample = u32::from_ne_bytes(sample.to_ne_bytes());
                    // let sample = sample as u32;
                    let sample = sample as u32 * (u32::MAX / 16_777_215); // 0x10001;

                    *s = sample;
                }
            }
            Either::Second(MixerCMD::Play {
                drum_sound,
                volume,
                id,
            }) => {
                play_heads.push(PlayHead {
                    drum_sound,
                    volume,
                    id,
                    i: 0,
                });
            }
            Either::Second(MixerCMD::Stop { id }) => {
                play_heads.retain(|head| head.id != id);
            }
        }
    }
}
