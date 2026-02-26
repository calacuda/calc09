use alloc::{sync::Arc, vec::Vec};
use embassy_futures::select::{Either, select};
use embassy_rp::{
    Peri,
    peripherals::{PIN_26, PIN_27, PWM_SLICE5},
    pwm::{Config, Pwm, SetDutyCycle},
};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Receiver};
use embassy_time::Timer;

static SAMPLE_RATE: u64 = 48_000;

#[derive(Clone, Debug)]
pub enum MixerCMD {
    Play {
        /// audio file to play
        drum_sound: Arc<[f32]>,
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
    pub drum_sound: Arc<[f32]>,
    /// volume
    pub volume: f32,
    /// a unique-ish id to allow for stopping playback early
    pub id: u8,
    pub i: usize,
}

#[embassy_executor::task]
pub async fn mixer_task(
    left: Peri<'static, PIN_26>,
    right: Peri<'static, PIN_27>,
    slice5: Peri<'static, PWM_SLICE5>,
    cmds: Receiver<'static, CriticalSectionRawMutex, MixerCMD, 4>,
) {
    let mut c = Config::default();
    let desired_freq_hz = 25_000;
    let clock_freq_hz = embassy_rp::clocks::clk_sys_freq();
    let divider = 16u8;
    let period = (clock_freq_hz / (desired_freq_hz * divider as u32)) as u16 - 1;
    c.top = period;
    c.divider = divider.into();
    let mut pwm = Pwm::new_output_ab(slice5, left, right, c.clone());
    let mut play_heads: Vec<PlayHead> = Vec::new();

    loop {
        match select(Timer::after_nanos(1000000000 / SAMPLE_RATE), cmds.receive()).await {
            Either::First(_) => {
                let sample: f32 = {
                    let samples = play_heads.iter_mut().map(|head| {
                        let sample = head.drum_sound[head.i];
                        head.i += 1;
                        sample
                    });

                    samples.sum()
                };
                play_heads.retain(|head| head.i < head.drum_sound.len());
                // let sample = libm::tanhf(sample * 0.8);
                let sample = sample * 0.8;
                let perc = libm::roundf(sample * 100.) as u8;
                pwm.set_duty_cycle_percent(perc).unwrap();
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
