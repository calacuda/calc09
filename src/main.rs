#![no_std]
#![no_main]

#[allow(unused_imports)]
#[macro_use]
extern crate alloc;

use core::{char, ptr::addr_of_mut};

use alloc::{
    string::{String, ToString},
    vec::Vec,
};
use embassy_embedded_hal::{SetConfig, shared_bus::asynch::spi::SpiDevice};
use embassy_executor::{Executor, Spawner};
use embassy_futures::{join, yield_now};
use embassy_rp::{
    Peri, bind_interrupts,
    dma::InterruptHandler as DmaIrqHandler,
    gpio,
    i2c::InterruptHandler as I2cIrqHandler,
    multicore::{Stack, spawn_core1},
    peripherals::{
        DMA_CH0, DMA_CH1, I2C1, PIN_6, PIN_7, PIN_10, PIN_11, PIN_12, PIN_16, PIN_18, PIN_19, SPI0,
        SPI1, USB,
    },
    spi::{self, Async, Spi},
    usb::{Driver, InterruptHandler as UsbIrqHandler},
};
use embassy_sync::{
    blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex},
    channel::{Channel, Receiver, Sender},
    mutex::Mutex,
};
use embassy_time::{Delay, Timer};
use embassy_usb::{
    Builder, Config,
    class::cdc_acm::{CdcAcmClass, State},
};
use embassy_usb_logger::ReceiverHandler;
use embedded_alloc::LlffHeap as Heap;
use embedded_graphics::{
    mono_font::{MonoTextStyle, ascii::FONT_6X10},
    pixelcolor::Rgb565,
    prelude::*,
};
use embedded_hal_bus::spi::ExclusiveDevice;
use embedded_sdmmc::{SdCard, sdcard::DummyCsPin};
use gpio::{Level, Output};
use lcd_async::{
    models::ILI9488Rgb565,
    options::{ColorInversion, Orientation, Rotation},
    raw_framebuf::RawFrameBuf,
};
use log::*;
use static_cell::StaticCell;

use defmt_rtt as _;

pub mod key_codes;

pub static SCREEN_W: u16 = 320;
// pub static SCREEN_W: u16 = 80;
pub static SCREEN_H: u16 = 320;
// pub static SCREEN_H: u16 = 80;
pub const PIXEL_SIZE: f64 = 2.; // RGB565 = 2 bytes per pixel
// pub const PIXEL_SIZE: f64 = 2.25; // RGB666 = 2.25 bytes per pixel
pub const FRAME_SIZE: usize = ((SCREEN_W as f64) * (SCREEN_H as f64) * PIXEL_SIZE) as usize;

pub static FRAME_BUFFER: StaticCell<[u8; FRAME_SIZE]> = StaticCell::new();

// Program metadata for `picotool info`.
// This isn't needed, but it's recomended to have these minimal entries.
#[unsafe(link_section = ".bi_entries")]
#[used]
pub static PICOTOOL_ENTRIES: [embassy_rp::binary_info::EntryAddr; 4] = [
    embassy_rp::binary_info::rp_program_name!(c"calc09"),
    embassy_rp::binary_info::rp_program_description!(c""),
    embassy_rp::binary_info::rp_cargo_version!(),
    embassy_rp::binary_info::rp_program_build_attribute!(),
];

#[allow(static_mut_refs)]
fn init_heap() {
    use core::mem::MaybeUninit;
    static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
    unsafe { HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE) }
}

static mut CORE1_STACK: Stack<4096> = Stack::new();
// static mut CORE1_STACK: Stack<5120> = Stack::new();
// static EXECUTOR0: StaticCell<Executor> = StaticCell::new();
static EXECUTOR1: StaticCell<Executor> = StaticCell::new();

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => UsbIrqHandler<USB>;
    I2C1_IRQ => I2cIrqHandler<embassy_rp::peripherals::I2C1>;
    DMA_IRQ_0 => DmaIrqHandler<DMA_CH0>, DmaIrqHandler<DMA_CH1>;
});

#[global_allocator]
static HEAP: Heap = Heap::empty();
const HEAP_SIZE: usize = 128 * 1024;

static COMMAND_CHANNEL: Channel<CriticalSectionRawMutex, String, 4> = Channel::new();
static PC_KB_CHANNEL: Channel<CriticalSectionRawMutex, (PicoCalcKeyboardButton, bool), 4> =
    Channel::new();
static GUI_STATE: Channel<CriticalSectionRawMutex, GuiState, 4> = Channel::new();

// Keyboard I2C stuff
pub const _REG_VER: u8 = 0x01; //  fw version
pub const _REG_CFG: u8 = 0x02; //  config
pub const _REG_INT: u8 = 0x03; //  interrupt status
pub const _REG_KEY: u8 = 0x04; //  key status
pub const _REG_BKL: u8 = 0x05; //  backlight
pub const _REG_DEB: u8 = 0x06; //  debounce cfg
pub const _REG_FRQ: u8 = 0x07; //  poll freq cfg
pub const _REG_RST: u8 = 0x08; //  reset
pub const _REG_FIF: u8 = 0x09; //  fifo
pub const _REG_BK2: u8 = 0x0A; //  backlight 2
pub const _REG_BAT: u8 = 0x0B; //  battery
pub const _REG_DIR: u8 = 0x0C; //  gpio direction
pub const _REG_PUE: u8 = 0x0D; //  gpio input pull enable
pub const _REG_PUD: u8 = 0x0E; //  gpio input pull direction
pub const _REG_GIO: u8 = 0x0F; //  gpio value
pub const _REG_GIC: u8 = 0x10; //  gpio interrupt config
pub const _REG_GIN: u8 = 0x11; //  gpio interrupt status
pub const _KEY_COUNT_MASK: u8 = 0x1F;
pub const _WRITE_MASK: u8 = 1 << 7;
pub const _STATE_IDLE: u8 = 0;
pub const _STATE_PRESS: u8 = 1;
pub const _STATE_LONG_PRESS: u8 = 2;
pub const _STATE_RELEASE: u8 = 3;

#[derive(Clone, Copy, Debug)]
pub enum PicoCalcKeyboardButton {
    Text(char),
    ArrowUp,
    ArrowDown,
    ArrowLeft,
    ArrowRight,
    F1,
    F2,
    F3,
    F4,
    F5,
    F6,
    F7,
    F8,
    F9,
    F10,
    Esc,
    Brk,
    Tab,
    Home,
    CapsLK,
    Del,
    End,
    BackSpace,
    LShift,
    LCTRL,
    Alt,
    RShift,
    Enter,
    Ins,
    PgUp,
    PgDown,
}

#[derive(Clone, Debug)]
pub struct GuiState {}

impl GuiState {}

pub struct CmdHandler {}

impl ReceiverHandler for CmdHandler {
    fn new() -> Self {
        Self {}
    }

    async fn handle_data(&self, data: &[u8]) {
        match core::str::from_utf8(data) {
            Ok(cmd) => {
                info!("recv a command {cmd}");
                // let mut buf = [0u8; 256];
                // COMMAND_CHANNEL.send(cmd.to_string()).await;

                if cmd.starts_with("/greet ") {
                    let name = &cmd[7..cmd.len()];
                    info!("Hello, {name}!");
                } else if cmd.starts_with("/") {
                    error!("unknown command!");
                }
            }
            Err(e) => error!("messeage failed to parse with error: {e}. (likely invalid utf8)"),
        };
    }
}

struct DummyTimesource();

impl embedded_sdmmc::TimeSource for DummyTimesource {
    fn get_timestamp(&self) -> embedded_sdmmc::Timestamp {
        embedded_sdmmc::Timestamp {
            year_since_1970: 0,
            zero_indexed_month: 0,
            zero_indexed_day: 0,
            hours: 0,
            minutes: 0,
            seconds: 0,
        }
    }
}

#[embassy_executor::task]
async fn usb_task(driver: Driver<'static, USB>) {
    // Create embassy-usb Config
    let mut config = Config::new(0xc0de, 0xcafe);
    config.manufacturer = Some("Calacuda");
    config.product = Some("calc09");
    config.serial_number = Some("12345678");
    config.max_power = 100;
    config.max_packet_size_0 = 64;
    config.composite_with_iads = false;
    config.device_class = 0;
    config.device_sub_class = 0;
    config.device_protocol = 0;

    // Create embassy-usb DeviceBuilder using the driver and config.
    // It needs some buffers for building the descriptors.
    let mut config_descriptor = [0; 256];
    let mut bos_descriptor = [0; 256];
    let mut control_buf = [0; 64];

    let mut logger_state = State::new();

    let mut builder = Builder::new(
        driver,
        config,
        &mut config_descriptor,
        &mut bos_descriptor,
        &mut [], // no msos descriptors
        &mut control_buf,
    );

    // Create a class for the logger
    let logger_class = CdcAcmClass::new(&mut builder, &mut logger_state, 64);

    #[allow(static_mut_refs)]
    let log_fut = unsafe {
        static mut LOGGER: ::embassy_usb_logger::UsbLogger<1024, CmdHandler> =
            ::embassy_usb_logger::UsbLogger::with_custom_style(|record, writer| {
                use core::fmt::Write;
                let level = record.level().as_str();

                if record
                    .target()
                    .starts_with(&env!("CARGO_PKG_NAME").replace("-", "_"))
                {
                    write!(writer, "[{level}] {}\r\n", record.args(),).unwrap();
                }
            });
        LOGGER.with_handler(CmdHandler {});
        let _ = ::log::set_logger_racy(&LOGGER)
            .map(|()| log::set_max_level_racy(log::LevelFilter::Debug));

        LOGGER.create_future_from_class(logger_class)
    };

    // Build the builder.
    let mut usb = builder.build();

    // Run the USB device.
    let usb_fut = usb.run();

    join::join(log_fut, usb_fut).await;
}

#[embassy_executor::task]
async fn blinky(mut led: Output<'static>) {
    loop {
        led.set_high();
        #[cfg(not(debug_assertions))]
        trace!("on");
        #[cfg(debug_assertions)]
        debug!("on");
        Timer::after_millis(500).await;

        led.set_low();
        #[cfg(not(debug_assertions))]
        trace!("off");
        #[cfg(debug_assertions)]
        debug!("off");
        Timer::after_millis(500).await;
    }
}

#[embassy_executor::task]
async fn kbd_reader(
    i2c: Peri<'static, I2C1>,
    sda: Peri<'static, PIN_6>,
    scl: Peri<'static, PIN_7>,
    kbd_output: Sender<'static, CriticalSectionRawMutex, (PicoCalcKeyboardButton, bool), 4>,
) {
    // SETUP KEEB
    // let i2c_freq = 200_000;
    let i2c_freq = 10_000;
    let mut config = embassy_rp::i2c::Config::default();
    config.frequency = i2c_freq;
    let mut kbd = embassy_rp::i2c::I2c::new_async(i2c, scl, sda, Irqs, config);
    let kbd_addr: u16 = 0x1f;

    loop {
        // get_n_keys
        let mut n_keys: [u8; 2] = [0, 0];

        if let Err(_e) = kbd
            .write_read_async(kbd_addr, [_REG_KEY], &mut n_keys)
            .await
        {
            // error!("failed to get the number of keys that need reading: {e:?}");
            // Timer::after(Duration::from_micros(250)).await;
            continue;
        }

        let n_keys = n_keys[0] & _KEY_COUNT_MASK;

        for _i in 0..n_keys {
            // get key
            let mut key_got: [u8; 2] = [0, 0];

            if let Err(e) = kbd
                .write_read_async(kbd_addr, [_REG_FIF], &mut key_got)
                .await
            {
                error!("failed to get the number of keys that need reading: {e:?}");
                continue;
            }

            // send keys
            let state = key_got[0];
            let key = key_got[1];

            debug!("got key press: 0x{key:0x}");

            let char = char::from(key);

            if state == _STATE_LONG_PRESS {
                continue;
            }

            if char.is_ascii_graphic() {
                // debug!("got craphical key: 0x{key:0x}");

                if state == _STATE_PRESS {
                    kbd_output
                        .send((PicoCalcKeyboardButton::Text(char), true))
                        .await;
                } else if state == _STATE_RELEASE {
                    kbd_output
                        .send((PicoCalcKeyboardButton::Text(char), false))
                        .await;
                } else {
                    // key_presses.press_key(key);
                }
            } else {
                let pressed = if state == _STATE_PRESS {
                    true
                } else if state == _STATE_RELEASE {
                    false
                } else {
                    continue;
                };

                match key {
                    key_codes::KEY_BACKSPACE => {
                        kbd_output.send((PicoCalcKeyboardButton::BackSpace, pressed))
                    }
                    key_codes::KEY_TAB => kbd_output.send((PicoCalcKeyboardButton::Tab, pressed)),
                    key_codes::KEY_ENTER => {
                        kbd_output.send((PicoCalcKeyboardButton::Enter, pressed))
                    }
                    key_codes::KEY_BTN_LEFT2 => {
                        kbd_output.send((PicoCalcKeyboardButton::ArrowLeft, pressed))
                    }
                    key_codes::KEY_BTN_RIGHT2 => {
                        kbd_output.send((PicoCalcKeyboardButton::ArrowRight, pressed))
                    }
                    key_codes::KEY_MOD_ALT => {
                        kbd_output.send((PicoCalcKeyboardButton::Alt, pressed))
                    }
                    key_codes::KEY_MOD_SHL => {
                        kbd_output.send((PicoCalcKeyboardButton::LShift, pressed))
                    }
                    key_codes::KEY_MOD_SHR => {
                        kbd_output.send((PicoCalcKeyboardButton::RShift, pressed))
                    }
                    key_codes::KEY_MOD_SYM => {
                        continue;
                    }
                    key_codes::KEY_MOD_CTRL => {
                        kbd_output.send((PicoCalcKeyboardButton::LCTRL, pressed))
                    }
                    key_codes::KEY_ESC => kbd_output.send((PicoCalcKeyboardButton::Esc, pressed)),
                    key_codes::KEY_UP => {
                        kbd_output.send((PicoCalcKeyboardButton::ArrowUp, pressed))
                    }
                    key_codes::KEY_DOWN => {
                        kbd_output.send((PicoCalcKeyboardButton::ArrowDown, pressed))
                    }
                    key_codes::KEY_LEFT => {
                        kbd_output.send((PicoCalcKeyboardButton::ArrowLeft, pressed))
                    }
                    key_codes::KEY_RIGHT => {
                        kbd_output.send((PicoCalcKeyboardButton::ArrowRight, pressed))
                    }
                    key_codes::KEY_BREAK => kbd_output.send((PicoCalcKeyboardButton::Brk, pressed)),
                    key_codes::KEY_INSERT => {
                        kbd_output.send((PicoCalcKeyboardButton::Ins, pressed))
                    }
                    key_codes::KEY_HOME => kbd_output.send((PicoCalcKeyboardButton::Home, pressed)),
                    key_codes::KEY_DEL => kbd_output.send((PicoCalcKeyboardButton::Del, pressed)),
                    key_codes::KEY_END => kbd_output.send((PicoCalcKeyboardButton::End, pressed)),
                    key_codes::KEY_PAGE_UP => {
                        kbd_output.send((PicoCalcKeyboardButton::PgUp, pressed))
                    }
                    key_codes::KEY_PAGE_DOWN => {
                        kbd_output.send((PicoCalcKeyboardButton::PgDown, pressed))
                    }
                    key_codes::KEY_CAPS_LOCK => {
                        kbd_output.send((PicoCalcKeyboardButton::CapsLK, pressed))
                    }
                    key_codes::KEY_F1 => kbd_output.send((PicoCalcKeyboardButton::F1, pressed)),
                    key_codes::KEY_F2 => kbd_output.send((PicoCalcKeyboardButton::F2, pressed)),
                    key_codes::KEY_F3 => kbd_output.send((PicoCalcKeyboardButton::F3, pressed)),
                    key_codes::KEY_F4 => kbd_output.send((PicoCalcKeyboardButton::F4, pressed)),
                    key_codes::KEY_F5 => kbd_output.send((PicoCalcKeyboardButton::F5, pressed)),
                    key_codes::KEY_F6 => kbd_output.send((PicoCalcKeyboardButton::F6, pressed)),
                    key_codes::KEY_F7 => kbd_output.send((PicoCalcKeyboardButton::F7, pressed)),
                    key_codes::KEY_F8 => kbd_output.send((PicoCalcKeyboardButton::F8, pressed)),
                    key_codes::KEY_F9 => kbd_output.send((PicoCalcKeyboardButton::F9, pressed)),
                    key_codes::KEY_F10 => kbd_output.send((PicoCalcKeyboardButton::F10, pressed)),
                    _ => {
                        warn!("Unknown keypress: {key:0x}");
                        continue;
                    }
                }
                .await
            }
        }
    }
}

#[embassy_executor::task]
async fn gui(
    dc: Output<'static>,
    cs: Output<'static>,
    rst: Output<'static>,
    clk: Peri<'static, PIN_10>,
    mosi: Peri<'static, PIN_11>,
    miso: Peri<'static, PIN_12>,
    spi: Peri<'static, SPI1>,
    dma_0: Peri<'static, DMA_CH0>,
    dma_1: Peri<'static, DMA_CH1>,
    gui_input: Receiver<'static, CriticalSectionRawMutex, GuiState, 4>,
) {
    // SETUP SCREEN
    let mut spi_conf = embassy_rp::spi::Config::default();
    spi_conf.frequency = 40_000_000u32;
    let spi = Spi::new(spi, clk, mosi, miso, dma_0, dma_1, Irqs, spi_conf);
    static SPI_BUS: StaticCell<Mutex<NoopRawMutex, Spi<'static, SPI1, Async>>> = StaticCell::new();
    let spi_bus = Mutex::new(spi);
    let spi_bus = SPI_BUS.init(spi_bus);
    let spi_device = SpiDevice::new(spi_bus, cs);

    // Create display interface
    let di = lcd_async::interface::SpiInterface::new(spi_device, dc);
    let mut delay = Delay;
    // Initialize the display
    let mut display = lcd_async::Builder::new(ILI9488Rgb565, di)
        .reset_pin(rst)
        .display_size(SCREEN_W, SCREEN_H)
        .orientation(Orientation {
            rotation: Rotation::Deg0,
            mirrored: true,
        })
        .display_offset(0, 0)
        .invert_colors(ColorInversion::Inverted)
        .init(&mut delay)
        .await
        .unwrap();
    info!("display initted successfully...");

    if let Err(e) = display
        .set_tearing_effect(lcd_async::options::TearingEffect::Vertical)
        .await
    {
        error!("setting screen tearing failed with error: {e:?}");
    } else {
        info!("set screen tearing on");
    }

    // Initialize frame buffer
    let Some(frame_buffer) = FRAME_BUFFER.try_init_with(|| [0; FRAME_SIZE]) else {
        error!("frame buffer failed to init");
        return;
    };

    info!("success, frame_buffer size: {}", frame_buffer.len());

    info!("clearing display");

    // Send the framebuffer data to the display
    if let Err(e) = display
        .show_raw_data(0, 0, SCREEN_W, SCREEN_H, frame_buffer)
        .await
    {
        error!("showing frame_buffer failed with error: {e:?}");
        // error!("showing raw_data {e:?}");
    }

    info!("display cleared");

    let top = SCREEN_H / 3;
    let bottom = SCREEN_H - top;
    // let style = MonoTextStyle::new(&FONT_6X10, Rgb565::new(0, 63, 31));
    let mut style = MonoTextStyle::new(&FONT_6X10, Rgb565::GREEN);
    let n_rows = (top - 8) / 10;
    let n_cols = 3;
    let mut gui_state = GuiState {};

    loop {
        // Create a framebuffer for drawing
        let mut raw_fb = RawFrameBuf::<Rgb565, _>::new(
            frame_buffer.as_mut_slice(),
            SCREEN_W.into(),
            SCREEN_H.into(),
        );

        // Clear the framebuffer to black
        if let Err(e) = raw_fb.clear(Rgb565::BLACK) {
            error!("failed to clear screen: {e}");
        }

        // Send the framebuffer data to the display
        if let Err(e) = display
            .show_raw_data(0, 0, SCREEN_W, SCREEN_H, frame_buffer)
            .await
        {
            error!("showing raw_data {e:?}");
        }

        debug!("awaiting GuiState input");
        gui_state = gui_input.receive().await;
        debug!("got new GuiState!");
    }
}

#[embassy_executor::task]
async fn sd_card_reader(
    cs: Output<'static>,
    clk: Peri<'static, PIN_18>,
    mosi: Peri<'static, PIN_19>,
    miso: Peri<'static, PIN_16>,
    spi: Peri<'static, SPI0>,
) {
    // SPI clock needs to be running at <= 400kHz during initialization
    let mut config = spi::Config::default();
    config.frequency = 400_000;

    let spi = Spi::new_blocking(spi, clk, mosi, miso, config);
    // Use a dummy cs pin here, for embedded-hal SpiDevice compatibility reasons
    let spi_dev = match ExclusiveDevice::new_no_delay(spi, DummyCsPin) {
        Ok(dev) => dev,
        Err(e) => {
            error!("{e}");
            return;
        }
    };

    let sdcard = SdCard::new(spi_dev, cs, embassy_time::Delay);
    info!("Card size is {} bytes", sdcard.num_bytes().unwrap());
    // Timer::after_millis(10).await;

    // Now that the card is initialized, the SPI clock can go faster
    let mut config = spi::Config::default();
    config.frequency = 16_000_000;
    sdcard
        .spi(|dev| SetConfig::set_config(dev.bus_mut(), &config))
        .ok();
    debug!("made sd card interface faster");
    // Timer::after_millis(10).await;

    // Now let's look for volumes (also known as partitions) on our block device.
    // To do this we need a Volume Manager. It will take ownership of the block device.
    let mut volume_mgr = embedded_sdmmc::VolumeManager::new(sdcard, DummyTimesource());
    debug!("made volume manager");
    // Timer::after_millis(10).await;

    // Try and access Volume 0 (i.e. the first partition).
    // The volume object holds information about the filesystem on that volume.
    let mut volume0 = volume_mgr
        .open_volume(embedded_sdmmc::VolumeIdx(0))
        .unwrap();
    info!("Volume 0: {:?}", defmt::Debug2Format(&volume0));
    // Timer::after_millis(10).await;

    // Open the root directory (mutably borrows from the volume).
    let mut root_dir = volume0.open_root_dir().unwrap();

    // // Open a file called "MY_FILE.TXT" in the root directory
    // // This mutably borrows the directory.
    // let mut my_file = root_dir
    //     .open_file_in_dir("MY_FILE.TXT", embedded_sdmmc::Mode::ReadOnly)
    //     .unwrap();

    let mut ls = Vec::new();

    if let Err(e) = root_dir.iterate_dir(|dir| ls.push(dir.name.to_string())) {
        error!("directory listing resulted in: {e:?}");
    }

    debug!("root_dir ls: {ls:?}");
    // Timer::after_millis(10).await;

    loop {
        yield_now().await
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    init_heap();

    let p = embassy_rp::init(Default::default());

    spawn_core1(
        p.CORE1,
        unsafe { &mut *addr_of_mut!(CORE1_STACK) },
        move || {
            let executor1 = EXECUTOR1.init(Executor::new());
            executor1.run(|spawner| {
                // task for serial logging & other usb stuff
                let driver = Driver::new(p.USB, Irqs);

                // run USB HID and logging in its own thread
                spawner.spawn(usb_task(driver).expect("failed to crate usb task"));
            })
        },
    );

    // LED section
    // let led = Output::new(p.PIN_25, Level::Low);
    let led = Output::new(p.PIN_5, Level::Low);
    // let led = Output::new(p.PIN_21, Level::Low);
    spawner.spawn(blinky(led).expect("failed to create blinky task"));

    // sd card reader thread
    let cs = Output::new(p.PIN_17, Level::High);
    let clk = p.PIN_18;
    let mosi = p.PIN_19;
    let miso = p.PIN_16;
    spawner.spawn(
        sd_card_reader(
            // dc,
            cs, // rst,
            clk, mosi, miso, p.SPI0,
        )
        .expect("failed to spawn sd card reader task"),
    );

    // display driver thread
    let dc = Output::new(p.PIN_14, Level::Low);
    let cs = Output::new(p.PIN_13, Level::Low);
    let rst = Output::new(p.PIN_15, Level::Low);
    let clk = p.PIN_10;
    let mosi = p.PIN_11;
    let miso = p.PIN_12;
    spawner.spawn(
        gui(
            dc,
            cs,
            rst,
            clk,
            mosi,
            miso,
            p.SPI1,
            p.DMA_CH0,
            p.DMA_CH1,
            GUI_STATE.receiver(),
        )
        .expect("failed to spawn gui thread"),
    );

    // picocalc keyboard input thread
    let sda = p.PIN_6;
    let scl = p.PIN_7;
    let i2c = p.I2C1;
    spawner.spawn(
        kbd_reader(i2c, sda, scl, PC_KB_CHANNEL.sender()).expect("failed to create kbd reader"),
    );

    info!("Hello, world!");

    let mut gui_state = GuiState {};

    loop {
        let (key, was_pressed) = PC_KB_CHANNEL.receive().await;
        if !was_pressed {
            match key {
                PicoCalcKeyboardButton::Enter => {}
                PicoCalcKeyboardButton::ArrowUp => {}
                PicoCalcKeyboardButton::ArrowDown => {}
                PicoCalcKeyboardButton::ArrowLeft => {}
                PicoCalcKeyboardButton::ArrowRight => {}
                PicoCalcKeyboardButton::BackSpace => {}
                PicoCalcKeyboardButton::Del => {}
                PicoCalcKeyboardButton::LShift => {}
                PicoCalcKeyboardButton::RShift => {}
                PicoCalcKeyboardButton::CapsLK => {}
                PicoCalcKeyboardButton::Text(char) => {}
                PicoCalcKeyboardButton::F1 => {}
                PicoCalcKeyboardButton::F2 => {}
                PicoCalcKeyboardButton::F3 => {}
                PicoCalcKeyboardButton::F4 => {}
                PicoCalcKeyboardButton::F5 => {}
                PicoCalcKeyboardButton::F6 => {}
                PicoCalcKeyboardButton::F7 => {}
                PicoCalcKeyboardButton::F8 => {}
                PicoCalcKeyboardButton::F9 => {}
                PicoCalcKeyboardButton::F10 => {}
                PicoCalcKeyboardButton::Esc => {}
                PicoCalcKeyboardButton::Brk => {}
                PicoCalcKeyboardButton::Tab => {}
                PicoCalcKeyboardButton::Home => {}
                PicoCalcKeyboardButton::End => {}
                PicoCalcKeyboardButton::LCTRL => {}
                PicoCalcKeyboardButton::Alt => {}
                PicoCalcKeyboardButton::Ins => {}
                PicoCalcKeyboardButton::PgUp => {}
                PicoCalcKeyboardButton::PgDown => {}
            }
        } else {
            match key {
                PicoCalcKeyboardButton::Enter => {}
                PicoCalcKeyboardButton::ArrowUp => {}
                PicoCalcKeyboardButton::ArrowDown => {}
                PicoCalcKeyboardButton::ArrowLeft => {}
                PicoCalcKeyboardButton::ArrowRight => {}
                PicoCalcKeyboardButton::BackSpace => {}
                PicoCalcKeyboardButton::Del => {}
                PicoCalcKeyboardButton::LShift => {}
                PicoCalcKeyboardButton::RShift => {}
                PicoCalcKeyboardButton::CapsLK => {}
                PicoCalcKeyboardButton::Text(char) => {}
                PicoCalcKeyboardButton::F1 => {}
                PicoCalcKeyboardButton::F2 => {}
                PicoCalcKeyboardButton::F3 => {}
                PicoCalcKeyboardButton::F4 => {}
                PicoCalcKeyboardButton::F5 => {}
                PicoCalcKeyboardButton::F6 => {}
                PicoCalcKeyboardButton::F7 => {}
                PicoCalcKeyboardButton::F8 => {}
                PicoCalcKeyboardButton::F9 => {}
                PicoCalcKeyboardButton::F10 => {}
                PicoCalcKeyboardButton::Esc => {}
                PicoCalcKeyboardButton::Brk => {}
                PicoCalcKeyboardButton::Tab => {}
                PicoCalcKeyboardButton::Home => {}
                PicoCalcKeyboardButton::End => {}
                PicoCalcKeyboardButton::LCTRL => {}
                PicoCalcKeyboardButton::Alt => {}
                PicoCalcKeyboardButton::Ins => {}
                PicoCalcKeyboardButton::PgUp => {}
                PicoCalcKeyboardButton::PgDown => {}
            }
        }
    }
}

// same panicking *behavior* as `panic-probe` but doesn't print a panic message
// this prevents the panic message being printed *twice* when `defmt::panic` is invoked
#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    let p = embassy_rp::init(Default::default());
    let mut led = Output::new(p.PIN_21, Level::Low);
    led.set_high();

    loop {
        cortex_m::asm::nop()
    }
    // cortex_m::asm::udf()
}
