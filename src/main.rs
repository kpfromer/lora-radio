#![no_main]
#![no_std]

pub mod display;
pub mod error;
pub mod led;
pub mod panic;
pub mod soil;
pub mod usb_serial;

pub mod prelude {
    pub use crate::error::*;
}

use core::fmt::Write;

use hal::Spim;
use hal::Timer;

use hal::gpio::p0::P0_02;
use hal::gpio::p0::P0_28;
use hal::gpio::Output;
use hal::gpio::PushPull;
use hal::pac::SPIM1;
use hal::pac::TIMER4;

use embedded_hal::blocking::spi::{Transfer, Write as SpiWrite};
use hal::prelude::OutputPin;
use heapless::{String, Vec};

use nrf52840_hal as _;

use profont::PROFONT_10_POINT;
use rtic::app;

use dwt_systick_monotonic::DwtSystick;
use dwt_systick_monotonic::ExtU32;

use nrf52840_hal as hal;
use nrf52840_hal::clocks::HFCLK_FREQ;
use nrf52840_hal::clocks::{ExternalOscillator, Internal, LfOscStopped};
use nrf52840_hal::gpio::Level;
use nrf52840_hal::gpiote::Gpiote;
use nrf52840_hal::Clocks;

use display_interface_spi::SPIInterfaceNoCS;
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::{
    mono_font::MonoTextStyleBuilder, prelude::*, primitives::Rectangle, text::Text,
};
use embedded_layout::layout::linear::LinearLayout;
use embedded_layout::prelude::*;

use st7789::ST7789;

use nrf52840_hal::usbd::{UsbPeripheral, Usbd};
use sx127x_lora::LoRa;
use usb_device::bus::UsbBusAllocator;

use profont::PROFONT_24_POINT;

use core::fmt::Debug;
use display::DisplayDevice;
use led::Led;
use usb_serial::UsbSerialDevice;

const FREQUENCY: i64 = 915;

type LoraRadio = LoRa<Spim<SPIM1>, P0_28<Output<PushPull>>, P0_02<Output<PushPull>>, Timer<TIMER4>>;

fn show_error(error: impl Debug, display_device: &mut DisplayDevice<nrf52840_hal::pac::TIMER1>) {
    let display_area = Rectangle::new(Point::new(80, 0), Size::new(240, 240));

    let error_type = Text::new(
        "Error",
        Point::zero(),
        MonoTextStyleBuilder::new()
            .font(&PROFONT_10_POINT)
            .text_color(Rgb565::RED)
            .background_color(Rgb565::BLACK)
            .build(),
    );
    let mut error_message_string: String<64> = String::new();
    write!(error_message_string, "{:?}", error).unwrap();
    let error_message = Text::new(
        error_message_string.as_str(),
        Point::zero(),
        MonoTextStyleBuilder::new()
            .font(&PROFONT_10_POINT)
            .text_color(Rgb565::RED)
            .background_color(Rgb565::BLACK)
            .build(),
    );
    LinearLayout::vertical(Chain::new(error_type).append(error_message))
        .with_alignment(horizontal::Center)
        .arrange()
        .align_to(&display_area, horizontal::Center, vertical::Center)
        .draw(&mut display_device.display)
        .unwrap();
}

#[derive(Debug)]
enum LoraError<RadioError> {
    LoraRadioError(RadioError),
    LoraOverflow,
}

use LoraError::*;

#[app(device = nrf52840_hal::pac, peripherals = true, dispatchers = [PWM3, SPIM3, SPIM2_SPIS2_SPI2])]
mod app {

    use profont::PROFONT_10_POINT;

    use super::*;

    #[shared]
    struct Shared {
        display_device: DisplayDevice<nrf52840_hal::pac::TIMER1>,
    }

    #[local]
    struct Local {
        white_led: Led,
        // usb_serial_device: UsbSerialDevice<'static, Usbd<UsbPeripheral<'static>>>,
        lora: LoraRadio,
        // gpiote: Gpiote,
    }

    // 64_000_000 matches hal::clocks::HFCLK_FREQ
    #[monotonic(binds = SysTick, default = true)]
    type MonoTimer = DwtSystick<HFCLK_FREQ>;

    struct DelayWrapper<'a>(&'a mut MonoTimer);

    impl embedded_hal::blocking::delay::DelayUs<u32> for DelayWrapper<'_> {
        fn delay_us(&mut self, us: u32) {
            let wait_until = self.0.now() + (us as u32).micros();
            while self.0.now() < wait_until {
                /* spin */
                cortex_m::asm::nop();
            }
        }
    }

    #[init(local=[clocks: Option<Clocks<ExternalOscillator, Internal, LfOscStopped>> = None, usb_bus: Option<UsbBusAllocator<Usbd<UsbPeripheral<'static>>>> = None])]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        let mut debug_control_block = cx.core.DCB;
        let clocks = cx.local.clocks;
        clocks.replace(hal::Clocks::new(cx.device.CLOCK).enable_ext_hfosc());
        let monotonic = DwtSystick::new(
            &mut debug_control_block,
            cx.core.DWT,
            cx.core.SYST,
            hal::clocks::HFCLK_FREQ,
        );

        let port0 = hal::gpio::p0::Parts::new(cx.device.P0);
        let port1 = hal::gpio::p1::Parts::new(cx.device.P1);

        let mut red_led = Led::new(port1.p1_01.degrade());
        red_led.off();

        let mut white_led = Led::new(port0.p0_10.degrade());

        let tft_reset = port1.p1_03.into_push_pull_output(Level::Low);
        let tft_backlight = port1.p1_05.into_push_pull_output(Level::Low);
        let _tft_cs = port0.p0_12.into_push_pull_output(Level::Low);
        let tft_dc = port0.p0_13.into_push_pull_output(Level::Low);
        let tft_sck = port0.p0_14.into_push_pull_output(Level::Low).degrade();
        let tft_mosi = port0.p0_15.into_push_pull_output(Level::Low).degrade();
        let pins = hal::spim::Pins {
            sck: Some(tft_sck),
            miso: None,
            mosi: Some(tft_mosi),
        };
        // https://github.com/almindor/st7789-examples/blob/master/examples/image.rs
        let spi = Spim::new(
            cx.device.SPIM0,
            pins,
            hal::spim::Frequency::M8,
            hal::spim::MODE_3,
            122,
        );
        // Display interface from SPI and DC
        let display_interface = SPIInterfaceNoCS::new(spi, tft_dc);
        // Create driver
        let mut display = ST7789::new(
            display_interface,
            Some(tft_reset),
            Some(tft_backlight),
            240,
            240,
        );

        // initialize
        let mut timer = Timer::new(cx.device.TIMER4);
        display.init(&mut timer).unwrap();
        // set default orientation
        display
            .set_orientation(st7789::Orientation::LandscapeSwapped)
            // .set_orientation(st7789::Orientation::Landscape)
            .unwrap();
        display.clear(Rgb565::BLACK).unwrap();
        let mut display_device = DisplayDevice::new(display, Timer::new(cx.device.TIMER1));
        // The layout
        let display_area = Rectangle::new(Point::new(80, 0), Size::new(240, 240));
        // LinearLayout::vertical(Chain::new(text).append(lt))
        //     .with_alignment(horizontal::Center)
        //     .arrange()
        //     .align_to(&display_area, horizontal::Center, vertical::Center)
        //     .draw(&mut display_device.display)
        //     .unwrap();
        // DONE

        // let usb_bus = cx.local.usb_bus;
        // usb_bus.replace(UsbBusAllocator::new(Usbd::new(UsbPeripheral::new(
        //     cx.device.USBD,
        //     clocks.as_ref().unwrap(),
        // ))));

        // let usb_serial_device = UsbSerialDevice::new(usb_bus.as_ref().unwrap());

        // GPIO interrupts
        let btn = port1.p1_02.into_pullup_input().degrade();
        let gpiote = Gpiote::new(cx.device.GPIOTE);
        gpiote.port().input_pin(&btn).low();
        gpiote.port().enable_interrupt();

        let mut lora = {
            // let sck = port0.p0_18.into_push_pull_output(Level::Low).degrade();
            // let miso = port0.p0_06.into_floating_input().degrade();
            // let mosi = port0.p0_26.into_push_pull_output(Level::Low).degrade();

            let sck = port0.p0_04.into_push_pull_output(Level::Low).degrade();
            let miso = port0.p0_05.into_floating_input().degrade();
            let mosi = port0.p0_03.into_push_pull_output(Level::Low).degrade();

            let cs = port0.p0_28.into_push_pull_output(Level::Low);
            let reset = port0.p0_02.into_push_pull_output(Level::Low);

            let pins = hal::spim::Pins {
                sck: Some(sck),
                miso: Some(miso),
                mosi: Some(mosi),
            };

            let spi = Spim::new(
                cx.device.SPIM1,
                pins,
                hal::spim::Frequency::M4,
                hal::spim::MODE_0,
                122,
            );

            match sx127x_lora::LoRa::new(spi, cs, reset, FREQUENCY, timer) {
                Ok(l) => l,
                Err(error) => {
                    show_error(error, &mut display_device);
                    panic!();
                }
            }
        };

        // let mut v: String<128> = String::new();

        // let poll = lora.poll_irq(Some(40));
        // if let Ok(size) = poll {
        //     let buffer = lora.read_packet().unwrap();
        //     for i in 0..size {
        //         v.push(buffer[i] as char).unwrap();
        //     }
        // }

        // TODO: move
        let text = Text::new(
            "BOOTED",
            Point::zero(),
            MonoTextStyleBuilder::new()
                .font(&PROFONT_24_POINT)
                .text_color(Rgb565::GREEN)
                .background_color(Rgb565::BLACK)
                .build(),
        );
        LinearLayout::vertical(Chain::new(text))
            .with_alignment(horizontal::Center)
            .arrange()
            .align_to(&display_area, horizontal::Center, vertical::Center)
            .draw(&mut display_device.display)
            .unwrap();

        (
            Shared { display_device },
            Local {
                white_led,
                // usb_serial_device,
                lora,
                // gpiote,
            },
            init::Monotonics(monotonic),
        )
    }

    #[idle(local = [lora], shared = [display_device])]
    fn idle(mut cx: idle::Context) -> ! {
        let lora = cx.local.lora;

        loop {
            match {
                // lora.poll_irq(Some(30))
                lora.poll_irq(Some(30 * 1000))
                    .map_err(LoraRadioError)
                    .and_then(|size| {
                        lora.read_packet()
                            .map(|buffer| (size, buffer))
                            .map_err(LoraRadioError)
                    })
                    .map(|(size, buffer)| {
                        let mut message_string: String<255> = String::new();
                        for i in 0..size {
                            message_string.push(buffer[i] as char).unwrap();
                        }
                        message_string
                    }) //30 Second timeout
                       // Received buffer. NOTE: 255 bytes are always returned
            } {
                Ok(message_string) => {
                    let display_area = Rectangle::new(Point::new(80, 0), Size::new(240, 240));
                    let text = Text::new(
                        message_string.as_str(),
                        Point::zero(),
                        MonoTextStyleBuilder::new()
                            .font(&PROFONT_24_POINT)
                            .text_color(Rgb565::GREEN)
                            .background_color(Rgb565::BLACK)
                            .build(),
                    );

                    cx.shared.display_device.lock(|display_device| {
                        LinearLayout::vertical(Chain::new(text))
                            .with_alignment(horizontal::Center)
                            .arrange()
                            .align_to(&display_area, horizontal::Center, vertical::Center)
                            .draw(&mut display_device.display)
                            .unwrap()
                    })
                }
                Err(error) => {
                    cx.shared
                        .display_device
                        .lock(|display_device| show_error(error, display_device));
                    panic!();
                }
            }
        }
    }

    // #[task(binds = GPIOTE, local = [gpiote], priority = 5)]
    // fn on_gpiote(cx: on_gpiote::Context) {
    //     cx.local.gpiote.reset_events();
    //     toggle_display::spawn().unwrap();
    // }

    // #[task(shared=[display_device], priority = 4)]
    // fn toggle_display(mut cx: toggle_display::Context) {
    //     cx.shared
    //         .display_device
    //         .lock(|display_device| display_device.toggle());
    // }

    // #[task(local=[soil_string: String<10> = String::new()], shared = [display_device], priority = 4)]
    // fn render(mut cx: render::Context, soil_moisture: u8) {
    //     let mut skip = false;
    //     cx.shared.display_device.lock(|display_device| {
    //         if !display_device.is_on {
    //             skip = true
    //         }
    //     });
    //     if skip {
    //         return;
    //     }

    //     let is_motor_on = soil_moisture < MOISTURE_THRESHOLD;

    //     let soil_string = cx.local.soil_string;
    //     write!(soil_string, " Soil {}% ", soil_moisture).unwrap();
    //     let motor_str = if is_motor_on {
    //         " Motor On "
    //     } else {
    //         " Motor Off "
    //     };

    //     let display_area = Rectangle::new(Point::new(80, 0), Size::new(240, 240));
    //     let title_text = Text::new(
    //         "PWAT 5000",
    //         Point::zero(),
    //         MonoTextStyleBuilder::new()
    //             .font(&PROFONT_24_POINT)
    //             .text_color(Rgb565::WHITE)
    //             .background_color(Rgb565::BLACK)
    //             .build(),
    //     );
    //     let soil_text = Text::new(
    //         soil_string.as_str(),
    //         Point::zero(),
    //         MonoTextStyleBuilder::new()
    //             .font(&PROFONT_24_POINT)
    //             .text_color(if is_motor_on {
    //                 Rgb565::RED
    //             } else {
    //                 Rgb565::GREEN
    //             })
    //             .background_color(Rgb565::BLACK)
    //             .build(),
    //     );
    //     let motor_text = Text::new(
    //         motor_str,
    //         Point::zero(),
    //         MonoTextStyleBuilder::new()
    //             .font(&PROFONT_24_POINT)
    //             .text_color(if is_motor_on {
    //                 Rgb565::GREEN
    //             } else {
    //                 Rgb565::RED
    //             })
    //             .background_color(Rgb565::BLACK)
    //             .build(),
    //     );

    //     cx.shared.display_device.lock(|display_device| {
    //         if display_device.is_on {
    //             // The layout
    //             LinearLayout::vertical(Chain::new(title_text).append(soil_text).append(motor_text))
    //                 .with_alignment(horizontal::Center)
    //                 .arrange()
    //                 .align_to(&display_area, horizontal::Center, vertical::Center)
    //                 .draw(&mut display_device.display)
    //                 .unwrap();
    //         }
    //     });

    //     // let bmp_data = include_bytes!("../images/rust-social.bmp");
    //     // // Parse the BMP file.
    //     // let bmp = Bmp::from_slice(bmp_data).unwrap();
    //     // // Draw the image with the top left corner at (10, 20) by wrapping it in
    //     // // an embedded-graphics `Image`.
    //     // Image::new(&bmp, Point::new(100, 50)).draw(display).unwrap();

    //     soil_string.clear();
    // }

    // #[task(local = [soil])]
    // fn soil_measure(cx: soil_measure::Context) {
    //     let soil = cx.local.soil;

    //     let soil_moisture = soil.soil_moisture_percentage().unwrap();

    //     // render dipslay
    //     render::spawn(soil_moisture).unwrap();
    //     // control motor
    //     motor::spawn(soil_moisture < MOISTURE_THRESHOLD).unwrap();

    //     // spawn again in a bit
    //     soil_measure::spawn_after(1.secs()).unwrap();
    // }

    // #[idle(local = [usb_serial_device, white_led])]
    // fn idle(cx: idle::Context) -> ! {
    //     let usb_serial_device = cx.local.usb_serial_device;
    //     let mut chars: Vec<u8, 64> = Vec::new();
    //     let prompt = b"Enter command: ";

    //     let clear_screen = |usb_serial_device: &mut UsbSerialDevice<Usbd<UsbPeripheral>>| {
    //         let mut chars: Vec<u8, 8> = Vec::new();
    //         write!(chars, "{}[2J", 27 as char).unwrap();
    //         usb_serial_device.write_chars(&chars).unwrap();
    //     };

    //     loop {
    //         usb_serial_device.write_chars(&prompt[..]).unwrap();

    //         // Read line and clear chars if there's an error
    //         if usb_serial_device.read_line(&mut chars).is_err() {
    //             chars.clear();
    //             continue;
    //         }

    //         match &chars[..] {
    //             b"on" => cx.local.white_led.on(),
    //             b"off" => cx.local.white_led.off(),
    //             b"motor" => motor::spawn(true).unwrap(),
    //             b"display" => toggle_display::spawn().unwrap(),
    //             _ => {}
    //         }

    //         // Clear input
    //         chars.clear();
    //         clear_screen(usb_serial_device);
    //     }
    // }

    // TODO: interrupt on usb
    // https://github.com/Jarrod-Bennett/rust-nrf52-bluetooth/blob/cda7d9cb181e3dbf6e3afb1c27427a0ece20cbb0/src/main.rs
}
