#![no_main]
#![no_std]

pub mod display;
pub mod error;
pub mod i2c;
pub mod led;
pub mod lora;
pub mod panic;
pub mod soil;
pub mod usb_serial;

pub mod prelude {
    pub use crate::error::*;
}
use prelude::*;

use hal::timer::Periodic;
use nrf52840_hal::prelude::*;

use i2c::*;

use hal::Spim;
use hal::Timer;

use hal::gpio::p0::P0_02;
use hal::gpio::p0::P0_28;
use hal::gpio::Output;
use hal::gpio::PushPull;
use hal::pac::SPIM2;
use hal::pac::{TIMER2, TIMER3, TIMER4};
use hal::twim::Twim;

use nrf52840_hal as _;

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

use display::DisplayDevice;
use led::Led;
use lora::{read_lora, write_lora};
use shared::Command;

const FREQUENCY: i64 = 915;

// pub const I2C_GYROACCEL: u8 = 0x6A;
// pub const I2C_MAGNETOMETER: u8 = 0x1c;
// pub const I2C_GESTURE: u8 = 0x39;
pub const I2C_HUMIDITY: u8 = 0x44;
pub const I2C_TEMPPRESSURE: u8 = 0x77;

pub type LoraRadio =
    LoRa<Spim<SPIM2>, P0_28<Output<PushPull>>, P0_02<Output<PushPull>>, Timer<TIMER4>>;

#[app(device = nrf52840_hal::pac, peripherals = true, dispatchers = [PWM0, PWM1, PWM3])]
mod app {
    use super::*;

    #[shared]
    struct Shared {
        display_device: DisplayDevice<nrf52840_hal::pac::TIMER1>,
    }

    #[local]
    struct Local {
        white_led: Led,
        lora: LoraRadio,
        timer: Timer<TIMER3, Periodic>,
        i2c: I2CSensors<Timer<TIMER2, Periodic>>,
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

    #[init(local=[
        clocks: Option<Clocks<ExternalOscillator, Internal, LfOscStopped>> = None,
        usb_bus: Option<UsbBusAllocator<Usbd<UsbPeripheral<'static>>>> = None
    ])]
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

        let white_led = Led::new(port0.p0_10.degrade());

        let mut display = {
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
            ST7789::new(
                display_interface,
                Some(tft_reset),
                Some(tft_backlight),
                240,
                240,
            )
        };

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

        // GPIO interrupts
        let btn = port1.p1_02.into_pullup_input().degrade();
        let gpiote = Gpiote::new(cx.device.GPIOTE);
        gpiote.port().input_pin(&btn).low();
        gpiote.port().enable_interrupt();

        let lora = {
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
                cx.device.SPIM2,
                pins,
                hal::spim::Frequency::M4,
                hal::spim::MODE_0,
                122,
            );

            let mut lora = sx127x_lora::LoRa::new(spi, cs, reset, FREQUENCY, timer).unwrap();
            lora.set_tx_power(15, 1).unwrap();
            lora
        };

        let timer = Timer::new(cx.device.TIMER3).into_periodic();

        let i2c = {
            // NOTE: TWIM0 interfears with spim0, twim1 interfears with spim1
            // https://infocenter.nordicsemi.com/index.jsp?topic=%2Fcom.nordic.infocenter.nrf52832.ps.v1.1%2Ftwim.html
            let pins = hal::twim::Pins {
                scl: port0.p0_25.into_floating_input().degrade(),
                sda: port0.p0_24.into_floating_input().degrade(),
            };
            let i2c = Twim::new(cx.device.TWIM1, pins, hal::twim::Frequency::K400);

            I2CSensors::new(i2c, Timer::new(cx.device.TIMER2).into_periodic())
        };

        handle_broadcast::spawn().unwrap();

        (
            Shared { display_device },
            Local {
                white_led,
                lora,
                timer,
                i2c,
            },
            init::Monotonics(monotonic),
        )
    }

    #[task(local = [white_led], shared = [display_device])]
    fn handle_command(mut cx: handle_command::Context, command: Command) {
        let white_led = cx.local.white_led;

        match command {
            Command::Led(status) => {
                if status {
                    white_led.on();
                } else {
                    white_led.off();
                }
            }
            Command::Display(status) => {
                cx.shared
                    .display_device
                    .lock(|display| if status { display.on() } else { display.off() });
            }
        }
    }

    #[task(local = [lora, timer, i2c])]
    fn handle_broadcast(cx: handle_broadcast::Context) {
        let handle_broadcast::LocalResources { lora, timer, i2c } = cx.local;

        let (temperature, pressure) = i2c.read_temp().unwrap();
        let humidity = i2c.read_humidity().unwrap();

        write_lora(
            lora,
            &shared::Transmission {
                src: shared::DevAddr(1),
                msg: shared::TempPressureSensorReport {
                    temperature,
                    pressure,
                    humidity,
                },
            },
        )
        .unwrap();
        timer.delay_ms(50u32); // ensure gap between transmissions

        let mut data_buffer = [0u8; 255];
        match read_lora::<shared::Command>(lora, &mut data_buffer) {
            Ok(command) => handle_command::spawn(command).unwrap(),
            // Continue on timeout or any other lora error
            Err(AppError::LoraError(_)) => {}
            _ => panic!(),
        }

        handle_broadcast::spawn_after(5.secs()).unwrap();
    }

    // TODO: gpiote toggle display
}
