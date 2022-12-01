extern crate linux_embedded_hal as hal;
extern crate sx127x_lora;

use std::env;
use std::fmt::Write;

use hal::spidev::{self, SpidevOptions};
use hal::Delay;
use hal::Spidev;
use rppal::gpio::{Gpio, OutputPin};

use uom::si::pressure::atmosphere;
use uom::si::thermodynamic_temperature::{degree_celsius, degree_fahrenheit};

#[allow(deprecated)]
use embedded_hal::digital::v1::OutputPin as HalOutputPin;

// Gpio uses BCM pin numbering
const LORA_CS_PIN: u8 = 7;
const LORA_RESET_PIN: u8 = 25;
const FREQUENCY: i64 = 915;

struct MyOutputPin(OutputPin);

#[allow(deprecated)]
impl HalOutputPin for MyOutputPin {
    fn set_low(&mut self) {
        self.0.set_low();
    }

    fn set_high(&mut self) {
        self.0.set_high();
    }
}

fn main() {
    let mut spi = Spidev::open("/dev/spidev0.1").unwrap();
    let options = SpidevOptions::new()
        .bits_per_word(8)
        .max_speed_hz(20_000)
        .mode(spidev::SpiModeFlags::SPI_MODE_0)
        .build();
    spi.configure(&options).unwrap();

    let gpio = Gpio::new().unwrap();

    let cs = MyOutputPin(gpio.get(LORA_CS_PIN).unwrap().into_output());
    let reset = MyOutputPin(gpio.get(LORA_RESET_PIN).unwrap().into_output());

    let mut lora = sx127x_lora::LoRa::new(spi, cs, reset, FREQUENCY, Delay)
        .expect("Failed to communicate with radio module!");

    lora.set_tx_power(17, 1).expect("Failed to boost"); //Using PA_BOOST. See your board for correct pin.

    // let size = lora.poll_irq(None).unwrap();
    // let data = lora.read_packet().unwrap();
    // println!("DATA: {}", std::str::from_utf8(&data[..size]).unwrap());

    // ----------

    let t = shared::Transmission {
        src: shared::DevAddr(0),
        msg: shared::Command::GetTempPressure,
    };
    let mut buffer = [0; 255];
    let ser = postcard::to_slice(&t, &mut buffer).unwrap();
    let ser_len = ser.len();
    println!("Transmitting command: {:?}", t);

    // let message = "Hello, world!";
    // let args: Vec<String> = env::args().collect();
    // let message = args[1].clone();
    // let mut buffer = [0; 255];
    // for (i, c) in message.chars().enumerate() {
    //     buffer[i] = c as u8;
    // }

    lora.transmit_payload(buffer, ser_len)
        .expect("Failed to send payload");

    println!("Done!");

    let size = lora.poll_irq(None).unwrap();
    let data = postcard::from_bytes::<shared::Transmission<shared::TempPressureSensorReport>>(
        &lora.read_packet().expect("Lora data packet")[..size],
    )
    .expect("Failed to parse structure");

    let temp = data.msg.temperature.get::<degree_fahrenheit>();
    let pressure = data.msg.pressure.get::<atmosphere>();

    println!("Temperature: {} F", temp);
    println!("Pressure: {} atmosphere", pressure);
}
