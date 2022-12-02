extern crate linux_embedded_hal as hal;
extern crate sx127x_lora;

use hal::spidev::{self, SpidevOptions};
use hal::Delay;
use hal::Spidev;
use rppal::gpio::{Gpio, OutputPin};

use sx127x_lora::LoRa;
use uom::si::pressure::atmosphere;
use uom::si::ratio::percent;
use uom::si::thermodynamic_temperature::degree_fahrenheit;

use anyhow::{anyhow, Context, Result};

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

type LoraRadio = LoRa<Spidev, MyOutputPin, MyOutputPin, Delay>;

fn create_lora() -> Result<LoraRadio> {
    let mut spi = Spidev::open("/dev/spidev0.1").context("Failed to open spi")?;
    let options = SpidevOptions::new()
        .bits_per_word(8)
        .max_speed_hz(20_000)
        .mode(spidev::SpiModeFlags::SPI_MODE_0)
        .build();
    spi.configure(&options).context("Failed to configure spi")?;

    let gpio = Gpio::new()?;

    let cs = MyOutputPin(gpio.get(LORA_CS_PIN)?.into_output());
    let reset = MyOutputPin(gpio.get(LORA_RESET_PIN)?.into_output());

    let mut lora = sx127x_lora::LoRa::new(spi, cs, reset, FREQUENCY, Delay)
        .map_err(|_| anyhow!("Failed to communicate with radio module!"))?;

    lora.set_tx_power(17, 1)
        .map_err(|_| anyhow!("Failed to boost"))?; //Using PA_BOOST. See your board for correct pin.

    Ok(lora)
}

fn write_lora(lora: &mut LoraRadio, message: shared::Command) -> Result<()> {
    let t = shared::Transmission {
        src: shared::DevAddr(0),
        msg: message,
    };
    let mut buffer = [0; 255];
    let ser = postcard::to_slice(&t, &mut buffer)?;
    let ser_len = ser.len();

    lora.transmit_payload(buffer, ser_len)
        .map_err(|_| anyhow!("Failed to send payload"))?;

    Ok(())
}

fn read_lora(lora: &mut LoraRadio) -> Result<shared::TempPressureSensorReport> {
    let size = lora
        .poll_irq(None)
        .map_err(|_| anyhow!("Failed to poll lora"))?;
    let buffer = lora
        .read_packet()
        .map_err(|_| anyhow!("Lora data packet"))?;
    let data = postcard::from_bytes::<shared::Transmission<shared::TempPressureSensorReport>>(
        &buffer[..size],
    )?;

    Ok(data.msg)
}

fn main() -> Result<()> {
    let mut lora = create_lora()?;

    println!("Started listening");

    loop {
        let message = read_lora(&mut lora)?;

        let temp = message.temperature.get::<degree_fahrenheit>();
        let pressure = message.pressure.get::<atmosphere>();
        let humidity = message.humidity.get::<percent>();

        println!(
            "Temperature: {} F - Pressure: {} atmosphere - humidity {}",
            temp, pressure, humidity
        );

        write_lora(&mut lora, shared::Command::Led(false))?;
    }
}
