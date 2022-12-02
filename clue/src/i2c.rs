use bmp280_rs::Error as TempPressureError;
use embedded_hal::blocking::delay::DelayMs;
use hal::pac::TWIM1;
use nrf52840_hal as hal;
use shared_bus_rtic::SharedBus;
use sht3x::SHT3x;
use uom::si::{
    f32::{Pressure, Ratio, ThermodynamicTemperature},
    pressure::pascal,
    ratio::percent,
    thermodynamic_temperature::degree_celsius,
};

use crate::prelude::{AppError, AppResult};

pub type BusI2C = hal::Twim<TWIM1>;
type I2C = SharedBus<BusI2C>;
pub type TempPressureSensor = bmp280_rs::BMP280<I2C, bmp280_rs::ModeSleep>;

pub struct I2CSensors<D> {
    delay: D,

    temp_i2c: I2C,
    temp_pressure_sensor: TempPressureSensor,

    humidity_sensor: SHT3x<I2C>,
}

impl<D: DelayMs<u8>> I2CSensors<D> {
    pub fn new(i2c: BusI2C, delay: D) -> Self {
        let bus = shared_bus_rtic::new!(i2c, BusI2C);

        let mut temp_i2c = bus.acquire();

        let temp_pressure_config = bmp280_rs::Config {
            measurement_standby_time_millis: Some(
                bmp280_rs::MeasurementStandbyTimeMillis::ZeroPointFive,
            ),
            pressure_oversampling: bmp280_rs::PressureOversampling::Four,
            temperature_oversampling: bmp280_rs::TemperatureOversampling::Four,
            iir_filter: bmp280_rs::IIRFilterCoefficient::Four,
        };
        let temp_pressure_sensor = bmp280_rs::BMP280::new(
            // &mut temp_pressure_i2c,
            &mut temp_i2c,
            bmp280_rs::I2CAddress::SdoPulledUp,
            temp_pressure_config,
        )
        .unwrap();

        let humidity_i2c = bus.acquire();
        let humidity_sensor = sht3x::SHT3x::new(humidity_i2c, sht3x::Address::Low);

        Self {
            delay,

            temp_i2c,
            temp_pressure_sensor,

            humidity_sensor,
        }
    }

    //     let h = humidity
    //     .measure(sht3x::Repeatability::High, &mut delay)
    //     .unwrap();
    // let mut humstring: String<64> = String::new();
    // write!(humstring, "HUMID {} TEMP {}", h.humidity, h.temperature).unwrap();

    pub fn read_temp(&mut self) -> Result<(ThermodynamicTemperature, Pressure), TempPressureError> {
        self.temp_pressure_sensor
            .trigger_measurement(&mut self.temp_i2c)?;
        let temp = self
            .temp_pressure_sensor
            .read_temperature(&mut self.temp_i2c)?;
        let pressure = self
            .temp_pressure_sensor
            .read_pressure(&mut self.temp_i2c)?;

        // For conversions see datasheet (https://cdn-shop.adafruit.com/datasheets/BST-BMP280-DS001-11.pdf)
        // page 22
        Ok((
            ThermodynamicTemperature::new::<degree_celsius>(temp as f32 / 100.0),
            Pressure::new::<pascal>(pressure as f32 / 256.0),
        ))
    }

    pub fn read_humidity(&mut self) -> AppResult<Ratio> {
        let sht3x::Measurement {
            humidity,
            temperature: _,
        } = self
            .humidity_sensor
            .measure(sht3x::Repeatability::High, &mut self.delay)
            .map_err(|_| AppError::SHT3XError)?;

        Ok(Ratio::new::<percent>(humidity as f32 / 100.0))
    }
}
