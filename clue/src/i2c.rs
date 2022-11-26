use bmp280_rs::Error as TempPressureError;
use nrf52840_hal as hal;

pub type I2C = hal::Twim<hal::pac::TWIM1>;
pub type TempPressureSensor = bmp280_rs::BMP280<I2C, bmp280_rs::ModeSleep>;

pub struct I2CSensors {
    i2c: I2C,
    temp_pressure_sensor: TempPressureSensor,
}

impl I2CSensors {
    pub fn new(mut i2c: I2C) -> Self {
        let temp_pressure_config = bmp280_rs::Config {
            measurement_standby_time_millis: Some(
                bmp280_rs::MeasurementStandbyTimeMillis::ZeroPointFive,
            ),
            pressure_oversampling: bmp280_rs::PressureOversampling::Four,
            temperature_oversampling: bmp280_rs::TemperatureOversampling::Four,
            iir_filter: bmp280_rs::IIRFilterCoefficient::Four,
        };
        // let mut temp_pressure_i2c = shared_sensor_i2c.acquire_i2c();
        // cx.local.temp_pressure_i2c.replace(temp_pressure_i2c);
        let temp_pressure_sensor = bmp280_rs::BMP280::new(
            // &mut temp_pressure_i2c,
            &mut i2c,
            bmp280_rs::I2CAddress::SdoPulledUp,
            temp_pressure_config,
        )
        .unwrap();

        Self {
            i2c,
            temp_pressure_sensor,
        }
    }
    pub fn read_temp(&mut self) -> Result<(i32, i32), TempPressureError> {
        self.temp_pressure_sensor
            .trigger_measurement(&mut self.i2c)?;
        let temp = self.temp_pressure_sensor.read_temperature(&mut self.i2c)?;
        let pressure = self.temp_pressure_sensor.read_pressure(&mut self.i2c)?;
        Ok((temp, pressure))
    }
}
