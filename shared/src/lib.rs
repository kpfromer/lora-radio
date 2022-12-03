#![cfg_attr(not(feature = "std"), no_std)]

use uom::si::f32::{Pressure, Ratio, ThermodynamicTemperature};

#[derive(serde::Serialize, serde::Deserialize, Debug, Clone)]
pub struct TempPressureSensorReport {
    pub temperature: ThermodynamicTemperature,
    pub pressure: Pressure,
}

#[derive(serde::Serialize, serde::Deserialize, Debug, Clone)]
pub struct HumidityReport {
    pub humidity: Ratio,
}

#[derive(serde::Serialize, serde::Deserialize, Debug, Clone)]
pub enum Message {
    TempPressureSensorReport(TempPressureSensorReport),
    HumidityReport(HumidityReport),
}

#[derive(serde::Serialize, serde::Deserialize, Debug, Clone)]
pub enum Command {
    Display(bool),
    Led(bool),
}

#[derive(serde::Serialize, serde::Deserialize, Debug, PartialEq, Eq)]
pub struct DevAddr(pub u8);

#[derive(serde::Serialize, serde::Deserialize, Debug)]
pub struct Transmission<T> {
    // TODO: add to field
    pub src: DevAddr,
    pub msg: T,
}
