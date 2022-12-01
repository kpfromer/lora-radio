#![cfg_attr(not(feature = "std"), no_std)]

use uom::si::f32::{Pressure, ThermodynamicTemperature};

#[derive(serde::Serialize, serde::Deserialize, Debug, Clone)]
pub struct TempPressureSensorReport {
    pub temperature: ThermodynamicTemperature,
    pub pressure: Pressure,
}

#[derive(serde::Serialize, serde::Deserialize, Debug, Clone)]
pub enum Command {
    Led(bool),
}

#[derive(serde::Serialize, serde::Deserialize, Debug, PartialEq, Eq)]
pub struct DevAddr(pub u8);

#[derive(serde::Serialize, serde::Deserialize, Debug)]
pub struct Transmission<T> {
    pub src: DevAddr,
    pub msg: T,
}
