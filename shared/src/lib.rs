#![cfg_attr(not(feature = "std"), no_std)]

#[derive(serde::Serialize, serde::Deserialize, Debug, Clone)]
pub enum Command {
    Temp,
    Moisture,
}

#[derive(serde::Serialize, serde::Deserialize, Debug, Clone)]
pub struct Test {
    pub name: heapless::String<8>,
    pub id: u8,
    pub command: Command,
}
