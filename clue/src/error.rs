//! Application errors

use core::convert::Infallible;

// pub type FResult =
pub type AppResult<T> = core::result::Result<T, AppError>;

#[derive(Debug, Clone, Copy)]
pub enum LoraError {
    Timeout,
    Read,
    Write,
    Serialization,
    Deserialization,
}

/// User visible errors
#[derive(Debug, Clone, Copy)]
pub enum AppError {
    Duh,
    FmtError,
    SoilReadingError,
    UsbSerialError,
    DisplayError(&'static str),

    SHT3XError,

    LoraError(LoraError),
}

impl From<Infallible> for AppError {
    fn from(_: Infallible) -> Self {
        AppError::Duh {}
    }
}

impl From<core::fmt::Error> for AppError {
    fn from(_: core::fmt::Error) -> Self {
        AppError::FmtError
    }
}
