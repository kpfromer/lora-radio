use crate::{prelude::*, LoraRadio};

/// In ms
const LORA_TIMEOUT: i32 = 1000;

pub fn read_lora<'de, T>(lora: &mut LoraRadio, buffer: &'de mut [u8; 255]) -> AppResult<T>
where
    T: serde::Deserialize<'de>,
{
    let size = lora
        .poll_irq(Some(LORA_TIMEOUT))
        .map_err(|_| AppError::LoraError(LoraError::Timeout))?;

    // Ensures that deseralized data is stored in same lifetime
    buffer.copy_from_slice(
        &lora
            .read_packet()
            .map_err(|_| AppError::LoraError(LoraError::Read))?,
    );

    let value = postcard::from_bytes::<shared::Transmission<T>>(&buffer[..size])
        .map_err(|_| AppError::LoraError(LoraError::Deserialization))?;

    Ok(value.msg)
}

pub fn write_lora<T>(lora: &mut LoraRadio, data: &T) -> AppResult<()>
where
    T: serde::Serialize,
{
    let mut buffer = [0u8; 255];
    let data_size = postcard::to_slice(data, &mut buffer)
        .map_err(|_| AppError::LoraError(LoraError::Serialization))?
        .len();
    lora.transmit_payload(buffer, data_size)
        .map_err(|_| AppError::LoraError(LoraError::Write))?;

    Ok(())
}
