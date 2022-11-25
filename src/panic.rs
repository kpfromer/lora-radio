use crate::led::Led;

use nrf52840_hal as hal;

#[panic_handler] // panicking behavior
fn panic(_: &core::panic::PanicInfo) -> ! {
    let device = unsafe { hal::pac::Peripherals::steal() };

    let port1 = hal::gpio::p1::Parts::new(device.P1);

    let mut red_led = Led::new(port1.p1_01.degrade());
    red_led.on();

    loop {
        cortex_m::asm::nop();
    }
}
