use accelerometer::{RawAccelerometer, Tracker};
use esp_idf_hal::delay::FreeRtos;
use esp_idf_hal::i2c::*;
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::prelude::*;
use esp_idf_sys;
use kxtj3_1057::{Kxtj3, Mode, Range, SlaveAddr};
fn main() {
    esp_idf_sys::link_patches();

    let peripherals = Peripherals::take().unwrap();
    let i2c = peripherals.i2c0;
    let sda = peripherals.pins.gpio10;
    let scl = peripherals.pins.gpio8;

    let config = I2cConfig::new().baudrate(400.kHz().into());
    let i2c = I2cDriver::new(i2c, sda, scl, &config).unwrap();

    let mut kxtj3 = Kxtj3::new(i2c, SlaveAddr::Default).unwrap();
    kxtj3.set_range(Range::G2).unwrap();
    kxtj3.set_mode(Mode::LowPower).unwrap();
    let mut tracker = Tracker::new(3700.0);
    loop {
        let accel = kxtj3.accel_raw().unwrap();
        let orientation = tracker.update(accel);
        println!("{:?}", orientation);
        FreeRtos::delay_ms(1000);
    }
}
