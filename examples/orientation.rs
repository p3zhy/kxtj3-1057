use accelerometer::{RawAccelerometer, Tracker};
use esp_idf_svc::hal::{delay::FreeRtos, i2c::*, prelude::Peripherals, units::Hertz};
use kxtj3_1057::{register::SlaveAddr, Kxtj3};
fn main() {
    esp_idf_svc::sys::link_patches();
    let peripherals = Peripherals::take().unwrap();
    let i2c = peripherals.i2c0;
    let sda = peripherals.pins.gpio2;
    let scl = peripherals.pins.gpio1;
    let config = I2cConfig::new()
        .baudrate(Hertz(400_000))
        .scl_enable_pullup(true)
        .sda_enable_pullup(true);
    let i2c = I2cDriver::new(i2c, sda, scl, &config).unwrap();
    let mut kxtj3 = Kxtj3::new(i2c, SlaveAddr::Default).unwrap();
    let mut tracker = Tracker::new(3700.0);
    loop {
        let accel = kxtj3.accel_raw().unwrap();
        let orientation = tracker.update(accel);
        println!("{:?}", orientation);
        FreeRtos::delay_ms(1000);
    }
}
