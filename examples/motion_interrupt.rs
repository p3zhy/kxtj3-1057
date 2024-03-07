use esp_idf_svc::hal::{delay::FreeRtos, i2c::*, prelude::Peripherals, units::Hertz};
use kxtj3_1057::{
    config::MotionDetctionConfiguration,
    register::{DataRate, Mode, Range, SlaveAddr},
    Kxtj3,
};
fn main() {
    esp_idf_svc::sys::link_patches();

    let peripherals = Peripherals::take().unwrap();
    let i2c = peripherals.i2c0;
    let sda = peripherals.pins.gpio10;
    let scl = peripherals.pins.gpio8;

    let config = I2cConfig::new()
        .baudrate(Hertz(400_000))
        .scl_enable_pullup(true)
        .sda_enable_pullup(true);
    let i2c = I2cDriver::new(i2c, sda, scl, &config).unwrap();

    let device_config = kxtj3_1057::config::Configuration {
        mode: Mode::HighResolution,
        datarate: DataRate::Hz_6_25,
        range: Range::G2,
        enable_new_acceleration_interrupt: true,
        motion_detection: Some(MotionDetctionConfiguration::default()),
    };

    let mut kxtj3 = Kxtj3::new_with_config(i2c, SlaveAddr::Default, device_config).unwrap();

    loop {
        if kxtj3.is_motion_detected().unwrap() {
            println!("{:?}", kxtj3.get_motion_detection_axis().unwrap());
            let _ = kxtj3.clear_motion_detection_lathced_info();
        }

        FreeRtos::delay_ms(500);
    }
}
