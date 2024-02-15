//! Platform-agnostic  KXTJ3-1057 accelerometer driver which uses I²C via
//! [embedded-hal]. This driver implements the [`Accelerometer`][acc-trait]
//! and [`RawAccelerometer`][raw-trait] traits from the [accelerometer] crate.
//!
//! [embedded-hal]: https://docs.rs/embedded-hal
//! [accelerometer]: https://docs.rs/accelerometer
//! [acc-trait]: https://docs.rs/accelerometer/latest/accelerometer/trait.Accelerometer.html
//! [raw-trait]: https://docs.rs/accelerometer/latest/accelerometer/trait.RawAccelerometer.html
//!

#![no_std]

pub use accelerometer;
use accelerometer::error::Error as AccelerometerError;
use accelerometer::vector::{F32x3, I16x3};
use accelerometer::{Accelerometer, RawAccelerometer};
use embedded_hal::i2c::I2c;

pub mod config;
pub mod register;
use config::*;
use register::*;
use register::{DataRate, Mode, Range, Register, SlaveAddr};

/// Accelerometer errors, generic around another error type `E` representing
/// an (optional) cause of this error.
#[derive(Debug)]
pub enum Error<BusError, PinError> {
    /// I²C bus error
    Bus(BusError),
    Pin(PinError),

    /// Invalid Axis and direction
    InvalidAxis,

    /// Invalid data rate selection
    InvalidDataRate,

    /// Invalid acceleration range selection
    InvalidRange,

    /// Invalid operating mode selection
    InvalidMode,

    /// Attempted to write to a read-only register
    WriteToReadOnly,

    /// Invalid address provided
    WrongAddress,
}

/// `KXTJ3-1057` driver.
pub struct Kxtj3<I2C> {
    /// Underlying I²C device
    i2c: I2C,

    /// Current I²C slave address
    address: u8,
}

// impl<I2C, E> Kxtj3<I2C>
// where
//     I2C: WriteRead<Error = E> + i2c::Write<Error = E>,

impl<I2C, E> Kxtj3<I2C>
where
    I2C: I2c<Error = E>,
{
    /// Create a new KXTJ3-1057 driver from the given I2C peripheral.
    /// Default is Hz_400 LowPower.
    /// An example using the [esp_idf_hal](https://esp-rs.github.io/esp-idf-hal/esp_idf_hal):
    ///     
    ///     use esp_idf_hal::i2c::*;
    ///     use esp_idf_hal::peripherals::Peripherals;
    ///     use esp_idf_hal::prelude::*;
    ///     use kxtj3_1057::Kxtj3;
    ///           
    ///     let peripherals = Peripherals::take().unwrap();
    ///     let i2c = peripherals.i2c0;
    ///     
    ///     let sda = peripherals.pins.gpio10;
    ///     let scl = peripherals.pins.gpio8;
    ///     let config = I2cConfig::new().baudrate(400.kHz().into()).scl_enable_pullup(true).sda_enable_pullup(true);
    ///
    ///     let i2c = I2cDriver::new(i2c, sda, scl, &config).unwrap();     
    ///     let kxtj3 = Kxtj3::new(i2c, kxtj3_1057::SlaveAddr::Default).unwrap();     

    pub fn new(i2c: I2C, address: SlaveAddr) -> Result<Self, Error<E, core::convert::Infallible>> {
        Self::new_with_config(i2c, address, Configuration::default())
    }

    pub fn new_with_config(
        i2c: I2C,
        address: SlaveAddr,
        config: Configuration,
    ) -> Result<Self, Error<E, core::convert::Infallible>> {
        let mut kxtj3 = Kxtj3 {
            i2c,
            address: address.addr(),
        };
        kxtj3.configure(config)?;

        Ok(kxtj3)
    }

    /// Configures the device
    pub fn configure(
        &mut self,
        conf: Configuration,
    ) -> Result<(), Error<E, core::convert::Infallible>> {
        if self.get_device_id()? != DEVICE_ID {
            return Err(Error::WrongAddress);
        }
        self.enable_standby_mode()?;
        self.set_mode(conf.mode)?;
        self.set_range(conf.range)?;
        self.set_datarate(conf.datarate)?;

        if conf.enable_new_acceleration_interrupt {
            self.enable_new_accelration_interrupt()?;
        }
        if let Some(md_conf) = conf.motion_detection {
            self.set_motion_detection_datarate(md_conf.datarate)?;
            self.set_motion_detection_latch_mode(md_conf.latch_mode)?;
            self.set_motion_detection_na_counter(md_conf.non_activity_counter)?;
            self.set_motion_detection_wakeup_counter(md_conf.wakeup_counter)?;
            self.set_motion_detection_threshold(md_conf.wakeup_threshold)?;
            self.enable_motion_detection_axes(
                md_conf.enable_x_negative,
                md_conf.enable_x_positive,
                md_conf.enable_y_negative,
                md_conf.enable_y_positive,
                md_conf.enable_z_negative,
                md_conf.enable_z_positive,
            )?;
            if let Some(ip_conf) = md_conf.interrupt_pin {
                self.set_motion_detection_interrupt_pin_polarity(ip_conf.polarity)?;
                self.set_motion_detection_interrupt_pin_response(ip_conf.response)?;
            }
        }
        self.disable_standby_mode()
    }

    /// Writes a byte to the given register.
    fn write_register(
        &mut self,
        register: Register,
        value: u8,
    ) -> Result<(), Error<E, core::convert::Infallible>> {
        if register.read_only() {
            return Err(Error::WriteToReadOnly);
        }

        self.i2c
            .write(self.address, &[register.addr(), value])
            .map_err(Error::Bus)
    }

    /// Reads a byte from the given register.
    fn read_register(
        &mut self,
        register: Register,
    ) -> Result<u8, Error<E, core::convert::Infallible>> {
        let mut data = [0];
        self.i2c
            .write_read(self.address, &[register.addr()], &mut data)
            .map_err(Error::Bus)
            .and(Ok(data[0]))
    }

    /// `WHO_AM_I` register.
    pub fn get_device_id(&mut self) -> Result<u8, Error<E, core::convert::Infallible>> {
        self.read_register(Register::WHOAMI)
    }

    /// Enable stand-by mode .
    /// `CTRL_REG1`: `PC1` bit.
    pub fn enable_standby_mode(&mut self) -> Result<(), Error<E, core::convert::Infallible>> {
        self.register_clear_bits(Register::CTRL1, PC1_EN)
    }

    /// Disable stand-by mode .
    /// `CTRL_REG1`: `PC1` bit.
    pub fn disable_standby_mode(&mut self) -> Result<(), Error<E, core::convert::Infallible>> {
        self.register_set_bits(Register::CTRL1, PC1_EN)
    }

    /// Controls the operating mode of the KXTJ3 .
    /// `CTRL_REG1`: `RES` bit.
    /// Before using this function, the device must be in standby mode.
    pub fn set_mode(&mut self, mode: Mode) -> Result<(), Error<E, core::convert::Infallible>> {
        match mode {
            Mode::LowPower => {
                self.register_clear_bits(Register::CTRL1, RES_EN)?;
            }
            Mode::HighResolution => {
                self.register_set_bits(Register::CTRL1, RES_EN)?;
            }
            _ => {
                return Err(Error::InvalidMode);
            }
        }

        Ok(())
    }

    /// Reads the current operating mode.
    pub fn get_mode(&mut self) -> Result<Mode, Error<E, core::convert::Infallible>> {
        let ctrl1 = self.read_register(Register::CTRL1)?;

        let is_pc1_set = (ctrl1 >> 7) & 0x01 != 0;
        let is_res_set = (ctrl1 >> 6) & 0x01 != 0;

        let mode = match (is_pc1_set, is_res_set) {
            (false, _) => Mode::Standby,
            (true, false) => Mode::LowPower,
            (true, true) => Mode::HighResolution,
        };

        Ok(mode)
    }

    /// Data rate selection.
    ///
    /// Before using this function, the device must be in standby mode.
    pub fn set_datarate(
        &mut self,
        datarate: DataRate,
    ) -> Result<(), Error<E, core::convert::Infallible>> {
        self.modify_register(Register::DATA_CTRL, |mut data_ctrl| {
            // Mask off highest 4 bits
            data_ctrl &= !ODR_MASK;
            // Write in lowest 4 bits
            data_ctrl |= datarate.bits();

            data_ctrl
        })
    }

    /// Reads the current data selection rate.
    pub fn get_datarate(&mut self) -> Result<DataRate, Error<E, core::convert::Infallible>> {
        let data_ctrl = self.read_register(Register::DATA_CTRL)?;
        let odr = data_ctrl & 0x0F;

        DataRate::try_from(odr).map_err(|_| Error::InvalidDataRate)
    }

    /// Sets the acceleration Range.
    ///
    /// Before using this function, the device must be in standby mode.
    pub fn set_range(&mut self, range: Range) -> Result<(), Error<E, core::convert::Infallible>> {
        self.modify_register(Register::CTRL1, |mut ctrl1| {
            ctrl1 &= !GSEL_MASK;
            ctrl1 |= range.bits() << 2;
            ctrl1
        })
    }

    /// Reads the acceleration Range
    pub fn get_range(&mut self) -> Result<Range, Error<E, core::convert::Infallible>> {
        let ctrl1 = self.read_register(Register::CTRL1)?;
        let gsel = (ctrl1 >> 2) & 0x07;

        Range::try_from(gsel).map_err(|_| Error::InvalidRange)
    }

    /// Reads from the registers for each of the 3 axes.
    fn read_accel_bytes(&mut self) -> Result<[u8; 6], Error<E, core::convert::Infallible>> {
        let mut data = [0u8; 6];

        self.i2c
            .write_read(self.address, &[Register::XOUT_L.addr() | 0x80], &mut data)
            .map_err(Error::Bus)
            .and(Ok(data))
    }

    /// Enables the reporting of the availability of new acceleration data as an interrupt.
    ///
    /// Before using this function, the device must be in standby mode.
    pub fn enable_new_accelration_interrupt(
        &mut self,
    ) -> Result<(), Error<E, core::convert::Infallible>> {
        self.register_set_bits(Register::CTRL1, DRDYE_EN)
    }

    /// Enables the Wake-Up (motion detect) function.
    ///
    /// Before using this function, the device must be in standby mode.
    pub fn enable_motion_detection(&mut self) -> Result<(), Error<E, core::convert::Infallible>> {
        self.register_set_bits(Register::CTRL1, WUFE_EN)
    }

    /// Enables the physical interrupt pin (INT).
    ///
    /// Before using this function, the device must be in standby mode.
    pub fn enable_motion_detection_physical_interrupt_pin(
        &mut self,
    ) -> Result<(), Error<E, core::convert::Infallible>> {
        self.register_set_bits(Register::INT_CTRL1, IEN_EN)
    }

    /// Sets the polarity of the physical interrupt pin
    ///
    /// Before using this function, the device must be in standby mode.
    pub fn set_motion_detection_interrupt_pin_polarity(
        &mut self,
        polarity: InterruptPinPolarity,
    ) -> Result<(), Error<E, core::convert::Infallible>> {
        match polarity {
            InterruptPinPolarity::ActiveHigh => self.register_set_bits(Register::INT_CTRL1, IEA_EN),
            InterruptPinPolarity::ActiveLow => {
                self.register_clear_bits(Register::INT_CTRL1, IEA_EN)
            }
        }
    }

    /// Sets the response of the physical interrupt pin.
    ///
    /// Before using this function, the device must be in standby mode.
    pub fn set_motion_detection_interrupt_pin_response(
        &mut self,
        response: InterruptPinResponse,
    ) -> Result<(), Error<E, core::convert::Infallible>> {
        match response {
            InterruptPinResponse::Latched => self.register_clear_bits(Register::INT_CTRL1, IEL_EN),
            InterruptPinResponse::Pulsed => self.register_set_bits(Register::INT_CTRL1, IEL_EN),
        }
    }

    /// Sets the Output Data Rate for the motion detection function
    ///
    /// Before using this function, the device must be in standby mode.
    pub fn set_motion_detection_datarate(
        &mut self,
        datarate: MotionDetectionDataRate,
    ) -> Result<(), Error<E, core::convert::Infallible>> {
        self.modify_register(Register::CTRL2, |mut ctrl2| {
            ctrl2 &= !ODRW_MASK;
            ctrl2 |= datarate.bits();
            ctrl2
        })
    }

    /// Reads the current data selection rate the motion detection function.
    pub fn get_motion_detection_datarate(
        &mut self,
    ) -> Result<MotionDetectionDataRate, Error<E, core::convert::Infallible>> {
        let data_ctrl = self.read_register(Register::CTRL2)?;
        let odr = data_ctrl & 0x07;
        MotionDetectionDataRate::try_from(odr).map_err(|_| Error::InvalidDataRate)
    }

    /// Sets the motion detection latch mode.
    ///
    /// Before using this function, the device must be in standby mode.
    pub fn set_motion_detection_latch_mode(
        &mut self,
        latch_mode: MotionDetectionLatchMode,
    ) -> Result<(), Error<E, core::convert::Infallible>> {
        self.modify_register(Register::INT_CTRL2, |mut int_ctrl2| {
            int_ctrl2 &= !ULMODE_EN;
            int_ctrl2 |= latch_mode.bits() << 7;
            int_ctrl2
        })
    }

    /// Sets the time motion must be present before a wake-up interrupt is set.
    ///
    /// `WAKEUP_COUNTER (counts) = Wake-Up Delay Time (sec) x Wake-Up Function ODR (Hz)`
    ///
    /// Before using this function, the device must be in standby mode.
    pub fn set_motion_detection_wakeup_counter(
        &mut self,
        wakeup_counter: u8,
    ) -> Result<(), Error<E, core::convert::Infallible>> {
        self.write_register(Register::WAKEUP_COUNTER, wakeup_counter)
    }

    /// Sets the non-activity time required before another wake-up interrupt can be set.
    ///
    /// `NA_COUNTER (counts) = Non-Activity Time (sec) x Wake-Up Function ODR (Hz)`
    ///
    /// Before using this function, the device must be in standby mode.
    pub fn set_motion_detection_na_counter(
        &mut self,
        na_counter: u8,
    ) -> Result<(), Error<E, core::convert::Infallible>> {
        self.write_register(Register::NA_COUNTER, na_counter)
    }

    /// Sets the threshold for motion detection interrupt is set.
    ///
    /// `WAKEUP_THRESHOLD (counts) = Desired Threshold (g) x 256 (counts/g)`
    ///
    /// Before using this function, the device must be in standby mode.
    pub fn set_motion_detection_threshold(
        &mut self,
        desired_threshold: u8,
    ) -> Result<(), Error<E, core::convert::Infallible>> {
        let upper_8_bits = (desired_threshold >> 4) as u8;
        let lower_8_bits = (desired_threshold << 4) as u8;
        self.write_register(Register::WAKEUP_THRESHOLD_H, upper_8_bits)?;
        self.write_register(Register::WAKEUP_THRESHOLD_L, lower_8_bits)
    }

    /// Reports the axis and direction of detected motion.
    pub fn get_motion_detection_axis(
        &mut self,
    ) -> Result<MotionDetectionAxis, Error<E, core::convert::Infallible>> {
        let int_src2 = self.read_register(Register::INT_SOURCE2)?;
        MotionDetectionAxis::try_from(int_src2).map_err(|_| Error::InvalidAxis)
    }
    ///Indicates Wake-up has occurred or not.
    pub fn is_motion_detected(&mut self) -> Result<bool, Error<E, core::convert::Infallible>> {
        let int_src1 = self.read_register(Register::INT_SOURCE1)?;
        let wufs = (int_src1 >> 1) & 0x01 != 0;
        Ok(wufs)
    }

    /// Indicates that new acceleration data (0x06 to 0x0B) is available or not .
    pub fn is_acceleration_data_ready(
        &mut self,
    ) -> Result<bool, Error<E, core::convert::Infallible>> {
        let int_src1 = self.read_register(Register::INT_SOURCE1)?;
        let drdy = (int_src1 >> 4) & 0x01 != 0;
        Ok(drdy)
    }

    /// Sets which axes and directions of detected motion can cause an interrupt.
    ///
    /// Before using this function, the device must be in standby mode.
    pub fn enable_motion_detection_axes(
        &mut self,
        x_negative: bool,
        x_positive: bool,
        y_negative: bool,
        y_positive: bool,
        z_negative: bool,
        z_positive: bool,
    ) -> Result<(), Error<E, core::convert::Infallible>> {
        self.modify_register(Register::INT_CTRL2, |mut int_ctrl2| {
            int_ctrl2 &= !WUE_MASK;
            int_ctrl2 |= if x_negative {
                MotionDetectionAxis::X_Negative.bits()
            } else {
                0
            };
            int_ctrl2 |= if x_positive {
                MotionDetectionAxis::X_Positive.bits()
            } else {
                0
            };
            int_ctrl2 |= if y_negative {
                MotionDetectionAxis::Y_Negative.bits()
            } else {
                0
            };
            int_ctrl2 |= if y_positive {
                MotionDetectionAxis::Y_Positive.bits()
            } else {
                0
            };
            int_ctrl2 |= if z_negative {
                MotionDetectionAxis::Z_Negative.bits()
            } else {
                0
            };
            int_ctrl2 |= if z_positive {
                MotionDetectionAxis::Z_Positive.bits()
            } else {
                0
            };
            int_ctrl2
        })
    }

    /// Clears Latched interrupt source information (INT_SOURCE1 and INT_SOURCE2).
    /// Changes physical interrupt latched pin (INT) to inactive state.
    pub fn clear_motion_detection_lathced_info(
        &mut self,
    ) -> Result<(), Error<E, core::convert::Infallible>> {
        self.read_register(Register::INT_REL)?;
        Ok(())
    }

    /// Modify a register's value. Read the current value of the register,
    /// update the value with the provided function, and set the register to
    /// the return value.
    fn modify_register<F>(
        &mut self,
        register: Register,
        f: F,
    ) -> Result<(), Error<E, core::convert::Infallible>>
    where
        F: FnOnce(u8) -> u8,
    {
        let value = self.read_register(register)?;

        self.write_register(register, f(value))
    }

    /// Clear the given bits in the given register. For example:
    ///
    ///     kxtj3.register_clear_bits(0b0110)
    ///
    /// This call clears (sets to 0) the bits at index 1 and 2. Other bits of the register are not touched.
    pub fn register_clear_bits(
        &mut self,
        reg: Register,
        bits: u8,
    ) -> Result<(), Error<E, core::convert::Infallible>> {
        self.modify_register(reg, |v| v & !bits)
    }

    /// Set the given bits in the given register. For example:
    ///
    ///     kxtj3.register_set_bits(0b0110)
    ///
    /// This call sets to 1 the bits at index 1 and 2. Other bits of the register are not touched.
    pub fn register_set_bits(
        &mut self,
        reg: Register,
        bits: u8,
    ) -> Result<(), Error<E, core::convert::Infallible>> {
        self.modify_register(reg, |v| v | bits)
    }
}

impl<I2C, E> RawAccelerometer<I16x3> for Kxtj3<I2C>
where
    I2C: I2c<Error = E>,
    E: core::fmt::Debug,
{
    type Error = Error<E, core::convert::Infallible>;

    fn accel_raw(&mut self) -> Result<I16x3, AccelerometerError<Self::Error>> {
        let accel_bytes = self.read_accel_bytes()?;

        let x = i16::from_le_bytes(accel_bytes[0..2].try_into().unwrap());
        let y = i16::from_le_bytes(accel_bytes[2..4].try_into().unwrap());
        let z = i16::from_le_bytes(accel_bytes[4..6].try_into().unwrap());

        Ok(I16x3::new(x, y, z))
    }
}

impl<I2C, E> Accelerometer for Kxtj3<I2C>
where
    I2C: I2c<Error = E>,
    E: core::fmt::Debug,
{
    type Error = Error<E, core::convert::Infallible>;

    /// Get normalized ±g reading from the accelerometer.
    fn accel_norm(&mut self) -> Result<F32x3, AccelerometerError<Self::Error>> {
        let mode = self.get_mode()?;
        let range = self.get_range()?;

        // See "Mechanical Specifications" in the datasheet to find the values below.

        // Depending on which Mode we are operating in, the data has different
        // resolution. Using this knowledge, we determine how many bits the
        // data needs to be shifted. This is necessary because the raw data
        // is in left-justified two's complement and we would like for it to be
        // right-justified instead.

        let (scale, shift) = match (mode, range) {
            // High Resolution mode - 14-bit data output
            (Mode::HighResolution, Range::G8_14Bit) => (0.001, 2),
            (Mode::HighResolution, Range::G16_14Bit) => (0.002, 2),
            // High Resolution mode-12 bit data output
            (Mode::HighResolution, Range::G2) => (0.001, 4),
            (Mode::HighResolution, Range::G4) => (0.002, 4),
            (Mode::HighResolution, Range::G8) => (0.004, 4),
            (Mode::HighResolution, Range::G16_1)
            | (Mode::HighResolution, Range::G16_2)
            | (Mode::HighResolution, Range::G16_3) => (0.008, 4),

            // Low power mode
            (Mode::LowPower, Range::G2) => (0.015, 8),
            (Mode::LowPower, Range::G4) => (0.031, 8),
            (Mode::LowPower, Range::G8) => (0.062, 8),
            (Mode::LowPower, Range::G16_1)
            | (Mode::LowPower, Range::G16_2)
            | (Mode::LowPower, Range::G16_3) => (0.125, 8),

            _ => (0.0, 0),
        };

        let acc_raw = self.accel_raw()?;
        let x = (acc_raw.x >> shift) as f32 * scale;
        let y = (acc_raw.y >> shift) as f32 * scale;
        let z = (acc_raw.z >> shift) as f32 * scale;

        Ok(F32x3::new(x, y, z))
    }

    /// Get the sample rate of the accelerometer data.
    fn sample_rate(&mut self) -> Result<f32, AccelerometerError<Self::Error>> {
        Ok(self.get_datarate()?.sample_rate())
    }
}
