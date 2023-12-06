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
use accelerometer::vector::I16x3;
use accelerometer::RawAccelerometer;
// use accelerometer::{Accelerometer, RawAccelerometer};
use embedded_hal::blocking::i2c::{self, WriteRead};

mod register;

use register::*;
pub use register::{Mode, Register, SlaveAddr, DEVICE_ID};

/// Accelerometer errors, generic around another error type `E` representing
/// an (optional) cause of this error.
#[derive(Debug)]
pub enum Error<BusError, PinError> {
    /// I²C bus error
    Bus(BusError),
    Pin(PinError),

    /// Invalid data rate selection
    InvalidDataRate,

    /// Invalid selects the acceleration range
    InvalidRange,

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

impl<I2C, E> Kxtj3<I2C>
where
    I2C: WriteRead<Error = E> + i2c::Write<Error = E>,
{
    /// TODO:Example
    pub fn new(i2c: I2C, address: SlaveAddr) -> Result<Self, Error<E, core::convert::Infallible>> {
        let mut kxtj3 = Kxtj3 {
            i2c,
            address: address.addr(),
        };

        if kxtj3.get_device_id()? != DEVICE_ID {
            return Err(Error::WrongAddress);
        }

        Ok(kxtj3)
    }

    /// Write a byte to the given register.
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

    /// Read a byte from the given register.
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

    /// Controls the operating mode of the KXTJ3 .
    /// `CTRL_REG1`: `PC1` bit , CTRL_REG1`: `RES` bit.
    pub fn set_mode(&mut self, mode: Mode) -> Result<(), Error<E, core::convert::Infallible>> {
        self.register_clear_bits(Register::CTRL1, PC1_EN)?;
        match mode {
            Mode::LowPower => {
                self.register_clear_bits(Register::CTRL1, RES_EN)?;
                self.register_set_bits(Register::CTRL1, PC1_EN)?;
            }
            Mode::Standby => {}
            Mode::HighResolution => {
                self.register_set_bits(Register::CTRL1, RES_EN)?;
                self.register_set_bits(Register::CTRL1, PC1_EN)?;
            }
        }

        Ok(())
    }

    /// Read the current operating mode.
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
    pub fn set_datarate(
        &mut self,
        datarate: DataRate,
    ) -> Result<(), Error<E, core::convert::Infallible>> {
        self.register_clear_bits(Register::CTRL1, PC1_EN)?;
        self.modify_register(Register::DATA_CTRL, |mut data_ctrl| {
            // Mask off highest 4 bits
            data_ctrl &= !ODR_MASK;
            // Write in lowest 4 bits
            data_ctrl |= datarate.bits();

            data_ctrl
        })?;
        self.register_set_bits(Register::CTRL1, PC1_EN)
    }

    /// Read the current data selection rate.
    pub fn get_datarate(&mut self) -> Result<DataRate, Error<E, core::convert::Infallible>> {
        let data_ctrl = self.read_register(Register::DATA_CTRL)?;
        let odr = data_ctrl & 0x0F;

        DataRate::try_from(odr).map_err(|_| Error::InvalidDataRate)
    }

    /// Set the acceleration Range
    pub fn set_range(&mut self, range: Range) -> Result<(), Error<E, core::convert::Infallible>> {
        self.register_clear_bits(Register::CTRL1, PC1_EN)?;
        self.modify_register(Register::CTRL1, |mut ctrl1| {
            ctrl1 &= !GSEL_MASK;
            ctrl1 |= range.bits() << 2;

            ctrl1
        })?;
        self.register_set_bits(Register::CTRL1, PC1_EN)
    }

    /// Read the acceleration Range
    pub fn get_range(&mut self) -> Result<Range, Error<E, core::convert::Infallible>> {
        let ctrl1 = self.read_register(Register::CTRL1)?;
        let gsel = (ctrl1 >> 2) & 0x07;

        Range::try_from(gsel).map_err(|_| Error::InvalidRange)
    }

    /// Read from the registers for each of the 3 axes.
    fn read_accel_bytes(&mut self) -> Result<[u8; 6], Error<E, core::convert::Infallible>> {
        let mut data = [0u8; 6];

        self.i2c
            .write_read(self.address, &[Register::XOUT_L.addr() | 0x80], &mut data)
            .map_err(Error::Bus)
            .and(Ok(data))
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
    I2C: WriteRead<Error = E> + i2c::Write<Error = E>, E: core::fmt::Debug
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
