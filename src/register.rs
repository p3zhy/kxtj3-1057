//! This module provides definitions for controlling the accelerometer sensor.

use num_enum::TryFromPrimitive;

/// Possible I²C slave addresses.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum SlaveAddr {
    /// Default slave address (`0x0E`)
    Default = 0x0E,

    /// Alternate slave address (`0x0F`)
    Alternate = 0x0F,
}

impl SlaveAddr {
    pub fn addr(self) -> u8 {
        self as u8
    }
}

/// Enumerate all device registers.
#[allow(dead_code, non_camel_case_types, clippy::upper_case_acronyms)]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum Register {
    WHOAMI = 0x0F,
    DCST_RESP = 0x0C,
    SOFT_REST = 0x7F,
    XOUT_L = 0x06,
    XOUT_H = 0x07,
    YOUT_L = 0x08,
    YOUT_H = 0x09,
    ZOUT_L = 0x0A,
    ZOUT_H = 0x0B,
    STATUS_REG = 0x18,
    INT_SOURCE1 = 0x16,
    INT_SOURCE2 = 0x17,
    INT_REL = 0x1A,
    CTRL1 = 0x1B,
    CTRL2 = 0x1D,
    INT_CTRL1 = 0x1E,
    INT_CTRL2 = 0x1F,
    DATA_CTRL = 0x21,
    WAKEUP_COUNTER = 0x29,
    NA_COUNTER = 0x2A,
    SELF_TEST = 0x3A,
    WAKEUP_THRESHOLD_H = 0x6A,
    WAKEUP_THRESHOLD_L = 0x6B,
}

impl Register {
    /// Get register address
    pub fn addr(self) -> u8 {
        self as u8
    }

    /// Is the register read-only?
    pub fn read_only(self) -> bool {
        matches!(
            self,
            Register::WHOAMI
                | Register::XOUT_L
                | Register::XOUT_H
                | Register::YOUT_H
                | Register::YOUT_L
                | Register::ZOUT_H
                | Register::ZOUT_L
                | Register::DCST_RESP
                | Register::INT_REL
                | Register::STATUS_REG
                | Register::INT_SOURCE1
                | Register::INT_SOURCE2
        )
    }
}

///The operating mode .
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum Mode {
    /// High-resolution mode (12-bit or 14-bit data output) .
    HighResolution,

    /// Low-power mode (8-bit data output).
    LowPower,

    /// Stand-by mode
    Standby,
}

/// The acceleration output data rate.
#[allow(non_camel_case_types)]
#[derive(Copy, Clone, Debug, Eq, PartialEq, TryFromPrimitive)]
#[repr(u8)]
pub enum DataRate {
    Hz_0_781 = 0b1000,
    Hz_1_563 = 0b1001,
    Hz_3_125 = 0b1010,
    Hz_6_25 = 0b1011,
    Hz_12_5 = 0b0000,
    Hz_25 = 0b0001,
    Hz_50 = 0b0010,
    Hz_100 = 0b0011,
    Hz_200 = 0b0100,
    Hz_400HZ = 0b0101,
    Hz_800HZ = 0b0110,
    Hz_1600HZ = 0b0111,
}

impl DataRate {
    pub const fn bits(self) -> u8 {
        self as u8
    }
    pub const fn sample_rate(self) -> f32 {
        match self {
            DataRate::Hz_0_781 => 0.781,
            DataRate::Hz_1_563 => 1.563,
            DataRate::Hz_3_125 => 3.125,
            DataRate::Hz_6_25 => 6.25,
            DataRate::Hz_12_5 => 12.5,
            DataRate::Hz_25 => 25.0,
            DataRate::Hz_50 => 50.0,
            DataRate::Hz_100 => 100.0,
            DataRate::Hz_200 => 200.0,
            DataRate::Hz_400HZ => 400.0,
            DataRate::Hz_800HZ => 800.0,
            DataRate::Hz_1600HZ => 1600.0,
        }
    }
}
///The acceleration range.
#[allow(non_camel_case_types)]
#[derive(Copy, Clone, Debug, Eq, PartialEq, TryFromPrimitive)]
#[repr(u8)]
pub enum Range {
    /// ±16g
    G16_1 = 0b001,

    /// ±16g
    G16_2 = 0b011,

    /// ±16g
    G16_3 = 0b101,

    /// ±16g available only in 14-bit mode
    G16_14Bit = 0b111,

    /// ±8g
    G8 = 0b100,

    /// ±8g available only in 14-bit mode
    G8_14Bit = 0b110,

    /// ±4g
    G4 = 0b010,

    /// ±2g (Default)
    G2 = 0b000,
}
impl Range {
    pub const fn bits(self) -> u8 {
        self as u8
    }
}

/// The output data rate for the wake-up function.
#[allow(dead_code, non_camel_case_types)]
#[derive(Copy, Clone, Debug, Eq, PartialEq, TryFromPrimitive)]
#[repr(u8)]
pub enum MotionDetectionDataRate {
    Hz_0_781 = 0b000,
    Hz_1_563 = 0b001,
    Hz_3_125 = 0b010,
    Hz_6_25 = 0b011,
    Hz_12_5 = 0b100,
    Hz_25 = 0b101,
    Hz_50 = 0b110,
    Hz_100 = 0b111,
}
impl MotionDetectionDataRate {
    pub const fn bits(self) -> u8 {
        self as u8
    }
    pub const fn sample_rate(self) -> f32 {
        match self {
            MotionDetectionDataRate::Hz_0_781 => 0.781,
            MotionDetectionDataRate::Hz_1_563 => 1.563,
            MotionDetectionDataRate::Hz_100 => 100.0,
            MotionDetectionDataRate::Hz_50 => 50.0,
            MotionDetectionDataRate::Hz_25 => 25.0,
            MotionDetectionDataRate::Hz_12_5 => 12.5,
            MotionDetectionDataRate::Hz_3_125 => 3.125,
            MotionDetectionDataRate::Hz_6_25 => 6.25,
        }
    }
}

///The axis and direction of motion.
#[allow(dead_code, non_camel_case_types)]
#[derive(Copy, Clone, Debug, Eq, PartialEq, TryFromPrimitive)]
#[repr(u8)]
pub enum MotionDetectionAxis {
    X_Negative = 0b0010_0000,
    X_Positive = 0b0001_0000,
    Y_Negative = 0b0000_1000,
    Y_Positive = 0b0000_0100,
    Z_Negative = 0b0000_0010,
    Z_Positive = 0b0000_0001,
    None = 0b0000_0000,
}
impl MotionDetectionAxis {
    pub const fn bits(self) -> u8 {
        self as u8
    }
}

///The latch mode of motion interrupt.
#[allow(dead_code, non_camel_case_types)]
#[derive(Copy, Clone, Debug, Eq, PartialEq, TryFromPrimitive)]
#[repr(u8)]
pub enum MotionDetectionLatchMode {
    Latched = 0,
    UnLatched = 1,
}
impl MotionDetectionLatchMode {
    pub const fn bits(self) -> u8 {
        self as u8
    }
}
///The polarity of the physical interrupt pin.
#[allow(dead_code, non_camel_case_types)]
#[derive(Copy, Clone, Debug, Eq, PartialEq, TryFromPrimitive)]
#[repr(u8)]
pub enum InterruptPinPolarity {
    ActiveLow,
    ActiveHigh,
}

///The response of the physical interrupt pin.
#[allow(dead_code, non_camel_case_types)]
#[derive(Copy, Clone, Debug, Eq, PartialEq, TryFromPrimitive)]
#[repr(u8)]
pub enum InterruptPinResponse {
    /// The physical interrupt pin (INT) latches until it is cleared by reading INT_REL
    Latched,
    /// The physical interrupt pin (INT) will transmit one pulse with a period of 0.03 - 0.05ms
    Pulsed,
}

// === WHO_AMI_I (0Fh) ===
///`WHO_AM_I` device identification register
pub(crate) const DEVICE_ID: u8 = 0x35;

// === CTRL_REG1 (1Bh) ===
pub(crate) const PC1_EN: u8 = 0b1000_0000;
pub(crate) const WUFE_EN: u8 = 0b0000_0010;
pub(crate) const DRDYE_EN: u8 = 0b0010_0000;
pub(crate) const RES_EN: u8 = 0b0100_0000;
pub(crate) const GSEL_MASK: u8 = 0b0001_1100;

// === CTRL_REG2 (1Dh) ===
pub(crate) const ODRW_MASK: u8 = 0b0000_0111;

// === DATA_CTRL_REG (21h) ===
pub(crate) const ODR_MASK: u8 = 0b0000_1111;

// === INT_CTRL_REG1(1Eh)  ====
pub(crate) const IEN_EN: u8 = 0b0010_0000;
pub(crate) const IEA_EN: u8 = 0b0001_0000;
pub(crate) const IEL_EN: u8 = 0b0000_1000;

// === INT_CTRL_REG2(1Fh)  ====
pub(crate) const ULMODE_EN: u8 = 0b1000_0000;
pub(crate) const WUE_MASK: u8 = 0b0011_1111;
