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

// === WHO_AMI_I (0Fh) ===
/// `WHO_AM_I` device identification register
pub const DEVICE_ID: u8 = 0x35;

// === CTRL_REG1 (20h) ===
pub const PC1_EN: u8 = 0b1000_0000;
pub const RES_EN: u8 = 0b0100_0000;
/// Operating mode.
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
