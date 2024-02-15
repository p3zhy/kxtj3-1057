//! This module provides definitions for configuring the accelerometer sensor.

use crate::register::*;

/// Sensor configuration options
#[derive(Debug, Clone, Copy)]
pub struct Configuration {
    /// The operating mode, default [`Mode::HighResolution`].
    pub mode: Mode,
    /// The output data rate, default [`DataRate::Hz_6_25`].
    pub datarate: DataRate,
    /// The output acceleration range , default [`Range::G2`].
    pub range: Range,
    /// The reporting of the availability of new acceleration data as an interrupt , default [`false`]
    pub enable_new_acceleration_interrupt: bool,
    /// The Wake-Up (motion detect) function, default [`None`]
    pub motion_detection: Option<MotionDetctionConfiguration>,
}

impl Default for Configuration {
    fn default() -> Self {
        Self {
            mode: Mode::HighResolution,
            datarate: DataRate::Hz_6_25,
            range: Range::G2,
            enable_new_acceleration_interrupt: false,
            motion_detection: None,
        }
    }
}

/// Motion detction configuration options
#[derive(Debug, Clone, Copy)]
pub struct MotionDetctionConfiguration {
    /// The Output Data Rate for the Wake-Up function (motion detection)  , default [`MotionDetectionDataRate::Hz_6_25`]
    pub datarate: MotionDetectionDataRate,
    /// The latche mode of motion interrupt, default [`MotionDetectionLatchMode::Latched`] (clears after INT_REL is read)
    pub latch_mode: MotionDetectionLatchMode,
    /// The non-activity time required before another wake-up interrupt can be set , default `10` (1.6s)
    pub non_activity_counter: u8,
    /// The time motion must be present before a wake-up interrupt is set , default `1` (160ms)
    pub wakeup_counter: u8,
    /// The threshold for wake-up (motion detect) interrupt is set , default `128` (0.5g)
    pub wakeup_threshold: u8,
    /// The physical interrupt pin , default [`None`]
    pub interrupt_pin: Option<InterruptPinConfiguration>,
    /// The X- can cause an interrupt or not , default [`true`]
    pub enable_x_negative: bool,
    /// The X+ can cause an interrupt or not , default [`true`]
    pub enable_x_positive: bool,
    /// The Y- can cause an interrupt or not , default [`true`]
    pub enable_y_negative: bool,
    /// The Y+ can cause an interrupt or not , default [`true`]
    pub enable_y_positive: bool,
    /// The Z- can cause an interrupt or not , default [`true`]
    pub enable_z_negative: bool,
    /// The Z+ can cause an interrupt or not , default [`true`]
    pub enable_z_positive: bool,
}

impl Default for MotionDetctionConfiguration {
    fn default() -> Self {
        Self {
            datarate: MotionDetectionDataRate::Hz_6_25,
            latch_mode: MotionDetectionLatchMode::Latched,
            non_activity_counter: 10,
            wakeup_counter: 1,
            wakeup_threshold: 40,
            interrupt_pin: None,
            enable_x_negative: true,
            enable_x_positive: true,
            enable_y_negative: true,
            enable_y_positive: true,
            enable_z_negative: true,
            enable_z_positive: true,
        }
    }
}

///Physical interrupt pin configuration options
#[derive(Debug, Clone, Copy)]
pub struct InterruptPinConfiguration {
    /// The polarity of the physical interrupt pin , default [`InterruptPinPolarity::ActiveLow`]
    pub polarity: InterruptPinPolarity,
    /// The response of the physical interrupt pin , default [`InterruptPinResponse::Latched`]
    pub response: InterruptPinResponse,
}

impl Default for InterruptPinConfiguration {
    fn default() -> Self {
        Self {
            polarity: InterruptPinPolarity::ActiveLow,
            response: InterruptPinResponse::Latched,
        }
    }
}
