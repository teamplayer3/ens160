// #![allow(incomplete_features)]
// #![feature(generic_const_exprs)]
#![cfg_attr(not(feature = "std"), no_std)]

pub mod error;

use core::{
    convert::TryFrom,
    ops::{Deref, DerefMut},
};

#[cfg(not(feature = "async"))]
use embedded_hal::i2c::I2c;
#[cfg(feature = "async")]
use embedded_hal_async::i2c::I2c as AsyncI2c;

use bitfield::bitfield;
use error::AirQualityConvError;

// ENS160 Register address
// This 2-byte register contains the part number in little endian of the ENS160.
const ENS160_PART_ID_REG: u8 = 0x00;
// This 1-byte register sets the Operating Mode of the ENS160.
const ENS160_OPMODE_REG: u8 = 0x10;
// This 1-byte register configures the action of the INTn pin.
const ENS160_CONFIG_REG: u8 = 0x11;
// This 1-byte register allows some additional commands to be executed on the ENS160.
#[allow(dead_code)]
const ENS160_COMMAND_REG: u8 = 0x12;
// This 2-byte register allows the host system to write ambient temperature data to ENS160 for compensation.
const ENS160_TEMP_IN_REG: u8 = 0x13;
// This 2-byte register allows the host system to write relative humidity data to ENS160 for compensation.
#[allow(dead_code)]
const ENS160_RH_IN_REG: u8 = 0x15;
// This 1-byte register indicates the current STATUS of the ENS160.
const ENS160_DATA_STATUS_REG: u8 = 0x20;
// This 1-byte register reports the calculated Air Quality Index according to the UBA.
const ENS160_DATA_AQI_REG: u8 = 0x21;
// This 2-byte register reports the calculated TVOC concentration in ppb.
const ENS160_DATA_TVOC_REG: u8 = 0x22;
// This 2-byte register reports the calculated equivalent CO2-concentration in ppm, based on the detected VOCs and hydrogen.
const ENS160_DATA_ECO2_REG: u8 = 0x24;
// This 2-byte register reports the temperature used in its calculations (taken from TEMP_IN, if supplied).
const ENS160_DATA_T_REG: u8 = 0x30;
// This 2-byte register reports the relative humidity used in its calculations (taken from RH_IN if supplied).
#[allow(dead_code)]
const ENS160_DATA_RH_REG: u8 = 0x32;
// This 1-byte register reports the calculated checksum of the previous DATA_ read transaction (of n-bytes).
#[allow(dead_code)]
const ENS160_DATA_MISR_REG: u8 = 0x38;
// This 8-byte register is used by several functions for the Host System to pass data to the ENS160.
#[allow(dead_code)]
const ENS160_GPR_WRITE_REG: u8 = 0x40;
// This 8-byte register is used by several functions for the ENS160 to pass data to the Host System.
#[allow(dead_code)]
const ENS160_GPR_READ_REG: u8 = 0x48;

/// A driver for the `ENS160` sensor connected with I2C to the host.
pub struct Ens160<I2C> {
    i2c: I2C,
    address: u8,
}

impl<I2C> Ens160<I2C> {
    /// Creates a new sensor driver.
    pub fn new(i2c: I2C, address: u8) -> Self {
        Self { i2c, address }
    }

    /// Releases the underlying I2C bus and destroys the driver.
    pub fn release(self) -> I2C {
        self.i2c
    }
}

#[maybe_async_cfg::maybe(
    sync(
        cfg(not(feature = "async")),
        self = "Ens160",
        idents(AsyncI2c(sync = "I2c"))
    ),
    async(feature = "async", keep_self)
)]

impl<I2C, E> Ens160<I2C>
where
    I2C: AsyncI2c<Error = E>,
{
    /// Resets the device.
    pub async fn reset(&mut self) -> Result<(), E> {
        self.write_register([ENS160_OPMODE_REG, OperationMode::Reset as u8])
            .await
    }

    /// Switches the device to idle mode.
    ///
    /// Only in idle mode operations with `ENS160_COMMAND_REG` can be performed.
    pub async fn idle(&mut self) -> Result<(), E> {
        self.write_register([ENS160_OPMODE_REG, OperationMode::Idle as u8])
            .await
    }

    /// Switches the device to deep sleep mode.
    ///
    /// This function can be used to conserve power when the device is not in use.
    pub async fn deep_sleep(&mut self) -> Result<(), E> {
        self.write_register([ENS160_OPMODE_REG, OperationMode::Sleep as u8])
            .await
    }

    /// Switches the device to operational mode.
    ///
    /// Call this function when you want the device to start taking measurements.
    pub async fn operational(&mut self) -> Result<(), E> {
        self.write_register([ENS160_OPMODE_REG, OperationMode::Standard as u8])
            .await
    }

    /// Clears the command register of the device.
    pub async fn clear_command(&mut self) -> Result<(), E> {
        self.write_register([ENS160_COMMAND_REG, Command::Nop as u8])
            .await?;
        self.write_register([ENS160_COMMAND_REG, Command::Clear as u8])
            .await?;
        Ok(())
    }

    /// Returns the part ID of the sensor.
    pub async fn part_id(&mut self) -> Result<u16, E> {
        self.read_register::<2>(ENS160_PART_ID_REG)
            .await
            .map(u16::from_le_bytes)
    }

    /// Returns the firmware version of the sensor.
    pub async fn firmware_version(&mut self) -> Result<(u8, u8, u8), E> {
        self.write_register([ENS160_COMMAND_REG, Command::GetAppVersion as u8])
            .await?;
        let buffer = self.read_register::<3>(ENS160_GPR_READ_REG).await?;
        Ok((buffer[0], buffer[1], buffer[2]))
    }

    /// Returns the current status of the sensor.
    pub async fn status(&mut self) -> Result<Status, E> {
        self.read_register::<1>(ENS160_DATA_STATUS_REG)
            .await
            .map(|v| Status(v[0]))
    }

    /// Returns the current Air Quality Index (AQI) reading from the sensor.
    ///
    /// The AQI is calculated based on the current sensor readings.
    pub async fn air_quality_index(&mut self) -> Result<AirQualityIndex, E> {
        self.read_register::<1>(ENS160_DATA_AQI_REG)
            .await
            .map(|v| AirQualityIndex::from(v[0] & 0x07))
    }

    /// Returns the Total Volatile Organic Compounds (TVOC) measurement from the sensor.
    ///
    /// The TVOC level is expressed in parts per billion (ppb) in the range 0-65000.
    pub async fn tvoc(&mut self) -> Result<u16, E> {
        self.read_register::<2>(ENS160_DATA_TVOC_REG)
            .await
            .map(u16::from_le_bytes)
    }

    /// Returns the Equivalent Carbon Dioxide (eCO2) measurement from the sensor.
    ///
    /// The eCO2 level is expressed in parts per million (ppm) in the range 400-65000.
    pub async fn eco2(&mut self) -> Result<ECo2, E> {
        self.read_register::<2>(ENS160_DATA_ECO2_REG)
            .await
            .map(u16::from_le_bytes)
            .map(ECo2::from)
    }

    /// Returns the temperature (in °C) and relative humidity (in %) values used in the calculations.
    ///
    /// The units are scaled by 100. For example, a temperature value of 2550 represents 25.50 °C,
    /// and a humidity value of 5025 represents 50.25% RH.
    ///
    /// These values can be set using [`Ens160::set_temp_and_hum()`].
    pub async fn temp_and_hum(&mut self) -> Result<(i16, u16), E> {
        let buffer = self.read_register::<4>(ENS160_DATA_T_REG).await?;
        let temp = u16::from_le_bytes([buffer[0], buffer[1]]);
        let rh = u16::from_le_bytes([buffer[2], buffer[3]]);

        let temp = temp as i32 * 100 / 64 - 27315;
        let hum = rh as u32 * 100 / 512;

        Ok((temp as i16, hum as u16))
    }

    /// Sets the temperature value used in the device's calculations.
    ///
    /// Unit is scaled by 100. For example, a temperature value of 2550 should be used for 25.50 °C.
    pub async fn set_temp(&mut self, ambient_temp: i16) -> Result<(), E> {
        let temp = ((ambient_temp as i32 + 27315) * 64 / 100) as u16;
        let temp = temp.to_le_bytes();
        let tbuffer = [ENS160_TEMP_IN_REG, temp[0], temp[1]];
        self.write_register(tbuffer).await
    }

    /// Sets the relative humidity value used in the device's calculations.
    ///
    /// Unit is scaled by 100. For example, a humidity value of 5025 should be used for 50.25% RH.
    pub async fn set_hum(&mut self, relative_humidity: u16) -> Result<(), E> {
        let rh = (relative_humidity as u32 * 512 / 100) as u16;
        let rh = rh.to_le_bytes();
        let hbuffer = [ENS160_RH_IN_REG, rh[0], rh[1]];
        self.write_register(hbuffer).await
    }

    /// Sets interrupt configuration.
    pub async fn set_interrupt_config(&mut self, config: InterruptConfig) -> Result<(), E> {
        self.write_register([ENS160_CONFIG_REG, config.finish().0])
            .await
    }

    async fn read_register<const N: usize>(&mut self, register: u8) -> Result<[u8; N], E> {
        let mut write_buffer = [0u8; 1];
        write_buffer[0] = register;
        let mut buffer = [0u8; N];
        self.i2c
            .write_read(self.address, &write_buffer, &mut buffer)
            .await?;
        Ok(buffer)
    }

    async fn write_register<const N: usize>(&mut self, buffer: [u8; N]) -> Result<(), E> {
        self.i2c.write(self.address, &buffer).await
    }
}

/// Commands for ENS160_COMMAND_REG.
#[repr(u8)]
enum Command {
    /// No operation
    Nop = 0x00,
    /// Get FW version
    GetAppVersion = 0x0E,
    /// Clears GPR Read Registers
    Clear = 0xCC,
}

/// Operation Mode of the sensor.
#[repr(u8)]
enum OperationMode {
    /// DEEP SLEEP mode (low power standby).
    Sleep = 0x00,
    /// IDLE mode (low-power).
    Idle = 0x01,
    /// STANDARD Gas Sensing Modes.
    Standard = 0x02,
    /// Reset device.
    Reset = 0xF0,
}

bitfield! {
    /// Status of the sensor.
    pub struct Status(u8);
    impl Debug;
    pub bool, running_normally, _: 7;
    pub bool, error, _: 6;
    pub into Validity, validity_flag, _: 3,2;
    pub bool, data_is_ready, _: 1;
    pub bool, new_data_in_gpr, _: 0;
}

// #[derive(BitfieldSpecifier)]
#[derive(Debug, Clone, Copy)]
pub enum Validity {
    NormalOperation,
    WarmupPhase,
    InitStartupPhase,
    InvalidOutput,
}

impl From<u8> for Validity {
    fn from(v: u8) -> Self {
        match v {
            0b00 => Self::NormalOperation,
            0b01 => Self::WarmupPhase,
            0b10 => Self::InitStartupPhase,
            0b11 => Self::InvalidOutput,
            _ => unreachable!(),
        }
    }
}

bitfield! {
    #[derive(Default)]
    struct InterruptRegister(u8);
    impl Debug;
    from into InterruptState, _, set_interrupt_state: 6, 6;
    from into PinMode,  ddfsdf, set_pin_mode: 5, 5;
    bool,  _, set_on_data_in_gpr_register: 3;
    bool,  _, set_on_data_in_data_register: 1;
    bool,  _, set_enabled: 0;
}

// #[derive(BitfieldSpecifier)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PinMode {
    OpenDrain,
    PushPull,
}

impl From<PinMode> for u8 {
    fn from(m: PinMode) -> u8 {
        match m {
            PinMode::OpenDrain => 0x0,
            PinMode::PushPull => 0x1,
        }
    }
}

impl From<u8> for PinMode {
    fn from(b: u8) -> Self {
        match b {
            0x1 => Self::PushPull,
            _ => Self::OpenDrain,
        }
    }
}

// #[derive(BitfieldSpecifier)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InterruptState {
    ActiveLow,
    ActiveHigh,
}

impl From<InterruptState> for u8 {
    fn from(i: InterruptState) -> u8 {
        match i {
            InterruptState::ActiveHigh => 0x1,
            InterruptState::ActiveLow => 0x0,
        }
    }
}

impl From<u8> for InterruptState {
    fn from(b: u8) -> Self {
        match b {
            0x1 => Self::ActiveHigh,
            _ => Self::ActiveLow,
        }
    }
}

#[derive(Debug, Default)]
pub struct InterruptConfig(InterruptRegister);

impl InterruptConfig {
    pub fn set_pin_interrupt_state(mut self, state: InterruptState) -> Self {
        self.0.set_interrupt_state(state);
        self
    }

    pub fn enable_for_measure_data_is_ready(mut self) -> Self {
        self.0.set_enabled(true);
        self.0.set_on_data_in_data_register(true);
        self
    }

    pub fn enable_for_data_in_read_register(mut self) -> Self {
        self.0.set_enabled(true);
        self.0.set_on_data_in_gpr_register(true);
        self
    }

    pub fn set_pin_mode(mut self, mode: PinMode) -> Self {
        self.0.set_pin_mode(mode);
        self
    }

    fn finish(self) -> InterruptRegister {
        self.0
    }
}

#[derive(Debug, Clone, Copy, Hash, PartialEq, Eq, PartialOrd, Ord)]
#[repr(u8)]
pub enum AirQualityIndex {
    Excellent = 1,
    Good = 2,
    Moderate = 3,
    Poor = 4,
    Unhealthy = 5,
}

impl From<u8> for AirQualityIndex {
    fn from(i: u8) -> Self {
        match i {
            1 => Self::Excellent,
            2 => Self::Good,
            3 => Self::Moderate,
            4 => Self::Poor,
            5 => Self::Unhealthy,
            _ => Self::Unhealthy,
        }
    }
}

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, PartialOrd, Ord)]
pub struct ECo2(u16);

impl From<u16> for ECo2 {
    fn from(v: u16) -> Self {
        Self(v)
    }
}

impl TryFrom<ECo2> for AirQualityIndex {
    type Error = AirQualityConvError;

    fn try_from(e: ECo2) -> Result<Self, Self::Error> {
        let value = e.0;
        match value {
            400..=599 => Ok(Self::Excellent),
            600..=799 => Ok(Self::Good),
            800..=999 => Ok(Self::Moderate),
            1000..=1499 => Ok(Self::Poor),
            1500..=u16::MAX => Ok(Self::Unhealthy),
            _ => Err(AirQualityConvError(value)),
        }
    }
}

impl Deref for ECo2 {
    type Target = u16;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl DerefMut for ECo2 {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

#[cfg(test)]
mod test {

    use crate::{InterruptConfig, PinMode, Status, Validity};

    #[test]
    fn test_status_register_layout() {
        let status = Status(0b00000001);
        assert!(status.new_data_in_gpr());

        let status = Status(0b10000100);
        assert!(status.running_normally());
        assert!(matches!(status.validity_flag(), Validity::WarmupPhase));

        let status = Status(0b00001110);
        assert!(status.data_is_ready());
        assert!(matches!(status.validity_flag(), Validity::InvalidOutput))
    }

    #[test]
    fn test_interrupt_config() {
        let config = InterruptConfig::default()
            .enable_for_measure_data_is_ready()
            .set_pin_mode(PinMode::PushPull)
            .finish();
        assert_eq!(config.0, 0b00100011)
    }

    #[test]
    fn test_byte_order() {
        let b: u16 = 0x10;
        assert_eq!(b.to_be_bytes(), [0x0, 0x10])
    }
}
