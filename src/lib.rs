#![allow(incomplete_features)]
#![feature(generic_const_exprs)]
#![cfg_attr(not(feature = "std"), no_std)]

pub mod error;

use core::{
    convert::TryFrom,
    ops::{Deref, DerefMut},
};

use embedded_hal::blocking::i2c::{Write, WriteRead};

use bitfield::bitfield;
use error::AirqualityConvError;

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

impl<I2C, E> Ens160<I2C>
where
    I2C: WriteRead<Error = E> + Write<Error = E>,
{
    /// Returns the sensors part id.
    pub fn get_part_id(&mut self) -> Result<u16, E> {
        self.read_register::<2>(ENS160_PART_ID_REG)
            .map(u16::from_be_bytes)
    }

    /// Returns the current status of the sensor.
    pub fn get_status(&mut self) -> Result<Status, E> {
        self.read_register::<1>(ENS160_DATA_STATUS_REG)
            .map(|v| Status(v[0]))
    }

    /// Returns an index of airquality related to the current measures.
    pub fn get_airquality_index(&mut self) -> Result<AirqualityIndex, E> {
        self.read_register::<1>(ENS160_DATA_AQI_REG)
            .map(|v| v[0].into())
    }

    /// Get tvoc measures in `ppb` in range `0-65000`.
    pub fn get_tvoc(&mut self) -> Result<u16, E> {
        self.read_register::<2>(ENS160_DATA_TVOC_REG)
            .map(u16::from_be_bytes)
    }

    /// Get eco2 measures in `ppm` in range `400-65000`.
    pub fn get_eco2(&mut self) -> Result<ECo2, E> {
        self.read_register::<2>(ENS160_DATA_ECO2_REG)
            .map(u16::from_be_bytes)
            .map(ECo2::from)
    }

    /// Returns the previous set temperature and humidity.
    ///
    /// Values can be set with [`Ens160::set_temp_and_hum()`].
    pub fn get_temp_and_hum(&mut self) -> Result<(f32, f32), E> {
        let buffer = self.read_register::<4>(ENS160_DATA_T_REG)?;
        let temp = u16::from_be_bytes([buffer[0], buffer[1]]);
        let rh = u16::from_be_bytes([buffer[2], buffer[3]]);

        let temp = ((temp as f32) - 0.5) / 64.0 - 273.15;
        let hum = ((rh as f32) - 0.5) / 512.0;

        Ok((temp, hum))
    }

    /// Sets the opration mode of the sensor.
    pub fn set_operation_mode(&mut self, mode: OperationMode) -> Result<(), E> {
        self.write_register::<1>(ENS160_OPMODE_REG, [mode as u8])
    }

    /// Sets the interrupt configuration.
    pub fn set_interrupt_config(&mut self, config: InterruptConfig) -> Result<(), E> {
        self.write_register(ENS160_CONFIG_REG, [config.finish().0])
    }

    /// Sets the temperature (°C) and humidity (%rh (relative Humidity in percent)) for calculations.
    pub fn set_temp_and_hum(&mut self, ambient_temp: f32, relative_humidity: f32) -> Result<(), E> {
        let temp = ((ambient_temp + 273.15) * 64.0 + 0.5) as u16;
        let rh = (relative_humidity * 512.0 + 0.5) as u16;

        let temp = temp.to_le_bytes();
        let rh = rh.to_le_bytes();

        let buffer = [temp[0], temp[1], rh[0], rh[1]];
        self.write_register(ENS160_TEMP_IN_REG, buffer)
    }

    fn read_register<const N: usize>(&mut self, register: u8) -> Result<[u8; N], E> {
        let mut write_buffer = [0u8; N];
        write_buffer[0] = register;
        let mut buffer = [0u8; N];
        self.i2c
            .write_read(self.address, &write_buffer, &mut buffer)?;
        Ok(buffer)
    }

    fn write_register<const N: usize>(&mut self, register: u8, data: [u8; N]) -> Result<(), E>
    where
        [(); N + 1]: Sized,
    {
        let mut final_buffer = [0u8; (N + 1)];
        final_buffer[0] = register;
        final_buffer[1..].copy_from_slice(&data);
        self.i2c.write(self.address, &final_buffer)
    }
}

/// Operation Mode of the sensor.
#[repr(u8)]
pub enum OperationMode {
    /// DEEP SLEEP mode (low power standby).
    Sleep = 0x00,
    /// IDLE mode (low-power).
    Idle = 0x01,
    /// STANDARD Gas Sensing Modes.
    Standard = 0x02,
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

#[derive(Debug, Clone, Copy, Hash, PartialEq, Eq)]
#[repr(u8)]
pub enum AirqualityIndex {
    Excellent = 0x1,
    Good = 0x2,
    Moderate = 0x3,
    Poor = 0x4,
    Unhealthy = 0x5,
}

impl PartialOrd for AirqualityIndex {
    fn partial_cmp(&self, other: &Self) -> Option<core::cmp::Ordering> {
        use core::cmp::Ordering;
        let self_index = *self as u8;
        let other_index = *other as u8;
        Some(match self_index.cmp(&other_index) {
            Ordering::Less => Ordering::Greater,
            Ordering::Greater => Ordering::Less,
            _ => Ordering::Equal,
        })
    }
}

impl Ord for AirqualityIndex {
    fn cmp(&self, other: &Self) -> core::cmp::Ordering {
        self.partial_cmp(other).unwrap()
    }
}

impl From<u8> for AirqualityIndex {
    fn from(i: u8) -> Self {
        match i {
            0x01 => Self::Excellent,
            0x02 => Self::Good,
            0x03 => Self::Moderate,
            0x04 => Self::Poor,
            0x05 => Self::Unhealthy,
            _ => unimplemented!(),
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

impl TryFrom<ECo2> for AirqualityIndex {
    type Error = AirqualityConvError;

    fn try_from(e: ECo2) -> Result<Self, Self::Error> {
        let value = e.0;
        match value {
            400..=599 => Ok(Self::Excellent),
            600..=799 => Ok(Self::Good),
            800..=999 => Ok(Self::Moderate),
            1000..=1499 => Ok(Self::Poor),
            1500..=u16::MAX => Ok(Self::Unhealthy),
            _ => Err(AirqualityConvError(value)),
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
