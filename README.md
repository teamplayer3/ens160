# ENS160 driver

[![Build Status](https://github.com/teamplayer3/ens160/workflows/Rust/badge.svg)](https://github.com/teamplayer3/ens160/actions?query=workflow%3ARust)
[![License](https://img.shields.io/badge/license-MIT%2FApache--2.0-blue.svg)](https://github.com/teamplayer3/ens160)
[![Crates.io](https://img.shields.io/crates/v/ens160.svg)](https://crates.io/crates/ens160)
[![Documentation](https://docs.rs/ens160/badge.svg)](https://docs.rs/ens160)

Implementation of an ENS160 sensor driver written in rust. This sensor can measure `TVOC` and `ECO2`. Implementation is inspired by the [driver implementation](https://github.com/DFRobot/DFRobot_ENS160) by dfrobots written in python.

For more information about the sensor [here](https://wiki.dfrobot.com/SKU_SEN0515_Fermion_ENS160_Air_Quality_Sensor).

## Example

```rust
let i2c = ...; // I2C bus to use

let mut device = Ens160::new(i2c);
let tvoc = device.get_tvoc().unwrap();
let eco2 = device.get_eco2().unwrap();

let airquality_index = AirqualityIndex::try_from(eco2).unwrap();

let i2c = device.release(); // destruct driver to use bus with other drivers
```