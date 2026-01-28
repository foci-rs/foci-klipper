//! SPI stub commands (TMC4671 SPI is internal).
//!
//! These exist because Klipper expects SPI commands in the data dictionary.
//! They are all no-ops since the TMC4671 SPI bus is managed directly by
//! the firmware, not exposed to the host.

/// SPI configuration stub.
pub fn config_spi(_oid: u8, _pin: u32, _cs_active_high: u8) {}

/// SPI configuration without chip-select stub.
pub fn config_spi_without_cs(_oid: u8) {}

/// SPI bus assignment stub.
pub fn spi_set_bus(_oid: u8, _spi_bus: u32, _mode: u32, _rate: u32) {}

/// SPI shutdown message configuration stub.
pub fn config_spi_shutdown(_oid: u8, _spi_oid: u8, _shutdown_msg: &[u8]) {}

/// SPI send stub (data is accepted but not forwarded).
pub fn spi_send(_oid: u8, _data: &[u8]) {}
