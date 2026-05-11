//! MCU application state container.

use crate::shutdown::McuState;
use heapless::LinearMap;

/// Number of stepper objects supported.
pub const NUM_STEPPERS: usize = 4;
/// Number of TrSync objects supported.
pub const NUM_TRSYNC: usize = 4;
/// Number of endstop objects supported.
pub const NUM_ENDSTOPS: usize = 4;

/// MCU application state.
///
/// Contains objects managed by the Klipper protocol that do NOT belong
/// in the TriggerDomain: stepper OID maps, digital output enable maps,
/// and MCU lifecycle state.
///
/// TrSync, endstop, and stepper-stop state have moved to the board
/// crate's `TriggerDomain` static (accessed via `with_trigger_domain()`).
pub struct State {
    /// Current MCU lifecycle state.
    pub mcu_state: McuState,
    /// Configuration CRC from finalize_config.
    pub config_crc: Option<u32>,
    /// Map from Klipper OID to stepper array index.
    pub steppers_by_oid: LinearMap<u8, usize, NUM_STEPPERS>,
    /// Map from digital output OID to stepper array index (for enable/disable).
    pub enable_by_oid: LinearMap<u8, usize, NUM_STEPPERS>,
    /// Channel derived from each configured stepper's virtual STEP/DIR pins.
    pub stepper_channels: [Option<u8>; NUM_STEPPERS],
    /// Channel derived from each configured digital output's virtual enable pin.
    pub enable_channels: [Option<u8>; NUM_STEPPERS],
}

impl State {
    /// Create a new state with all objects in their default configuration.
    pub fn new() -> Self {
        Self {
            mcu_state: McuState::Configuring,
            config_crc: None,
            steppers_by_oid: LinearMap::new(),
            enable_by_oid: LinearMap::new(),
            stepper_channels: [None; NUM_STEPPERS],
            enable_channels: [None; NUM_STEPPERS],
        }
    }

    /// Return the configured channel for a stepper OID.
    pub fn stepper_channel(&self, oid: u8) -> Option<u8> {
        let idx = *self.steppers_by_oid.get(&oid)?;
        self.stepper_channels[idx]
    }

    /// Return the configured channel for a digital-output enable OID.
    pub fn enable_channel(&self, oid: u8) -> Option<u8> {
        let idx = *self.enable_by_oid.get(&oid)?;
        self.enable_channels[idx]
    }

    /// Reset all state for reconfiguration (after shutdown recovery).
    pub fn reset(&mut self) {
        self.mcu_state = McuState::Configuring;
        self.config_crc = None;
        self.steppers_by_oid.clear();
        self.enable_by_oid.clear();
        self.stepper_channels = [None; NUM_STEPPERS];
        self.enable_channels = [None; NUM_STEPPERS];
    }
}

impl ankyra::ShutdownState for State {
    fn is_shutdown(&self) -> bool {
        self.mcu_state.is_shutdown()
    }
}

impl Default for State {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn new_state_is_configuring() {
        let state = State::new();
        assert_eq!(state.mcu_state, McuState::Configuring);
        assert!(state.config_crc.is_none());
    }

    #[test]
    fn reset_clears_everything() {
        let mut state = State::new();
        state.mcu_state = McuState::Shutdown { reason: 1 };
        state.config_crc = Some(0xDEAD);
        state.steppers_by_oid.insert(0, 0).unwrap();
        state.enable_by_oid.insert(1, 0).unwrap();
        state.stepper_channels[0] = Some(0);
        state.enable_channels[0] = Some(0);

        state.reset();

        assert_eq!(state.mcu_state, McuState::Configuring);
        assert!(state.config_crc.is_none());
        assert!(state.steppers_by_oid.is_empty());
        assert!(state.enable_by_oid.is_empty());
        assert_eq!(state.stepper_channels, [None; NUM_STEPPERS]);
        assert_eq!(state.enable_channels, [None; NUM_STEPPERS]);
    }

    #[test]
    fn default_is_same_as_new() {
        let a = State::new();
        let b = State::default();
        assert_eq!(a.mcu_state, b.mcu_state);
        assert_eq!(a.config_crc, b.config_crc);
    }

    #[test]
    fn shutdown_state_configuring() {
        use ankyra::ShutdownState;
        let state = State::new();
        assert!(!state.is_shutdown());
    }

    #[test]
    fn shutdown_state_running() {
        use ankyra::ShutdownState;
        let mut state = State::new();
        state.mcu_state = McuState::Running;
        assert!(!state.is_shutdown());
    }

    #[test]
    fn shutdown_state_shutdown() {
        use ankyra::ShutdownState;
        let mut state = State::new();
        state.mcu_state = McuState::Shutdown { reason: 0 };
        assert!(state.is_shutdown());
    }
}
