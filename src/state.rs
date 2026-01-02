//! MCU application state container.

use crate::shutdown::McuState;
use crate::stepper::EmulatedStepper;
use heapless::LinearMap;

/// Number of stepper objects supported.
pub const NUM_STEPPERS: usize = 2;

/// MCU application state.
///
/// Contains all objects managed by the Klipper protocol:
/// steppers, digital outputs, and configuration state.
pub struct State {
    /// Current MCU lifecycle state.
    pub mcu_state: McuState,
    /// Configuration CRC from finalize_config.
    pub config_crc: Option<u32>,
    /// Stepper motor objects.
    pub steppers: [EmulatedStepper; NUM_STEPPERS],
    /// Map from Klipper OID to stepper array index.
    pub steppers_by_oid: LinearMap<u8, usize, NUM_STEPPERS>,
    /// Map from digital output OID to stepper array index (for enable/disable).
    pub enable_by_oid: LinearMap<u8, usize, NUM_STEPPERS>,
}

impl State {
    /// Create a new state with all objects in their default configuration.
    pub fn new() -> Self {
        Self {
            mcu_state: McuState::Configuring,
            config_crc: None,
            steppers: core::array::from_fn(|_| EmulatedStepper::new()),
            steppers_by_oid: LinearMap::new(),
            enable_by_oid: LinearMap::new(),
        }
    }

    /// Look up a stepper by OID.
    pub fn stepper(&self, oid: u8) -> Option<&EmulatedStepper> {
        self.steppers_by_oid
            .get(&oid)
            .map(|&idx| &self.steppers[idx])
    }

    /// Look up a mutable stepper by OID.
    pub fn stepper_mut(&mut self, oid: u8) -> Option<&mut EmulatedStepper> {
        self.steppers_by_oid
            .get(&oid)
            .copied()
            .map(move |idx| &mut self.steppers[idx])
    }

    /// Reset all state for reconfiguration (after shutdown recovery).
    pub fn reset(&mut self) {
        self.mcu_state = McuState::Configuring;
        self.config_crc = None;
        self.steppers_by_oid.clear();
        self.enable_by_oid.clear();
        for s in &mut self.steppers {
            *s = EmulatedStepper::new();
        }
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
    fn stepper_oid_lookup() {
        let mut state = State::new();
        state.steppers_by_oid.insert(0, 0).unwrap();
        state.steppers_by_oid.insert(3, 1).unwrap();

        assert!(state.stepper(0).is_some());
        assert!(state.stepper(3).is_some());
        assert!(state.stepper(1).is_none());
    }

    #[test]
    fn stepper_mut_oid_lookup() {
        let mut state = State::new();
        state.steppers_by_oid.insert(5, 0).unwrap();

        let stepper = state.stepper_mut(5).unwrap();
        stepper.set_enabled(true);
        assert!(state.steppers[0].enabled());
    }

    #[test]
    fn reset_clears_everything() {
        let mut state = State::new();
        state.mcu_state = McuState::Shutdown { reason: 1 };
        state.config_crc = Some(0xDEAD);
        state.steppers_by_oid.insert(0, 0).unwrap();
        state.enable_by_oid.insert(1, 0).unwrap();
        state.steppers[0].set_enabled(true);

        state.reset();

        assert_eq!(state.mcu_state, McuState::Configuring);
        assert!(state.config_crc.is_none());
        assert!(state.steppers_by_oid.is_empty());
        assert!(state.enable_by_oid.is_empty());
        assert!(!state.steppers[0].enabled());
    }

    #[test]
    fn default_is_same_as_new() {
        let a = State::new();
        let b = State::default();
        assert_eq!(a.mcu_state, b.mcu_state);
        assert_eq!(a.config_crc, b.config_crc);
    }
}
