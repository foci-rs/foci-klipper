//! Digital output (motor enable/disable) commands.

use crate::commands::CommandError;
use crate::state::{State, NUM_STEPPERS};

/// Register a digital output object for motor enable/disable.
///
/// Pin parameters are accepted by the firmware wrapper for protocol
/// compatibility but not passed here.
pub fn config_digital_out(state: &mut State, oid: u8) -> Result<(), CommandError> {
    if state.enable_by_oid.contains_key(&oid) {
        return Err(CommandError::DuplicateOid(oid));
    }
    let idx = state.enable_by_oid.len();
    if idx >= NUM_STEPPERS {
        return Err(CommandError::SlotsFull);
    }
    state
        .enable_by_oid
        .insert(oid, idx)
        .map_err(|_| CommandError::SlotsFull)?;
    Ok(())
}

/// Schedule a digital output change (motor enable/disable).
///
/// Returns `Ok(enabled)` where `enabled` indicates whether the motor
/// should be turned on (`true`) or off (`false`). The firmware wrapper
/// sends the hardware enable/disable command based on this value.
pub fn queue_digital_out(state: &State, oid: u8, on_ticks: u32) -> Result<bool, CommandError> {
    if !state.enable_by_oid.contains_key(&oid) {
        return Err(CommandError::UnknownOid(oid));
    }
    Ok(on_ticks != 0)
}

/// Immediately update a digital output value.
///
/// Returns `Ok(enabled)` for the firmware wrapper to send the hardware
/// enable/disable command.
pub fn update_digital_out(state: &State, oid: u8, value: u8) -> Result<bool, CommandError> {
    if !state.enable_by_oid.contains_key(&oid) {
        return Err(CommandError::UnknownOid(oid));
    }
    Ok(value != 0)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn config_digital_out_success() {
        let mut state = State::new();
        assert!(config_digital_out(&mut state, 5).is_ok());
    }

    #[test]
    fn config_digital_out_slots_full() {
        let mut state = State::new();
        for i in 0..NUM_STEPPERS as u8 {
            config_digital_out(&mut state, i).unwrap();
        }
        assert_eq!(
            config_digital_out(&mut state, 99),
            Err(CommandError::SlotsFull)
        );
    }

    #[test]
    fn queue_digital_out_enable() {
        let mut state = State::new();
        config_digital_out(&mut state, 10).unwrap();
        assert_eq!(queue_digital_out(&state, 10, 1), Ok(true));
    }

    #[test]
    fn queue_digital_out_disable() {
        let mut state = State::new();
        config_digital_out(&mut state, 10).unwrap();
        assert_eq!(queue_digital_out(&state, 10, 0), Ok(false));
    }

    #[test]
    fn queue_digital_out_unknown_oid() {
        let state = State::new();
        assert_eq!(
            queue_digital_out(&state, 99, 1),
            Err(CommandError::UnknownOid(99))
        );
    }

    #[test]
    fn update_digital_out_enable() {
        let mut state = State::new();
        config_digital_out(&mut state, 10).unwrap();
        assert_eq!(update_digital_out(&state, 10, 1), Ok(true));
    }

    #[test]
    fn update_digital_out_unknown_oid() {
        let state = State::new();
        assert_eq!(
            update_digital_out(&state, 99, 1),
            Err(CommandError::UnknownOid(99))
        );
    }
}
