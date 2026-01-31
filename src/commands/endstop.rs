//! Endstop registration and homing commands.
//!
//! GPIO pin configuration and reading are firmware responsibilities.
//! These functions handle OID registration and state machine delegation.

use crate::commands::CommandError;
use crate::state::{State, NUM_ENDSTOPS};

/// Register an endstop object by OID.
///
/// GPIO pin configuration is handled by the firmware wrapper
/// (board-specific PAC register access). This function only registers
/// the OID mapping.
pub fn config_endstop(state: &mut State, oid: u8) -> Result<(), CommandError> {
    if state.endstop_by_oid.contains_key(&oid) {
        return Err(CommandError::DuplicateOid(oid));
    }
    let idx = state.endstop_by_oid.len();
    if idx >= NUM_ENDSTOPS {
        return Err(CommandError::SlotsFull);
    }
    state
        .endstop_by_oid
        .insert(oid, idx)
        .map_err(|_| CommandError::SlotsFull)?;
    Ok(())
}

/// Arm an endstop for homing.
///
/// Configures sampling parameters and links to a TrSync for trigger
/// notification. Actual GPIO sampling is done by the firmware's
/// polling loop.
#[allow(clippy::too_many_arguments)]
pub fn endstop_home(
    state: &mut State,
    oid: u8,
    clock: u32,
    sample_ticks: u32,
    sample_count: u8,
    rest_ticks: u32,
    pin_value: bool,
    trsync_oid: u8,
    trigger_reason: u8,
) {
    if let Some(endstop) = state.endstop_mut(oid) {
        endstop.start_homing(
            clock,
            sample_ticks,
            sample_count,
            rest_ticks,
            pin_value,
            trsync_oid,
            trigger_reason,
        );
    }
}

/// Query the state of an endstop.
///
/// Returns `(homing, next_clock)`. The firmware wrapper adds the live
/// GPIO pin state for the full reply.
pub fn endstop_query_state(state: &State, oid: u8) -> (bool, u32) {
    state
        .endstop(oid)
        .map(|e| e.query_state())
        .unwrap_or((false, 0))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn config_endstop_success() {
        let mut state = State::new();
        assert!(config_endstop(&mut state, 20).is_ok());
        assert!(state.endstop(20).is_some());
    }

    #[test]
    fn config_endstop_slots_full() {
        let mut state = State::new();
        for i in 0..NUM_ENDSTOPS as u8 {
            config_endstop(&mut state, i).unwrap();
        }
        assert_eq!(config_endstop(&mut state, 99), Err(CommandError::SlotsFull));
    }

    #[test]
    fn endstop_home_arms() {
        let mut state = State::new();
        config_endstop(&mut state, 20).unwrap();
        endstop_home(&mut state, 20, 1000, 500, 3, 100, true, 10, 1);
        assert!(state.endstop(20).unwrap().is_homing());
    }

    #[test]
    fn endstop_home_unknown_oid_is_noop() {
        let mut state = State::new();
        endstop_home(&mut state, 99, 1000, 500, 3, 100, true, 10, 1);
    }

    #[test]
    fn endstop_query_state_not_homing() {
        let mut state = State::new();
        config_endstop(&mut state, 20).unwrap();
        let (homing, _) = endstop_query_state(&state, 20);
        assert!(!homing);
    }

    #[test]
    fn endstop_query_state_homing() {
        let mut state = State::new();
        config_endstop(&mut state, 20).unwrap();
        endstop_home(&mut state, 20, 1000, 500, 3, 100, true, 10, 1);
        let (homing, next_clock) = endstop_query_state(&state, 20);
        assert!(homing);
        assert_eq!(next_clock, 1000);
    }

    #[test]
    fn endstop_query_state_unknown_oid() {
        let state = State::new();
        let (homing, next_clock) = endstop_query_state(&state, 99);
        assert!(!homing);
        assert_eq!(next_clock, 0);
    }
}
