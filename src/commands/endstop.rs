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
) -> Result<(), CommandError> {
    if !state.endstop_by_oid.contains_key(&oid) {
        return Err(CommandError::UnknownOid(oid));
    }
    // A zero sample_count is a disarm request. It does not arm sampling and
    // therefore does not require a TrSync to be configured yet.
    if sample_count != 0 && !state.trsync_by_oid.contains_key(&trsync_oid) {
        return Err(CommandError::UnknownOid(trsync_oid));
    }
    let endstop = state.endstop_mut(oid).expect("validated above");
    endstop.start_homing(
        clock,
        sample_ticks,
        sample_count,
        rest_ticks,
        pin_value,
        trsync_oid,
        trigger_reason,
    );
    Ok(())
}

/// Query the state of an endstop.
///
/// Returns `(homing, next_clock)`. The firmware wrapper adds the live
/// GPIO pin state for the full reply.
pub fn endstop_query_state(state: &State, oid: u8) -> Result<(bool, u32), CommandError> {
    let endstop = state.endstop(oid).ok_or(CommandError::UnknownOid(oid))?;
    Ok(endstop.query_state())
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
        use crate::commands::trsync::config_trsync;
        let mut state = State::new();
        config_endstop(&mut state, 20).unwrap();
        config_trsync(&mut state, 10).unwrap();
        endstop_home(&mut state, 20, 1000, 500, 3, 100, true, 10, 1).unwrap();
        assert!(state.endstop(20).unwrap().is_homing());
    }

    #[test]
    fn endstop_home_unknown_endstop_oid() {
        use crate::commands::trsync::config_trsync;
        let mut state = State::new();
        config_trsync(&mut state, 10).unwrap();
        assert_eq!(
            endstop_home(&mut state, 99, 1000, 500, 3, 100, true, 10, 1),
            Err(CommandError::UnknownOid(99))
        );
    }

    #[test]
    fn endstop_home_unknown_trsync_oid() {
        let mut state = State::new();
        config_endstop(&mut state, 20).unwrap();
        assert_eq!(
            endstop_home(&mut state, 20, 1000, 500, 3, 100, true, 99, 1),
            Err(CommandError::UnknownOid(99))
        );
    }

    #[test]
    fn endstop_home_zero_sample_count_does_not_require_trsync() {
        let mut state = State::new();
        config_endstop(&mut state, 20).unwrap();
        assert_eq!(
            endstop_home(&mut state, 20, 1000, 500, 0, 100, true, 99, 1),
            Ok(())
        );
        assert!(!state.endstop(20).unwrap().is_homing());
    }

    #[test]
    fn endstop_query_state_not_homing() {
        let mut state = State::new();
        config_endstop(&mut state, 20).unwrap();
        let (homing, _) = endstop_query_state(&state, 20).unwrap();
        assert!(!homing);
    }

    #[test]
    fn endstop_query_state_homing() {
        use crate::commands::trsync::config_trsync;
        let mut state = State::new();
        config_endstop(&mut state, 20).unwrap();
        config_trsync(&mut state, 10).unwrap();
        endstop_home(&mut state, 20, 1000, 500, 3, 100, true, 10, 1).unwrap();
        let (homing, next_clock) = endstop_query_state(&state, 20).unwrap();
        assert!(homing);
        assert_eq!(next_clock, 1000);
    }

    #[test]
    fn endstop_query_state_unknown_oid() {
        let state = State::new();
        assert_eq!(
            endstop_query_state(&state, 99),
            Err(CommandError::UnknownOid(99))
        );
    }
}
