//! Trigger synchronization commands.

use crate::commands::CommandError;
use crate::state::{State, NUM_TRSYNC};

/// Register a TrSync (trigger synchronization) object by OID.
pub fn config_trsync(state: &mut State, oid: u8) -> Result<(), CommandError> {
    if state.trsync_by_oid.contains_key(&oid) {
        return Err(CommandError::DuplicateOid(oid));
    }
    let idx = state.trsync_by_oid.len();
    if idx >= NUM_TRSYNC {
        return Err(CommandError::SlotsFull);
    }
    state
        .trsync_by_oid
        .insert(oid, idx)
        .map_err(|_| CommandError::SlotsFull)?;
    Ok(())
}

/// Arm a TrSync for synchronized triggering.
///
/// Clears old stepper links for this TrSync OID first (matches C
/// `trsync_clear` behavior — without this, links accumulate across
/// homing cycles and overflow the Vec).
pub fn trsync_start(
    state: &mut State,
    oid: u8,
    report_clock: u32,
    report_ticks: u32,
    expire_reason: u8,
) -> Result<(), CommandError> {
    state
        .stepper_trsync_links
        .retain(|&(_, ts_oid)| ts_oid != oid);
    let ts = state.trsync_mut(oid).ok_or(CommandError::UnknownOid(oid))?;
    ts.arm(report_clock, report_ticks, expire_reason);
    Ok(())
}

/// Update the expiry deadline for a TrSync.
pub fn trsync_set_timeout(state: &mut State, oid: u8, clock: u32) -> Result<(), CommandError> {
    let ts = state.trsync_mut(oid).ok_or(CommandError::UnknownOid(oid))?;
    ts.set_timeout(clock);
    Ok(())
}

/// Manually trigger a TrSync and unconditionally clean up.
///
/// Mirrors Klipper C's `command_trsync_trigger`: attempts to trigger (no-op
/// if already triggered or idle), then unconditionally stops the report and
/// expire timers by resetting to Idle.
///
/// Returns `Ok(trigger_reason)` — the reason from the triggered state (or
/// the newly supplied `reason` if the trigger was accepted now).
/// Returns `Err(UnknownOid)` if the OID is not registered.
///
/// The caller must send a final `trsync_state` reply with `can_trigger=0`
/// and the returned `trigger_reason`.
pub fn trsync_trigger(state: &mut State, oid: u8, reason: u8) -> Result<u8, CommandError> {
    // Trigger + extract reason + disarm in one borrow, then stop steppers.
    // Uses disarm() (not reset()) to preserve trigger_reason across multiple
    // trsync_trigger calls — the trdispatch C helper sends one, then stop()
    // sends another, and both must see the original endstop trigger_reason.
    let (first_trigger, trigger_reason) = {
        let ts = state.trsync_mut(oid).ok_or(CommandError::UnknownOid(oid))?;
        let first_trigger = ts.trigger(reason);
        let trigger_reason = ts.trigger_reason();
        ts.disarm();
        (first_trigger, trigger_reason)
    };
    if first_trigger {
        stop_linked_steppers(state, oid);
    }
    Ok(trigger_reason)
}

/// Stop all steppers linked to the given TrSync OID.
///
/// No-op in the library crate. The board crate handles hardware stepper
/// stopping directly via `stepper_stop()` and the `STEPPER_NEED_RESET`
/// atomic when a TrSync fires.
fn stop_linked_steppers(_state: &mut State, _trsync_oid: u8) {}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::commands::stepper::{config_stepper, stepper_stop_on_trigger};
    use crate::trsync::TrSyncState;

    #[test]
    fn config_trsync_success() {
        let mut state = State::new();
        assert!(config_trsync(&mut state, 10).is_ok());
        assert!(state.trsync(10).is_some());
    }

    #[test]
    fn config_trsync_slots_full() {
        let mut state = State::new();
        for i in 0..NUM_TRSYNC as u8 {
            config_trsync(&mut state, i).unwrap();
        }
        assert_eq!(config_trsync(&mut state, 99), Err(CommandError::SlotsFull));
    }

    #[test]
    fn trsync_start_unknown_oid() {
        let mut state = State::new();
        assert_eq!(
            trsync_start(&mut state, 99, 1000, 500, 1),
            Err(CommandError::UnknownOid(99))
        );
    }

    #[test]
    fn trsync_set_timeout_unknown_oid() {
        let mut state = State::new();
        assert_eq!(
            trsync_set_timeout(&mut state, 99, 5000),
            Err(CommandError::UnknownOid(99))
        );
    }

    #[test]
    fn trsync_start_arms() {
        let mut state = State::new();
        config_trsync(&mut state, 10).unwrap();
        trsync_start(&mut state, 10, 1000, 500, 1).unwrap();
        assert_eq!(state.trsync(10).unwrap().state(), TrSyncState::Armed);
    }

    #[test]
    fn trsync_start_clears_old_links() {
        let mut state = State::new();
        config_stepper(&mut state, 0).unwrap();
        config_trsync(&mut state, 10).unwrap();
        stepper_stop_on_trigger(&mut state, 0, 10).unwrap();
        assert_eq!(state.stepper_trsync_links.len(), 1);

        trsync_start(&mut state, 10, 1000, 500, 1).unwrap();
        assert!(state.stepper_trsync_links.is_empty());
    }

    #[test]
    fn trsync_trigger_accepted_and_disarms() {
        let mut state = State::new();
        config_trsync(&mut state, 10).unwrap();
        trsync_start(&mut state, 10, 1000, 500, 1).unwrap();
        // Returns the trigger reason; state stays Triggered (not Idle)
        assert_eq!(trsync_trigger(&mut state, 10, 2), Ok(2));
        assert_eq!(state.trsync(10).unwrap().state(), TrSyncState::Triggered(2));
    }

    #[test]
    fn trsync_trigger_not_armed_returns_zero() {
        let mut state = State::new();
        config_trsync(&mut state, 10).unwrap();
        // Idle trsync: trigger is no-op, reason is 0
        assert_eq!(trsync_trigger(&mut state, 10, 1), Ok(0));
        assert_eq!(state.trsync(10).unwrap().state(), TrSyncState::Idle);
    }

    #[test]
    fn trsync_trigger_already_triggered_preserves_reason() {
        let mut state = State::new();
        config_trsync(&mut state, 10).unwrap();
        trsync_start(&mut state, 10, 1000, 500, 1).unwrap();
        // Endstop triggers with reason 1
        state.trsync_mut(10).unwrap().trigger(1);
        // trdispatch C helper sends trsync_trigger with reason 2 — returns original 1
        assert_eq!(trsync_trigger(&mut state, 10, 2), Ok(1));
        assert_eq!(state.trsync(10).unwrap().state(), TrSyncState::Triggered(1));
        // stop() sends another trsync_trigger with reason 2 — still returns original 1
        assert_eq!(trsync_trigger(&mut state, 10, 2), Ok(1));
        assert_eq!(state.trsync(10).unwrap().state(), TrSyncState::Triggered(1));
    }

    #[test]
    fn trsync_trigger_with_linked_stepper() {
        let mut state = State::new();
        config_stepper(&mut state, 0).unwrap();
        config_trsync(&mut state, 10).unwrap();

        trsync_start(&mut state, 10, 1000, 500, 1).unwrap();
        stepper_stop_on_trigger(&mut state, 0, 10).unwrap();
        assert_eq!(trsync_trigger(&mut state, 10, 2), Ok(2));
        assert_eq!(state.trsync(10).unwrap().state(), TrSyncState::Triggered(2));
    }

    #[test]
    fn trsync_start_clears_previous_trigger() {
        let mut state = State::new();
        config_trsync(&mut state, 10).unwrap();
        trsync_start(&mut state, 10, 1000, 500, 1).unwrap();
        trsync_trigger(&mut state, 10, 5).unwrap();
        // State is Triggered(5) after trigger + disarm
        assert_eq!(state.trsync(10).unwrap().state(), TrSyncState::Triggered(5));
        // Re-arm for next homing cycle clears everything
        trsync_start(&mut state, 10, 2000, 500, 2).unwrap();
        assert_eq!(state.trsync(10).unwrap().state(), TrSyncState::Armed);
    }

    #[test]
    fn trsync_trigger_unknown_oid() {
        let mut state = State::new();
        assert_eq!(
            trsync_trigger(&mut state, 99, 1),
            Err(CommandError::UnknownOid(99))
        );
    }
}
