//! Trigger synchronization commands.

use crate::commands::CommandError;
use crate::state::{State, NUM_TRSYNC};

/// Register a TrSync (trigger synchronization) object by OID.
pub fn config_trsync(state: &mut State, oid: u8) -> Result<(), CommandError> {
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
) {
    state
        .stepper_trsync_links
        .retain(|&(_, ts_oid)| ts_oid != oid);
    if let Some(ts) = state.trsync_mut(oid) {
        ts.arm(report_clock, report_ticks, expire_reason);
    }
}

/// Update the expiry deadline for a TrSync.
pub fn trsync_set_timeout(state: &mut State, oid: u8, clock: u32) {
    if let Some(ts) = state.trsync_mut(oid) {
        ts.set_timeout(clock);
    }
}

/// Manually trigger a TrSync with the given reason code.
///
/// Returns `true` if the trigger was accepted (TrSync was Armed).
/// When `true`, all steppers linked to this TrSync are stopped.
pub fn trsync_trigger(state: &mut State, oid: u8, reason: u8) -> bool {
    let accepted = state
        .trsync_mut(oid)
        .map(|ts| ts.trigger(reason))
        .unwrap_or(false);
    if accepted {
        stop_linked_steppers(state, oid);
    }
    accepted
}

/// Stop all steppers linked to the given TrSync OID.
///
/// Iterates `stepper_trsync_links` and calls `stop()` on each stepper
/// whose entry matches `trsync_oid`. Uses index-based iteration for
/// split borrows.
fn stop_linked_steppers(state: &mut State, trsync_oid: u8) {
    for i in 0..state.stepper_trsync_links.len() {
        let (stepper_oid, linked_trsync) = state.stepper_trsync_links[i];
        if linked_trsync == trsync_oid {
            if let Some(&s_idx) = state.steppers_by_oid.get(&stepper_oid) {
                state.steppers[s_idx].stop();
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::commands::stepper::{config_stepper, queue_step, stepper_stop_on_trigger};
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
    fn trsync_start_arms() {
        let mut state = State::new();
        config_trsync(&mut state, 10).unwrap();
        trsync_start(&mut state, 10, 1000, 500, 1);
        assert_eq!(state.trsync(10).unwrap().state(), TrSyncState::Armed);
    }

    #[test]
    fn trsync_start_clears_old_links() {
        let mut state = State::new();
        config_stepper(&mut state, 0).unwrap();
        config_trsync(&mut state, 10).unwrap();
        stepper_stop_on_trigger(&mut state, 0, 10).unwrap();
        assert_eq!(state.stepper_trsync_links.len(), 1);

        trsync_start(&mut state, 10, 1000, 500, 1);
        assert!(state.stepper_trsync_links.is_empty());
    }

    #[test]
    fn trsync_trigger_accepted() {
        let mut state = State::new();
        config_trsync(&mut state, 10).unwrap();
        trsync_start(&mut state, 10, 1000, 500, 1);
        assert!(trsync_trigger(&mut state, 10, 2));
        assert_eq!(state.trsync(10).unwrap().state(), TrSyncState::Triggered(2));
    }

    #[test]
    fn trsync_trigger_not_armed() {
        let mut state = State::new();
        config_trsync(&mut state, 10).unwrap();
        assert!(!trsync_trigger(&mut state, 10, 1));
    }

    #[test]
    fn trsync_trigger_stops_linked_steppers() {
        let mut state = State::new();
        config_stepper(&mut state, 0).unwrap();
        config_trsync(&mut state, 10).unwrap();

        queue_step(&mut state, 0, 1000, 100, 0).unwrap();
        assert!(!state.stepper(0).unwrap().move_queue_empty());

        trsync_start(&mut state, 10, 1000, 500, 1);
        stepper_stop_on_trigger(&mut state, 0, 10).unwrap();
        trsync_trigger(&mut state, 10, 2);

        assert!(state.stepper(0).unwrap().move_queue_empty());
    }

    #[test]
    fn trsync_trigger_unknown_oid() {
        let mut state = State::new();
        assert!(!trsync_trigger(&mut state, 99, 1));
    }
}
