//! Stepper registration and motion commands.

use crate::commands::CommandError;
use crate::state::{State, NUM_STEPPERS};
use crate::stepper::StepMove;

/// Register a stepper motor object by OID.
///
/// Pin parameters from the Klipper protocol are accepted by the firmware
/// wrapper but not passed here — the TMC4671 handles stepping internally.
pub fn config_stepper(state: &mut State, oid: u8) -> Result<(), CommandError> {
    let idx = state.steppers_by_oid.len();
    if idx >= NUM_STEPPERS {
        return Err(CommandError::SlotsFull);
    }
    state
        .steppers_by_oid
        .insert(oid, idx)
        .map_err(|_| CommandError::SlotsFull)?;
    Ok(())
}

/// Enqueue a trapezoidal move segment for a stepper.
pub fn queue_step(
    state: &mut State,
    oid: u8,
    interval: u32,
    count: u16,
    add: i16,
) -> Result<(), CommandError> {
    let stepper = state
        .stepper_mut(oid)
        .ok_or(CommandError::UnknownOid(oid))?;
    let m = StepMove::new(interval, count, add);
    stepper.queue_move(m).map_err(|_| CommandError::QueueFull)
}

/// Set the direction for subsequent steps on a stepper.
///
/// `dir=0` means negative, `dir=1` means positive (matches Klipper
/// stepcompress).
pub fn set_next_step_dir(state: &mut State, oid: u8, dir: u8) {
    if let Some(stepper) = state.stepper_mut(oid) {
        stepper.set_direction(dir != 0);
    }
}

/// Reset the step clock reference for a stepper.
pub fn reset_step_clock(state: &mut State, oid: u8, clock: u32) {
    if let Some(stepper) = state.stepper_mut(oid) {
        stepper.reset_clock(clock);
    }
}

/// Return the current position of a stepper.
pub fn stepper_get_position(state: &State, oid: u8) -> i32 {
    state.stepper(oid).map(|s| s.position()).unwrap_or(0)
}

/// Return the commanded (target) position of a stepper.
///
/// For TMC4671 mode this is equivalent to `stepper_get_position` since
/// we track only the logical step count.
pub fn stepper_get_commanded_position(state: &State, oid: u8) -> i32 {
    state.stepper(oid).map(|s| s.position()).unwrap_or(0)
}

/// Link a stepper to a TrSync so the stepper stops when the TrSync
/// triggers.
pub fn stepper_stop_on_trigger(
    state: &mut State,
    oid: u8,
    trsync_oid: u8,
) -> Result<(), CommandError> {
    state
        .stepper_trsync_links
        .push((oid, trsync_oid))
        .map_err(|_| CommandError::SlotsFull)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn config_stepper_success() {
        let mut state = State::new();
        assert!(config_stepper(&mut state, 0).is_ok());
        assert!(state.stepper(0).is_some());
    }

    #[test]
    fn config_stepper_two_oids() {
        let mut state = State::new();
        assert!(config_stepper(&mut state, 5).is_ok());
        assert!(config_stepper(&mut state, 10).is_ok());
        assert!(state.stepper(5).is_some());
        assert!(state.stepper(10).is_some());
    }

    #[test]
    fn config_stepper_slots_full() {
        let mut state = State::new();
        for i in 0..NUM_STEPPERS as u8 {
            assert!(config_stepper(&mut state, i).is_ok());
        }
        assert_eq!(config_stepper(&mut state, 99), Err(CommandError::SlotsFull));
    }

    #[test]
    fn queue_step_unknown_oid() {
        let mut state = State::new();
        assert_eq!(
            queue_step(&mut state, 42, 1000, 10, 0),
            Err(CommandError::UnknownOid(42))
        );
    }

    #[test]
    fn queue_step_success() {
        let mut state = State::new();
        config_stepper(&mut state, 0).unwrap();
        assert!(queue_step(&mut state, 0, 1000, 10, 0).is_ok());
    }

    #[test]
    fn set_next_step_dir_forward() {
        let mut state = State::new();
        config_stepper(&mut state, 0).unwrap();
        set_next_step_dir(&mut state, 0, 1);
        // Direction is consumed on next queue_move; verify no panic.
    }

    #[test]
    fn stepper_get_position_unknown_oid() {
        let state = State::new();
        assert_eq!(stepper_get_position(&state, 99), 0);
    }

    #[test]
    fn stepper_stop_on_trigger_success() {
        let mut state = State::new();
        config_stepper(&mut state, 0).unwrap();
        assert!(stepper_stop_on_trigger(&mut state, 0, 10).is_ok());
        assert_eq!(state.stepper_trsync_links.len(), 1);
        assert_eq!(state.stepper_trsync_links[0], (0, 10));
    }

    #[test]
    fn stepper_stop_on_trigger_link_table_full() {
        let mut state = State::new();
        config_stepper(&mut state, 0).unwrap();
        // Fill the link table (capacity 8)
        for i in 0..8u8 {
            stepper_stop_on_trigger(&mut state, 0, i).unwrap();
        }
        assert_eq!(
            stepper_stop_on_trigger(&mut state, 0, 99),
            Err(CommandError::SlotsFull)
        );
    }
}
