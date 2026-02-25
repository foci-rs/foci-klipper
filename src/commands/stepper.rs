//! Stepper registration and motion commands.

use crate::commands::CommandError;
use crate::state::{State, NUM_STEPPERS};
use crate::stepper::StepMove;

/// Register a stepper motor object by OID.
///
/// Pin parameters from the Klipper protocol are accepted by the firmware
/// wrapper but not passed here — the TMC4671 handles stepping internally.
pub fn config_stepper(state: &mut State, oid: u8) -> Result<(), CommandError> {
    if state.steppers_by_oid.contains_key(&oid) {
        return Err(CommandError::DuplicateOid(oid));
    }
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
pub fn set_next_step_dir(state: &mut State, oid: u8, dir: u8) -> Result<(), CommandError> {
    let stepper = state
        .stepper_mut(oid)
        .ok_or(CommandError::UnknownOid(oid))?;
    stepper.set_direction(dir != 0);
    Ok(())
}

/// Reset the step clock reference for a stepper.
pub fn reset_step_clock(state: &mut State, oid: u8, clock: u32) -> Result<(), CommandError> {
    let stepper = state
        .stepper_mut(oid)
        .ok_or(CommandError::UnknownOid(oid))?;
    stepper.reset_clock(clock);
    Ok(())
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
    if !state.steppers_by_oid.contains_key(&oid) {
        return Err(CommandError::UnknownOid(oid));
    }
    if !state.trsync_by_oid.contains_key(&trsync_oid) {
        return Err(CommandError::UnknownOid(trsync_oid));
    }
    state
        .stepper_trsync_links
        .push((oid, trsync_oid))
        .map_err(|_| CommandError::SlotsFull)
}

// ---------------------------------------------------------------------------
// Hardware step/dir command helpers
// ---------------------------------------------------------------------------

/// Result of a `queue_step` command for hardware step/dir mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum QueueStepResult {
    /// Move was silently discarded because `need_reset` is set.
    Discarded,
    /// Move was queued normally.
    Queued,
    /// Move was queued and the queue was previously empty.
    /// Caller may need to arm the timer if `motor_armed` is true.
    QueuedFirstMove,
}

/// Validate and prepare a `queue_step` command for hardware step/dir mode.
///
/// Checks `need_reset` (discard) and `count == 0` (error) before accepting
/// the move. The actual SPSC enqueue and timer arming are done by the board
/// crate caller — this function only validates.
///
/// `queue_was_empty` should reflect whether the SPSC queue was empty before
/// the caller pushes this move.
pub fn queue_step_hw(
    need_reset: bool,
    count: u16,
    queue_was_empty: bool,
) -> Result<QueueStepResult, CommandError> {
    if need_reset {
        return Ok(QueueStepResult::Discarded);
    }
    if count == 0 {
        return Err(CommandError::ZeroCount);
    }
    if queue_was_empty {
        Ok(QueueStepResult::QueuedFirstMove)
    } else {
        Ok(QueueStepResult::Queued)
    }
}

/// Validate a `reset_step_clock` command for hardware step/dir mode.
///
/// Matches Klipper `stepper.c` line 304: reset is only valid when the
/// stepper is inactive. If `timer_active` is true, returns
/// `Err(CommandError::StepperActive)` and the caller should trigger shutdown.
pub fn reset_step_clock_hw(timer_active: bool) -> Result<(), CommandError> {
    if timer_active {
        return Err(CommandError::StepperActive);
    }
    Ok(())
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
    fn config_stepper_duplicate_oid() {
        let mut state = State::new();
        assert!(config_stepper(&mut state, 5).is_ok());
        assert_eq!(
            config_stepper(&mut state, 5),
            Err(CommandError::DuplicateOid(5))
        );
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
        set_next_step_dir(&mut state, 0, 1).unwrap();
    }

    #[test]
    fn set_next_step_dir_unknown_oid() {
        let mut state = State::new();
        assert_eq!(
            set_next_step_dir(&mut state, 99, 1),
            Err(CommandError::UnknownOid(99))
        );
    }

    #[test]
    fn reset_step_clock_unknown_oid() {
        let mut state = State::new();
        assert_eq!(
            reset_step_clock(&mut state, 99, 1000),
            Err(CommandError::UnknownOid(99))
        );
    }

    #[test]
    fn stepper_get_position_unknown_oid() {
        let state = State::new();
        assert_eq!(stepper_get_position(&state, 99), 0);
    }

    #[test]
    fn stepper_stop_on_trigger_success() {
        use crate::commands::trsync::config_trsync;
        let mut state = State::new();
        config_stepper(&mut state, 0).unwrap();
        config_trsync(&mut state, 10).unwrap();
        assert!(stepper_stop_on_trigger(&mut state, 0, 10).is_ok());
        assert_eq!(state.stepper_trsync_links.len(), 1);
        assert_eq!(state.stepper_trsync_links[0], (0, 10));
    }

    #[test]
    fn stepper_stop_on_trigger_unknown_stepper() {
        use crate::commands::trsync::config_trsync;
        let mut state = State::new();
        config_trsync(&mut state, 10).unwrap();
        assert_eq!(
            stepper_stop_on_trigger(&mut state, 99, 10),
            Err(CommandError::UnknownOid(99))
        );
    }

    #[test]
    fn stepper_stop_on_trigger_unknown_trsync() {
        let mut state = State::new();
        config_stepper(&mut state, 0).unwrap();
        assert_eq!(
            stepper_stop_on_trigger(&mut state, 0, 99),
            Err(CommandError::UnknownOid(99))
        );
    }

    // --- Hardware step/dir command tests ---

    #[test]
    fn queue_step_hw_need_reset_discards() {
        let result = queue_step_hw(true, 10, false);
        assert_eq!(result, Ok(QueueStepResult::Discarded));
    }

    #[test]
    fn queue_step_hw_zero_count_errors() {
        let result = queue_step_hw(false, 0, false);
        assert_eq!(result, Err(CommandError::ZeroCount));
    }

    #[test]
    fn queue_step_hw_normal_queued() {
        let result = queue_step_hw(false, 10, false);
        assert_eq!(result, Ok(QueueStepResult::Queued));
    }

    #[test]
    fn queue_step_hw_first_move() {
        let result = queue_step_hw(false, 10, true);
        assert_eq!(result, Ok(QueueStepResult::QueuedFirstMove));
    }

    #[test]
    fn reset_step_clock_hw_active_errors() {
        let result = reset_step_clock_hw(true);
        assert_eq!(result, Err(CommandError::StepperActive));
    }

    #[test]
    fn reset_step_clock_hw_inactive_ok() {
        let result = reset_step_clock_hw(false);
        assert!(result.is_ok());
    }

    #[test]
    fn stepper_stop_on_trigger_link_table_full() {
        use crate::commands::trsync::config_trsync;
        let mut state = State::new();
        config_stepper(&mut state, 0).unwrap();
        config_stepper(&mut state, 1).unwrap();
        config_trsync(&mut state, 10).unwrap();
        config_trsync(&mut state, 11).unwrap();
        config_trsync(&mut state, 12).unwrap();
        config_trsync(&mut state, 13).unwrap();
        // Fill the link table (capacity 8) with valid OID combinations
        for trsync in 10..14u8 {
            stepper_stop_on_trigger(&mut state, 0, trsync).unwrap();
            stepper_stop_on_trigger(&mut state, 1, trsync).unwrap();
        }
        assert_eq!(
            stepper_stop_on_trigger(&mut state, 0, 10),
            Err(CommandError::SlotsFull)
        );
    }
}
