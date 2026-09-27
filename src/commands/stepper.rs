//! Stepper registration and motion commands.

use crate::commands::CommandError;
use crate::state::{NUM_STEPPERS, State};
use crate::stepper::HardwareStepper;

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

/// Register a stepper motor object and store its board-local channel.
pub fn config_stepper_channel(state: &mut State, oid: u8, channel: u8) -> Result<(), CommandError> {
    if state.steppers_by_oid.contains_key(&oid) {
        return Err(CommandError::DuplicateOid(oid));
    }
    if state.stepper_channels.contains(&Some(channel)) {
        return Err(CommandError::ChannelInUse(channel));
    }

    config_stepper(state, oid)?;
    let idx = *state
        .steppers_by_oid
        .get(&oid)
        .ok_or(CommandError::UnknownOid(oid))?;
    state.stepper_channels[idx] = Some(channel);
    Ok(())
}

// ---------------------------------------------------------------------------
// Hardware step/dir command helpers
// ---------------------------------------------------------------------------

/// Result of a `queue_step` command for hardware step/dir mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum QueueStepResult {
    /// Move was silently discarded because `need_reset` is set.
    Discarded,
    /// Move passed the reset and count checks; the caller may enqueue it.
    Admitted,
}

/// Resolve the `queue_step` checks that depend only on the command itself.
///
/// A pending `need_reset` discards the move, even a zero-count one.
///
/// # Errors
///
/// Returns [`CommandError::ZeroCount`] when `need_reset` is false and `count`
/// is zero.
pub fn queue_step_hw(need_reset: bool, count: u16) -> Result<QueueStepResult, CommandError> {
    if need_reset {
        return Ok(QueueStepResult::Discarded);
    }
    if count == 0 {
        return Err(CommandError::ZeroCount);
    }
    Ok(QueueStepResult::Admitted)
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

/// Rebase both halves of the split hardware stepper clock.
///
/// Call only after [`reset_step_clock_hw`] has accepted the reset and the
/// physical timer is inactive. The command-side stepper records direction
/// state for future queued moves; the ISR-side stepper owns the clock used
/// when those moves are executed.
pub fn reset_stepper_clocks(
    p2_stepper: &mut HardwareStepper,
    isr_stepper: &mut HardwareStepper,
    clock: u32,
) {
    p2_stepper.reset_step_clock(clock);
    isr_stepper.reset_step_clock(clock);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::stepper::HardwareStepper;

    #[test]
    fn config_stepper_success() {
        let mut state = State::new();
        assert!(config_stepper(&mut state, 0).is_ok());
        assert!(state.steppers_by_oid.contains_key(&0));
    }

    #[test]
    fn config_stepper_two_oids() {
        let mut state = State::new();
        assert!(config_stepper(&mut state, 5).is_ok());
        assert!(config_stepper(&mut state, 10).is_ok());
        assert!(state.steppers_by_oid.contains_key(&5));
        assert!(state.steppers_by_oid.contains_key(&10));
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
    fn config_stepper_channel_stores_channel() {
        let mut state = State::new();
        assert_eq!(config_stepper_channel(&mut state, 5, 1), Ok(()));
        assert_eq!(state.stepper_channel(5), Some(1));
    }

    #[test]
    fn config_stepper_channel_duplicate_oid_takes_precedence_over_channel() {
        let mut state = State::new();

        assert_eq!(config_stepper_channel(&mut state, 5, 1), Ok(()));
        assert_eq!(
            config_stepper_channel(&mut state, 5, 1),
            Err(CommandError::DuplicateOid(5))
        );
        assert_eq!(state.steppers_by_oid.len(), 1);
        assert_eq!(state.stepper_channel(5), Some(1));
    }

    #[test]
    fn config_stepper_channel_rejects_duplicate_channel() {
        let mut state = State::new();

        assert_eq!(config_stepper_channel(&mut state, 5, 1), Ok(()));
        let registered_steppers = state.steppers_by_oid.len();
        assert_eq!(
            config_stepper_channel(&mut state, 6, 1),
            Err(CommandError::ChannelInUse(1))
        );
        assert_eq!(state.steppers_by_oid.len(), registered_steppers);
        assert!(!state.steppers_by_oid.contains_key(&6));
        assert_eq!(state.stepper_channel(5), Some(1));
        assert_eq!(state.stepper_channel(6), None);
    }

    #[test]
    fn config_stepper_channel_allows_distinct_channels() {
        let mut state = State::new();

        assert_eq!(config_stepper_channel(&mut state, 5, 0), Ok(()));
        assert_eq!(config_stepper_channel(&mut state, 6, 1), Ok(()));

        assert_eq!(state.stepper_channel(5), Some(0));
        assert_eq!(state.stepper_channel(6), Some(1));
    }

    // --- Hardware step/dir command tests ---

    #[test]
    fn queue_step_hw_need_reset_discards_even_zero_count() {
        assert_eq!(queue_step_hw(true, 0), Ok(QueueStepResult::Discarded));
    }

    #[test]
    fn queue_step_hw_zero_count_errors() {
        assert_eq!(queue_step_hw(false, 0), Err(CommandError::ZeroCount));
    }

    #[test]
    fn queue_step_hw_admits_positive_count() {
        assert_eq!(queue_step_hw(false, 10), Ok(QueueStepResult::Admitted));
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
    fn reset_stepper_clocks_rebases_command_and_isr_state() {
        let mut p2_stepper = HardwareStepper::new();
        let mut isr_stepper = HardwareStepper::new();

        isr_stepper.reset_step_clock(0);
        isr_stepper.load_move(crate::stepper::StepMove::new(50_000, 1, 0));
        assert_eq!(isr_stepper.next_step_clock(), 50_000);

        reset_stepper_clocks(&mut p2_stepper, &mut isr_stepper, 1_000);

        let queued = p2_stepper.prepare_move(crate::stepper::StepMove::new(250, 1, 0));
        isr_stepper.load_move(queued);

        assert_eq!(isr_stepper.next_step_clock(), 1_250);
        assert_eq!(
            isr_stepper.check_deadline(isr_stepper.next_step_clock(), 1_000, 84_000_000),
            Ok(1_250)
        );
    }
}
