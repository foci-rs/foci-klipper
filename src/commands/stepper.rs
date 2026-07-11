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
    motion_scale_configured: bool,
) -> Result<QueueStepResult, CommandError> {
    if need_reset {
        return Ok(QueueStepResult::Discarded);
    }
    if count == 0 {
        return Err(CommandError::ZeroCount);
    }
    if !motion_scale_configured {
        return Err(CommandError::MotionScaleUnconfigured);
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
    fn queue_step_hw_need_reset_discards() {
        let result = queue_step_hw(true, 10, false, false);
        assert_eq!(result, Ok(QueueStepResult::Discarded));
    }

    #[test]
    fn queue_step_hw_zero_count_errors() {
        let result = queue_step_hw(false, 0, false, true);
        assert_eq!(result, Err(CommandError::ZeroCount));
    }

    #[test]
    fn queue_step_hw_normal_queued() {
        let result = queue_step_hw(false, 10, false, true);
        assert_eq!(result, Ok(QueueStepResult::Queued));
    }

    #[test]
    fn queue_step_hw_first_move() {
        let result = queue_step_hw(false, 10, true, true);
        assert_eq!(result, Ok(QueueStepResult::QueuedFirstMove));
    }

    #[test]
    fn queue_step_hw_requires_configured_motion_scale() {
        let result = queue_step_hw(false, 10, true, false);
        assert_eq!(result, Err(CommandError::MotionScaleUnconfigured));
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
