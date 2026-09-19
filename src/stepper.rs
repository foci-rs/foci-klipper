//! Stepper motor types for Klipper MCU protocol.
//!
//! - `StepMove`: Move segment passed through the SPSC queue to the timer ISR.
//! - `HardwareStepper`: Timer-driven GPIO step/dir scheduling engine.
//!   Matches Klipper `stepper.c` scheduling semantics.

use crate::pulse_mapper::{PulseMapperConfig, ReversiblePulseMapper};

/// Scheduled step time was more than 1ms in the past.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct MissedDeadline;

/// A single move segment from Klipper's `queue_step` command.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct StepMove {
    /// Clock ticks between steps (initial value for this segment).
    pub interval: u32,
    /// Number of steps in this segment.
    pub count: u16,
    /// Interval change per step (acceleration).
    pub add: i16,
    /// Absolute DIR level for this segment.
    ///
    /// Storing the level rather than a relative toggle keeps later segments
    /// correct when a stop path discards queued moves.
    direction: bool,
}
impl StepMove {
    /// Create a new forward move segment.
    ///
    /// [`HardwareStepper::prepare_move`] replaces the direction with the
    /// command-side level selected for the queued move.
    pub const fn new(interval: u32, count: u16, add: i16) -> Self {
        Self {
            interval,
            count,
            add,
            direction: true,
        }
    }

    /// Returns the smallest logical interval used by this segment.
    pub fn minimum_interval(&self) -> u32 {
        if self.add >= 0 || self.count <= 1 {
            return self.interval.max(1);
        }
        let last_offset = i64::from(self.add) * i64::from(self.count - 1);
        (i64::from(self.interval) + last_offset).max(1) as u32
    }
}

// ---------------------------------------------------------------------------
// Hardware step/dir stepper
// ---------------------------------------------------------------------------

/// A logical planner step and its board-owned physical pulse work.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct LogicalStepEvent {
    /// Logical step direction and required physical DIR level.
    pub direction: bool,
    /// Number of physical pulses to emit for this logical step.
    pub physical_pulses: u8,
    /// Whether the board must satisfy DIR setup before emitting pulses.
    pub dir_setup_required: bool,
}

/// Motion-scale configuration errors.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum MotionScaleConfigError {
    /// The active move must finish or be reset before reconfiguration.
    MoveLoaded,
}

/// Result of a single step event in the timer ISR.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StepEventResult {
    /// Emit `event`, then schedule the next logical step at `clock`.
    ScheduleNext { clock: u32, event: LogicalStepEvent },
    /// Emit `event`, then dequeue the next move.
    NeedMove { event: LogicalStepEvent },
}

impl StepEventResult {
    /// Returns the logical event carried by this result.
    pub const fn event(self) -> LogicalStepEvent {
        match self {
            Self::ScheduleNext { event, .. } | Self::NeedMove { event } => event,
        }
    }

    /// Returns the next logical deadline, or `None` when another move is needed.
    pub const fn next_clock(self) -> Option<u32> {
        match self {
            Self::ScheduleNext { clock, .. } => Some(clock),
            Self::NeedMove { .. } => None,
        }
    }
}

/// Coarse motion-phase classification of a step-queue segment, derived from
/// its `add` sign. `add < 0` shrinks the interval each step (faster stepping,
/// acceleration); `add > 0` grows it (deceleration); `add == 0` holds it
/// constant (cruise).
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum PhaseClass {
    Accel = 0,
    Cruise = 1,
    Decel = 2,
}

impl PhaseClass {
    const fn from_add(add: i16) -> Self {
        if add < 0 {
            PhaseClass::Accel
        } else if add > 0 {
            PhaseClass::Decel
        } else {
            PhaseClass::Cruise
        }
    }
}

/// Timer-driven hardware stepper scheduling engine.
///
/// This struct holds two categories of state:
/// - **ISR state** (`current_*`, `next_step_clock`, `was_active`, `dir_state`):
///   Only accessed by the P3 timer ISR via `load_move()` and `step_event()`.
/// - **P2 state** (`last_dir`, `next_dir`): Only accessed by P2 command
///   handlers via `set_next_step_dir()`, `next_move_changes_direction()`, and
///   `prepare_move()`.
///
/// The move queue and atomic flags live outside this struct as statics
/// in the board crate (see spec: Concurrency Model).
pub struct HardwareStepper {
    // --- ISR state (P3 only) ---
    /// Current segment interval (ticks between steps).
    current_interval: u32,
    /// Logical planner steps remaining in the current segment.
    current_count: u16,
    /// Interval change per step (acceleration).
    current_add: i16,
    /// Clock time of the next step.
    next_step_clock: u32,
    /// Whether the stepper was active before loading the current segment.
    /// Used for missed-deadline check (Klipper `was_active` semantics).
    was_active: bool,
    /// Direction used by the current or most recent ISR move.
    dir_state: bool,
    /// Whether the next logical event must establish the board-owned DIR level.
    dir_setup_pending: bool,
    /// Phase class of the most recently loaded segment, for phase-boundary
    /// marker detection. `None` when no segment has been loaded since the
    /// last `reset()`/`mark_idle()` — a fresh move has no valid prior phase
    /// to compare against.
    last_phase_class: Option<PhaseClass>,
    /// Stateful logical-to-physical pulse mapper.
    pulse_mapper: Option<ReversiblePulseMapper>,

    // --- P2 state (command handlers only) ---
    /// Direction of the most recently queued move (Klipper `SF_LAST_DIR`).
    last_dir: bool,
    /// Direction for the next queued move (Klipper `SF_NEXT_DIR`).
    next_dir: bool,
}

impl Default for HardwareStepper {
    fn default() -> Self {
        Self::new()
    }
}

impl HardwareStepper {
    /// Create a new stepper in idle state.
    pub const fn new() -> Self {
        Self {
            current_interval: 0,
            current_count: 0,
            current_add: 0,
            next_step_clock: 0,
            was_active: false,
            dir_state: true,
            dir_setup_pending: true,
            last_phase_class: None,
            pulse_mapper: None,
            last_dir: true,
            next_dir: true,
        }
    }

    // --- P2 methods (command handlers) ---

    /// Set direction for the next queued move.
    ///
    /// Called from P2 `set_next_step_dir` command handler.
    pub fn set_next_step_dir(&mut self, forward: bool) {
        self.next_dir = forward;
    }

    /// Returns whether the next queued move would change direction.
    ///
    /// This preview does not consume the pending direction change. A subsequent
    /// [`prepare_move`](Self::prepare_move) call records and consumes it.
    pub const fn next_move_changes_direction(&self) -> bool {
        self.last_dir != self.next_dir
    }

    /// Prepare a move for queuing by attaching direction state.
    ///
    /// Called from P2 `queue_step` command handler. Returns the amended
    /// [`StepMove`] with the absolute direction selected for the queued move.
    /// The caller pushes the result to the SPSC queue.
    pub fn prepare_move(&mut self, m: StepMove) -> StepMove {
        if self.last_dir != self.next_dir {
            self.last_dir = self.next_dir;
        }
        StepMove {
            direction: self.next_dir,
            ..m
        }
    }

    /// Set the step clock reference.
    ///
    /// Called from P2 `reset_step_clock` command handler. Only valid when
    /// the stepper is inactive (timer not armed). The caller must check
    /// `timer_active` and trigger shutdown if true.
    pub fn reset_step_clock(&mut self, clock: u32) {
        self.next_step_clock = clock;
        self.was_active = false;
    }

    /// Reset the ISR stepper engine to idle state.
    ///
    /// Clears the active segment so the next ISR entry dequeues a fresh
    /// move. Called by `stop_physical_stepper_no_lock()` after disabling
    /// the step timer to prevent stale partial segments from executing
    /// when the timer is re-armed. The next event must also re-establish DIR
    /// because board stop paths reset the GPIO independently.
    pub fn reset(&mut self) {
        self.current_count = 0;
        self.current_interval = 0;
        self.current_add = 0;
        self.was_active = false;
        self.dir_setup_pending = true;
        self.last_phase_class = None;
    }

    /// Mark the stepper idle after the move queue is observed empty.
    ///
    /// Klipper's `stepper_load_next()` can tell the difference between a
    /// contiguous segment transition and an empty queue because the final
    /// unstep calls it before clearing `count`. This split Rust
    /// implementation returns `NeedMove` after decrementing the event count,
    /// so the board ISR must explicitly call this when the queue is empty.
    pub fn mark_idle(&mut self) {
        self.was_active = false;
        self.last_phase_class = None;
    }

    /// Remove the logical-to-physical pulse mapper without resetting stepper state.
    ///
    /// The caller must first stop the step timer and prevent move-queue execution. After this
    /// call, [`step_event`](Self::step_event) is invalid until
    /// [`configure_motion_scale`](Self::configure_motion_scale) succeeds, and queue admission
    /// must reject moves while the motion scale is unconfigured.
    ///
    /// This unconditionally clears the mapper, including while a move is loaded, so a subsequent
    /// configuration failure leaves the stepper fail-closed. Calling it repeatedly has no further
    /// effect. Logical clock, position, direction, and ordinary lifecycle state are preserved.
    pub fn clear_motion_scale(&mut self) {
        self.pulse_mapper = None;
    }

    /// Configure or replace the logical-to-physical pulse mapper at a common origin.
    ///
    /// # Errors
    ///
    /// Returns [`MotionScaleConfigError::MoveLoaded`] while a move is loaded.
    pub fn configure_motion_scale(
        &mut self,
        config: PulseMapperConfig,
    ) -> Result<(), MotionScaleConfigError> {
        if self.current_count > 0 {
            return Err(MotionScaleConfigError::MoveLoaded);
        }
        match self.pulse_mapper.as_mut() {
            Some(mapper) => mapper.reconfigure(config),
            None => self.pulse_mapper = Some(ReversiblePulseMapper::new(config)),
        }
        Ok(())
    }

    /// Returns whether a motion scale has been configured.
    pub const fn motion_scale_configured(&self) -> bool {
        self.pulse_mapper.is_some()
    }

    /// Returns the configured logical planner steps per revolution.
    pub fn planner_steps_per_rev(&self) -> Option<u32> {
        self.pulse_mapper
            .as_ref()
            .map(ReversiblePulseMapper::planner_steps_per_rev)
    }

    /// Returns the mapper phase without changing it.
    pub fn mapper_phase(&self) -> Option<i64> {
        self.pulse_mapper.as_ref().map(ReversiblePulseMapper::phase)
    }

    // --- P3 methods (timer ISR) ---

    /// Load a move segment from the queue into the active segment.
    ///
    /// Called from P3 timer ISR when the previous segment is exhausted.
    /// Sets `was_active` based on whether there was a previous segment.
    /// Returns `Some((old_phase, new_phase))` if the motion phase changed,
    /// or `None` if there was no prior phase or the phase remained the same.
    pub fn load_move(&mut self, m: StepMove) -> Option<(PhaseClass, PhaseClass)> {
        self.was_active = self.was_active || self.current_count > 0;
        self.current_interval = m.interval;
        self.current_count = m.count;
        self.current_add = m.add;
        self.next_step_clock = self.next_step_clock.wrapping_add(m.interval);
        if self.dir_state != m.direction {
            self.dir_state = m.direction;
            self.dir_setup_pending = true;
        }
        let new_class = PhaseClass::from_add(m.add);
        let transition = match self.last_phase_class {
            Some(prev) if prev != new_class => Some((prev, new_class)),
            _ => None,
        };
        self.last_phase_class = Some(new_class);
        transition
    }

    /// Consume one logical planner step and return its physical pulse work.
    ///
    /// GPIO transitions and pulse timing are owned by the board caller.
    pub fn step_event(&mut self) -> StepEventResult {
        debug_assert!(self.current_count > 0);
        self.current_count -= 1;
        let signed_pulses = self
            .pulse_mapper
            .as_mut()
            .expect("motion scale configured before loading moves")
            .map_step(self.dir_state);
        let event = LogicalStepEvent {
            direction: self.dir_state,
            physical_pulses: signed_pulses.unsigned_abs(),
            dir_setup_required: core::mem::take(&mut self.dir_setup_pending),
        };

        if self.current_count == 0 {
            self.was_active = true;
            return StepEventResult::NeedMove { event };
        }

        self.current_interval = (self.current_interval as i32)
            .saturating_add(self.current_add as i32)
            .max(1) as u32;
        self.next_step_clock = self.next_step_clock.wrapping_add(self.current_interval);
        StepEventResult::ScheduleNext {
            clock: self.next_step_clock,
            event,
        }
    }

    /// Check if the scheduled step time has passed acceptable bounds.
    ///
    /// Matches Klipper `stepper_load_next()` missed-deadline logic:
    /// - `was_active = false` (first activation): no shutdown check
    /// - `was_active = true` and >1ms past: return Err (caller shuts down)
    /// - `was_active = true` and 0-1ms past: nudge to `now + min_ticks`
    ///
    /// If this returns an adjusted immediate clock, callers must schedule that
    /// compare without rebasing `next_step_clock`. Klipper keeps the original
    /// cadence anchor so a stepper that briefly runs dry can catch up to the
    /// host schedule once more queue entries arrive.
    ///
    /// `clock_freq` is the timer frequency (84 MHz for TIM5). The 1ms
    /// threshold is `clock_freq / 1000`.
    pub fn check_deadline(
        &self,
        scheduled: u32,
        now: u32,
        clock_freq: u32,
    ) -> Result<u32, MissedDeadline> {
        if !self.was_active {
            // First activation after idle: no past-time check.
            // Fire immediately if past, or at scheduled time if future.
            let diff = (now as i32).wrapping_sub(scheduled as i32);
            if diff >= 0 {
                // Past or now: fire immediately (small future offset).
                return Ok(now.wrapping_add(2));
            }
            return Ok(scheduled);
        }

        let diff = (scheduled as i32).wrapping_sub(now as i32);
        if diff >= 0 {
            // Scheduled time is in the future — normal case.
            return Ok(scheduled);
        }

        // Scheduled time is in the past.
        let threshold = (clock_freq / 1000) as i32; // 1ms in timer ticks
        if diff < -threshold {
            // More than 1ms past: fatal.
            return Err(MissedDeadline);
        }

        // Within 1ms: nudge to now + minimum offset.
        Ok(now.wrapping_add(2))
    }

    /// Current step interval (clock ticks between steps).
    /// Updated by `step_event()` when `add != 0`.
    pub fn current_interval(&self) -> u32 {
        self.current_interval
    }

    /// Returns the next scheduled step clock.
    pub fn next_step_clock(&self) -> u32 {
        self.next_step_clock
    }

    /// Returns the current DIR state.
    pub fn dir_state(&self) -> bool {
        self.dir_state
    }

    /// Returns whether the active segment has steps remaining.
    pub fn has_active_segment(&self) -> bool {
        self.current_count > 0
    }

    /// Remaining logical planner steps in the active segment.
    pub fn active_step_count(&self) -> u16 {
        self.current_count
    }
}

#[cfg(test)]
mod hardware_stepper_tests {
    use super::*;

    fn configured_stepper(planner_steps_per_rev: u32) -> HardwareStepper {
        let mut stepper = HardwareStepper::new();
        let config = PulseMapperConfig::new(planner_steps_per_rev).unwrap();
        stepper.configure_motion_scale(config).unwrap();
        stepper
    }

    #[test]
    fn prepare_move_records_absolute_direction() {
        let mut stepper = HardwareStepper::new();
        assert!(stepper.prepare_move(StepMove::new(100, 1, 0)).direction);

        stepper.set_next_step_dir(false);
        assert!(!stepper.prepare_move(StepMove::new(100, 1, 0)).direction);
        assert!(!stepper.prepare_move(StepMove::new(100, 1, 0)).direction);
    }

    #[test]
    fn discarded_direction_change_does_not_invert_the_next_executed_move() {
        let mut command_stepper = HardwareStepper::new();
        let mut isr_stepper = configured_stepper(3_200);

        command_stepper.set_next_step_dir(false);
        let discarded = command_stepper.prepare_move(StepMove::new(100, 1, 0));
        assert!(!discarded.direction);

        let next = command_stepper.prepare_move(StepMove::new(100, 1, 0));
        isr_stepper.load_move(next);

        assert!(!isr_stepper.step_event().event().direction);
    }

    #[test]
    fn direction_change_preview_tracks_pending_move_without_consuming_it() {
        let mut stepper = HardwareStepper::new();
        assert!(!stepper.next_move_changes_direction());

        stepper.set_next_step_dir(false);
        assert!(stepper.next_move_changes_direction());
        assert!(stepper.next_move_changes_direction());

        stepper.prepare_move(StepMove::new(100, 1, 0));
        assert!(!stepper.next_move_changes_direction());

        stepper.set_next_step_dir(true);
        assert!(stepper.next_move_changes_direction());
    }

    #[test]
    fn logical_events_consume_one_move_count_and_preserve_deadlines() {
        let mut stepper = configured_stepper(3_200);
        stepper.reset_step_clock(0);
        stepper.load_move(StepMove::new(100, 3, 0));

        assert_eq!(
            stepper.step_event(),
            StepEventResult::ScheduleNext {
                clock: 200,
                event: LogicalStepEvent {
                    direction: true,
                    physical_pulses: 1,
                    dir_setup_required: true,
                },
            }
        );
        assert_eq!(stepper.active_step_count(), 2);
        assert_eq!(
            stepper.step_event(),
            StepEventResult::ScheduleNext {
                clock: 300,
                event: LogicalStepEvent {
                    direction: true,
                    physical_pulses: 2,
                    dir_setup_required: false,
                },
            }
        );
        assert!(matches!(
            stepper.step_event(),
            StepEventResult::NeedMove {
                event: LogicalStepEvent { .. }
            }
        ));
    }

    #[test]
    fn zero_one_and_two_pulses_share_logical_cadence() {
        let mut sparse = configured_stepper(102_400);
        sparse.reset_step_clock(0);
        sparse.load_move(StepMove::new(100, 2, 0));
        let sparse_events = [sparse.step_event(), sparse.step_event()];

        let mut dense = configured_stepper(3_200);
        dense.reset_step_clock(0);
        dense.load_move(StepMove::new(100, 2, 0));
        let dense_events = [dense.step_event(), dense.step_event()];

        assert_eq!(sparse_events[0].event().physical_pulses, 1);
        assert_eq!(sparse_events[0].next_clock(), Some(200));
        assert_eq!(sparse_events[1].event().physical_pulses, 0);
        assert_eq!(sparse_events[1].next_clock(), None);
        assert_eq!(dense_events[0].event().physical_pulses, 1);
        assert_eq!(dense_events[0].next_clock(), Some(200));
        assert_eq!(dense_events[1].event().physical_pulses, 2);
        assert_eq!(dense_events[1].next_clock(), None);
    }

    #[test]
    fn acceleration_and_segment_transitions_use_logical_steps() {
        let mut stepper = configured_stepper(3_200);
        stepper.reset_step_clock(0);
        stepper.load_move(StepMove::new(100, 2, -20));

        assert_eq!(stepper.step_event().next_clock(), Some(180));
        assert_eq!(stepper.current_interval(), 80);
        assert_eq!(stepper.step_event().next_clock(), None);
        stepper.load_move(StepMove::new(60, 1, 0));
        assert_eq!(stepper.next_step_clock(), 240);
    }

    #[test]
    fn load_move_reports_no_transition_for_the_first_segment_after_reset() {
        let mut stepper = configured_stepper(3_200);
        assert_eq!(stepper.load_move(StepMove::new(100, 5, -1)), None);
    }

    #[test]
    fn load_move_reports_no_transition_for_the_first_segment_after_idle() {
        let mut stepper = configured_stepper(3_200);
        stepper.load_move(StepMove::new(100, 5, -1));
        stepper.mark_idle();
        assert_eq!(stepper.load_move(StepMove::new(100, 5, -1)), None);
    }

    #[test]
    fn load_move_reports_no_transition_when_add_sign_is_unchanged() {
        let mut stepper = configured_stepper(3_200);
        stepper.load_move(StepMove::new(100, 5, -1));
        assert_eq!(stepper.load_move(StepMove::new(90, 5, -1)), None);
    }

    #[test]
    fn load_move_reports_no_transition_across_two_zero_add_segments() {
        let mut stepper = configured_stepper(3_200);
        stepper.load_move(StepMove::new(100, 5, 0));
        assert_eq!(stepper.load_move(StepMove::new(100, 5, 0)), None);
    }

    #[test]
    fn load_move_maps_negative_add_to_accel_and_positive_add_to_decel() {
        let mut stepper = configured_stepper(3_200);
        stepper.load_move(StepMove::new(100, 5, -1));
        assert_eq!(
            stepper.load_move(StepMove::new(90, 5, 1)),
            Some((PhaseClass::Accel, PhaseClass::Decel))
        );
    }

    #[test]
    fn load_move_reports_each_of_the_six_class_transitions() {
        let cases = [
            (-1, 0, PhaseClass::Accel, PhaseClass::Cruise),
            (-1, 1, PhaseClass::Accel, PhaseClass::Decel),
            (0, -1, PhaseClass::Cruise, PhaseClass::Accel),
            (0, 1, PhaseClass::Cruise, PhaseClass::Decel),
            (1, -1, PhaseClass::Decel, PhaseClass::Accel),
            (1, 0, PhaseClass::Decel, PhaseClass::Cruise),
        ];
        for (first_add, second_add, expected_old, expected_new) in cases {
            let mut stepper = configured_stepper(3_200);
            stepper.load_move(StepMove::new(100, 5, first_add));
            assert_eq!(
                stepper.load_move(StepMove::new(100, 5, second_add)),
                Some((expected_old, expected_new)),
                "first_add={first_add} second_add={second_add}"
            );
        }
    }

    #[test]
    fn reset_clears_last_phase_class_so_the_next_move_reports_no_transition() {
        let mut stepper = configured_stepper(3_200);
        stepper.load_move(StepMove::new(100, 5, -1));
        stepper.load_move(StepMove::new(100, 5, 1)); // now Decel
        stepper.reset();
        // Next move's first segment is also Decel, but reset() must have
        // cleared last_phase_class, so this must NOT report a transition.
        assert_eq!(stepper.load_move(StepMove::new(100, 5, 1)), None);
    }

    #[test]
    fn mark_idle_clears_last_phase_class_so_the_next_move_reports_no_transition() {
        let mut stepper = configured_stepper(3_200);
        stepper.load_move(StepMove::new(100, 5, -1));
        stepper.load_move(StepMove::new(100, 5, 1)); // now Decel
        stepper.mark_idle();
        assert_eq!(stepper.load_move(StepMove::new(100, 5, 1)), None);
    }

    #[test]
    fn negative_add_clamps_runtime_interval_to_one_tick() {
        let mut stepper = configured_stepper(3_200);
        stepper.reset_step_clock(0);
        stepper.load_move(StepMove::new(5, 4, -3));

        assert_eq!(stepper.step_event().next_clock(), Some(7));
        assert_eq!(stepper.current_interval(), 2);
        assert_eq!(stepper.step_event().next_clock(), Some(8));
        assert_eq!(stepper.current_interval(), 1);
        assert_eq!(stepper.step_event().next_clock(), Some(9));
        assert_eq!(stepper.current_interval(), 1);
        assert_eq!(stepper.step_event().next_clock(), None);
        assert_eq!(stepper.current_interval(), 1);
    }

    #[test]
    fn logical_position_counts_once_per_event_in_both_directions() {
        let mut stepper = configured_stepper(3_200);
        stepper.load_move(StepMove::new(100, 2, 0));
        let mut homing_position = 0_i32;
        for _ in 0..2 {
            let event = stepper.step_event().event();
            homing_position += if event.direction { 1 } else { -1 };
        }

        stepper.set_next_step_dir(false);
        let reverse = stepper.prepare_move(StepMove::new(100, 1, 0));
        stepper.load_move(reverse);
        let event = stepper.step_event().event();
        homing_position += if event.direction { 1 } else { -1 };

        assert_eq!(homing_position, 1);
    }

    #[test]
    fn reversal_reports_dir_setup_and_preserves_mapper_phase() {
        let mut stepper = configured_stepper(3_200);
        stepper.reset_step_clock(0);
        stepper.load_move(StepMove::new(100, 1, 0));
        let forward = stepper.step_event().event();
        let phase_after_forward = stepper.mapper_phase().unwrap();

        stepper.mark_idle();
        stepper.reset_step_clock(1_000);
        stepper.set_next_step_dir(false);
        let reverse = stepper.prepare_move(StepMove::new(100, 1, 0));
        stepper.load_move(reverse);
        let reverse = stepper.step_event().event();

        assert!(forward.direction);
        assert!(!reverse.direction);
        assert!(reverse.dir_setup_required);
        assert_eq!(stepper.mapper_phase(), Some(1_600));
        assert_ne!(phase_after_forward, stepper.mapper_phase().unwrap());
    }

    #[test]
    fn first_move_reports_dir_setup_even_when_forward() {
        let mut stepper = configured_stepper(3_200);
        stepper.load_move(StepMove::new(100, 1, 0));

        let event = stepper.step_event().event();

        assert!(event.direction);
        assert!(event.dir_setup_required);
    }

    #[test]
    fn reset_requires_dir_setup_for_the_cached_direction() {
        let mut stepper = configured_stepper(3_200);
        stepper.load_move(StepMove::new(100, 1, 0));
        stepper.step_event();
        stepper.reset();
        stepper.load_move(StepMove::new(100, 1, 0));

        let event = stepper.step_event().event();

        assert!(event.direction);
        assert!(event.dir_setup_required);
    }

    #[test]
    fn reset_and_idle_preserve_mapper_phase() {
        let mut stepper = configured_stepper(3_200);
        stepper.load_move(StepMove::new(100, 1, 0));
        stepper.step_event();
        let phase = stepper.mapper_phase();

        stepper.mark_idle();
        stepper.reset_step_clock(123);
        stepper.reset();

        assert_eq!(stepper.mapper_phase(), phase);
    }

    #[test]
    fn mapper_reconfiguration_is_rejected_while_loaded() {
        let mut stepper = configured_stepper(3_200);
        stepper.load_move(StepMove::new(100, 1, 0));
        let replacement = PulseMapperConfig::new(4_000).unwrap();

        assert_eq!(
            stepper.configure_motion_scale(replacement),
            Err(MotionScaleConfigError::MoveLoaded)
        );
        assert_eq!(stepper.planner_steps_per_rev(), Some(3_200));
    }

    #[test]
    fn clearing_motion_scale_is_idempotent_and_unconfigures_accessors() {
        let mut stepper = configured_stepper(3_200);

        stepper.clear_motion_scale();

        assert!(!stepper.motion_scale_configured());
        assert_eq!(stepper.planner_steps_per_rev(), None);
        assert_eq!(stepper.mapper_phase(), None);

        stepper.clear_motion_scale();
        assert!(!stepper.motion_scale_configured());
    }

    #[test]
    fn clearing_loaded_motion_scale_stays_fail_closed_until_reset() {
        let mut stepper = configured_stepper(3_200);
        stepper.load_move(StepMove::new(100, 1, 0));
        stepper.clear_motion_scale();
        let replacement = PulseMapperConfig::new(4_000).unwrap();

        assert_eq!(
            stepper.configure_motion_scale(replacement),
            Err(MotionScaleConfigError::MoveLoaded)
        );
        assert!(!stepper.motion_scale_configured());
        assert_eq!(stepper.planner_steps_per_rev(), None);
        assert_eq!(stepper.mapper_phase(), None);

        stepper.reset();
        stepper.configure_motion_scale(replacement).unwrap();
        assert_eq!(stepper.planner_steps_per_rev(), Some(4_000));
        assert_eq!(stepper.mapper_phase(), Some(2_000));
    }

    #[test]
    fn minimum_interval_uses_acceleration_endpoint() {
        assert_eq!(StepMove::new(100, 5, 20).minimum_interval(), 100);
        assert_eq!(StepMove::new(100, 5, -20).minimum_interval(), 20);
        assert_eq!(StepMove::new(50, u16::MAX, i16::MIN).minimum_interval(), 1);
    }

    #[test]
    fn logical_clock_wraps_without_physical_pulse_offsets() {
        let mut stepper = configured_stepper(3_200);
        stepper.reset_step_clock(0xffff_ff00);
        stepper.load_move(StepMove::new(0x100, 2, 0));

        assert_eq!(stepper.next_step_clock(), 0);
        assert_eq!(stepper.step_event().next_clock(), Some(0x100));
    }

    #[test]
    fn deadline_policy_distinguishes_idle_and_contiguous_moves() {
        let mut stepper = configured_stepper(3_200);
        assert_eq!(stepper.check_deadline(100, 5_000, 84_000_000), Ok(5_002));

        stepper.reset_step_clock(0);
        stepper.load_move(StepMove::new(100, 1, 0));
        stepper.step_event();
        stepper.load_move(StepMove::new(100, 1, 0));
        assert_eq!(
            stepper.check_deadline(200, 100_200, 84_000_000),
            Err(MissedDeadline)
        );

        stepper.mark_idle();
        assert_eq!(
            stepper.check_deadline(200, 100_200, 84_000_000),
            Ok(100_202)
        );
    }

    #[test]
    fn active_stepper_preserves_future_deadline() {
        let mut stepper = configured_stepper(3_200);
        stepper.load_move(StepMove::new(100, 1, 0));
        stepper.step_event();

        assert_eq!(stepper.check_deadline(1_100, 1_000, 84_000_000), Ok(1_100));
    }

    #[test]
    fn active_stepper_nudges_slightly_past_deadline() {
        let mut stepper = configured_stepper(3_200);
        stepper.load_move(StepMove::new(100, 1, 0));
        stepper.step_event();

        assert_eq!(stepper.check_deadline(999, 1_000, 84_000_000), Ok(1_002));
    }

    #[test]
    fn active_stepper_compares_deadlines_across_clock_wrap() {
        let mut stepper = configured_stepper(3_200);
        stepper.load_move(StepMove::new(100, 1, 0));
        stepper.step_event();

        assert_eq!(
            stepper.check_deadline(0x0000_0010, 0xffff_fff0, 84_000_000),
            Ok(0x0000_0010)
        );
        assert_eq!(
            stepper.check_deadline(0xffff_fff0, 0x0000_0010, 84_000_000),
            Ok(0x0000_0012)
        );
    }

    #[test]
    fn reset_clears_loaded_move_but_not_motion_scale() {
        let mut stepper = configured_stepper(3_200);
        stepper.load_move(StepMove::new(100, 2, 0));
        let phase = stepper.mapper_phase();
        stepper.reset();

        assert!(!stepper.has_active_segment());
        assert!(stepper.motion_scale_configured());
        assert_eq!(stepper.planner_steps_per_rev(), Some(3_200));
        assert_eq!(stepper.mapper_phase(), phase);
    }
}
