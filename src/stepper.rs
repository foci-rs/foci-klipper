//! Stepper motor implementations for Klipper MCU protocol.
//!
//! Two implementations:
//! - `EmulatedStepper`: Software position accumulation via SPI (legacy).
//! - `HardwareStepper`: Timer-driven GPIO step/dir pulses for TMC4671 hardware
//!   step counter. Matches Klipper `stepper.c` scheduling semantics.

use heapless::Deque;

/// A single move segment from Klipper's `queue_step` command.
#[derive(Debug, Clone, Copy, defmt::Format)]
pub struct StepMove {
    /// Clock ticks between steps (initial value for this segment).
    pub interval: u32,
    /// Number of steps in this segment.
    pub count: u16,
    /// Interval change per step (acceleration).
    pub add: i16,
    /// Whether direction changed relative to the previous move.
    /// Set by `queue_move` when `last_dir != next_dir` (matches
    /// Klipper's `MF_DIR` flag). When true, `advance()` negates
    /// the stepper's direction on segment load.
    dir_changed: bool,
}

impl StepMove {
    /// Create a new move segment. The `dir_changed` flag is set
    /// automatically by `EmulatedStepper::queue_move`.
    pub const fn new(interval: u32, count: u16, add: i16) -> Self {
        Self {
            interval,
            count,
            add,
            dir_changed: false,
        }
    }
}

/// Emulates a stepper motor for Klipper.
///
/// Converts Klipper's `(interval, count, add)` step commands into
/// accumulated position values suitable for the TMC4671's position target.
pub struct EmulatedStepper {
    /// Queued move segments.
    moves: Deque<StepMove, 1024>,
    /// Current step interval (ticks between steps).
    interval: u32,
    /// Steps remaining in current segment.
    count: u16,
    /// Interval change per step.
    add: i16,
    /// Clock time of the next step.
    next_step_clock: u32,
    /// Accumulated position in steps (signed, bidirectional).
    position: i32,
    /// Current direction: 1 for forward, -1 for backward.
    direction: i32,
    /// Direction of the most recently queued move (Klipper `SF_LAST_DIR`).
    last_dir: bool,
    /// Direction set by `set_direction` for the next queued move
    /// (Klipper `SF_NEXT_DIR`).
    next_dir: bool,
    /// Step width in TMC4671 position units per step.
    step_width: i32,
    /// Whether the stepper is enabled.
    enabled: bool,
}

impl Default for EmulatedStepper {
    fn default() -> Self {
        Self::new()
    }
}

impl EmulatedStepper {
    /// Create a new `EmulatedStepper` in the disabled state with default settings.
    pub const fn new() -> Self {
        Self {
            moves: Deque::new(),
            interval: 0,
            count: 0,
            add: 0,
            next_step_clock: 0,
            position: 0,
            direction: 1,
            last_dir: true,
            next_dir: true,
            step_width: 1,
            enabled: false,
        }
    }

    /// Set step width (TMC4671 position units per Klipper step).
    pub fn set_step_width(&mut self, width: i32) {
        self.step_width = width;
    }

    /// Returns the current step width (TMC4671 position units per Klipper step).
    pub fn step_width(&self) -> i32 {
        self.step_width
    }

    /// Set direction for subsequently queued moves.
    ///
    /// The direction is captured into each `StepMove` at queue time.
    /// Matches Klipper's `set_next_step_dir` command.
    pub fn set_direction(&mut self, forward: bool) {
        self.next_dir = forward;
    }

    /// Reset the step clock reference to `clock`.
    ///
    /// The next step in any subsequently loaded segment will be scheduled
    /// relative to this value.
    pub fn reset_clock(&mut self, clock: u32) {
        self.next_step_clock = clock;
    }

    /// Enqueue a move segment.
    ///
    /// Captures the current direction state: if `next_dir` differs from
    /// `last_dir`, the move is flagged with `dir_changed` and `last_dir`
    /// is updated. Matches Klipper's `command_queue_step` logic.
    ///
    /// Returns `Err(m)` if the queue is full (capacity 1024).
    pub fn queue_move(&mut self, m: StepMove) -> Result<(), StepMove> {
        let dir_changed = self.last_dir != self.next_dir;
        if dir_changed {
            self.last_dir = self.next_dir;
        }
        let m = StepMove { dir_changed, ..m };
        self.moves.push_back(m).map_err(|_| m)
    }

    /// Get current position in raw step counts.
    pub fn position(&self) -> i32 {
        self.position
    }

    /// Get current position scaled to TMC4671 units.
    ///
    /// Returns `position * step_width`, using wrapping multiplication.
    pub fn position_target(&self) -> i32 {
        self.position.wrapping_mul(self.step_width)
    }

    /// Advance the stepper up to the given clock time.
    ///
    /// Processes queued moves, accumulating position for each step whose
    /// scheduled time has passed. Clock comparisons use signed 32-bit
    /// arithmetic to handle wraparound correctly.
    ///
    /// Returns `true` if any steps were taken.
    pub fn advance(&mut self, now: u32) -> bool {
        if !self.enabled {
            return false;
        }

        let mut stepped = false;

        loop {
            // If the current segment is exhausted, load the next one.
            if self.count == 0 {
                match self.moves.pop_front() {
                    Some(m) => {
                        self.interval = m.interval;
                        self.count = m.count;
                        self.add = m.add;
                        if m.dir_changed {
                            self.direction = -self.direction;
                        }
                        if self.count == 0 {
                            continue;
                        }
                        self.next_step_clock = self.next_step_clock.wrapping_add(self.interval);
                    }
                    None => break,
                }
            }

            // Check if the next step time has passed (signed wraparound comparison).
            let diff = (now as i32).wrapping_sub(self.next_step_clock as i32);
            if diff < 0 {
                break; // Next step is in the future.
            }

            // Execute one step.
            self.position = self.position.wrapping_add(self.direction);
            self.count -= 1;
            stepped = true;

            if self.count > 0 {
                self.interval = (self.interval as i32)
                    .saturating_add(self.add as i32)
                    .max(1) as u32;
                self.next_step_clock = self.next_step_clock.wrapping_add(self.interval);
            }
        }

        stepped
    }

    /// Enable or disable the stepper.
    ///
    /// A disabled stepper ignores all `advance` calls.
    pub fn set_enabled(&mut self, enabled: bool) {
        self.enabled = enabled;
    }

    /// Returns `true` if the stepper is currently enabled.
    pub fn enabled(&self) -> bool {
        self.enabled
    }

    /// Reset position to zero, stop all motion, and clear the move queue.
    ///
    /// Used when the hardware position reference is zeroed (e.g. during
    /// calibration or motor disable) so the stepper accumulator matches.
    pub fn reset_position(&mut self) {
        self.position = 0;
        self.stop();
    }

    /// Stop all motion and clear the move queue.
    ///
    /// The accumulated position is preserved. Any in-flight segment and all
    /// queued segments are discarded.
    pub fn stop(&mut self) {
        self.count = 0;
        self.interval = 0;
        self.add = 0;
        while self.moves.pop_front().is_some() {}
    }

    /// Returns true if the move queue is empty and no segment is in progress.
    pub fn move_queue_empty(&self) -> bool {
        self.moves.is_empty() && self.count == 0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn constant_velocity_steps() {
        let mut s = EmulatedStepper::new();
        s.set_enabled(true);
        s.reset_clock(0);
        s.set_step_width(10);

        s.queue_move(StepMove::new(100, 5, 0)).unwrap();
        s.advance(550);
        assert_eq!(s.position(), 5);
        assert_eq!(s.position_target(), 50);
    }

    #[test]
    fn direction_change() {
        let mut s = EmulatedStepper::new();
        s.set_enabled(true);
        s.reset_clock(0);

        s.queue_move(StepMove::new(10, 3, 0)).unwrap();
        s.advance(100);
        assert_eq!(s.position(), 3);

        s.set_direction(false);
        s.queue_move(StepMove::new(10, 2, 0)).unwrap();
        s.advance(200);
        assert_eq!(s.position(), 1);
    }

    #[test]
    fn accelerating_steps() {
        let mut s = EmulatedStepper::new();
        s.set_enabled(true);
        s.reset_clock(0);

        // 3 steps: intervals 100, 80, 60 (add = -20)
        s.queue_move(StepMove::new(100, 3, -20)).unwrap();

        assert!(!s.advance(99));
        assert!(s.advance(100));
        assert_eq!(s.position(), 1);

        assert!(s.advance(180));
        assert_eq!(s.position(), 2);

        assert!(s.advance(240));
        assert_eq!(s.position(), 3);
    }

    #[test]
    fn disabled_stepper_does_not_advance() {
        let mut s = EmulatedStepper::new();
        s.reset_clock(0);
        s.queue_move(StepMove::new(10, 5, 0)).unwrap();

        s.advance(100);
        assert_eq!(s.position(), 0);
    }

    #[test]
    fn queue_overflow_returns_error() {
        let mut s = EmulatedStepper::new();
        for _ in 0..1024 {
            s.queue_move(StepMove::new(10, 1, 0)).unwrap();
        }
        let result = s.queue_move(StepMove::new(10, 1, 0));
        assert!(result.is_err());
    }

    #[test]
    fn interval_underflow_clamps_to_one() {
        let mut s = EmulatedStepper::new();
        s.set_enabled(true);
        s.reset_clock(0);

        // interval=50, add=-100: without clamp, wraps to u32::MAX
        s.queue_move(StepMove::new(50, 3, -100)).unwrap();

        // Should complete all 3 steps quickly, not stall for ~25s
        s.advance(200);
        assert_eq!(s.position(), 3);
    }

    #[test]
    fn stop_clears_queue() {
        let mut s = EmulatedStepper::new();
        s.set_enabled(true);
        s.reset_clock(0);
        s.queue_move(StepMove::new(10, 100, 0)).unwrap();

        s.advance(50);
        let pos = s.position();
        assert!(pos > 0);

        s.stop();
        s.advance(10000);
        assert_eq!(s.position(), pos);
    }

    #[test]
    fn stop_preserves_position() {
        let mut s = EmulatedStepper::new();
        s.set_enabled(true);
        s.reset_clock(0);
        s.queue_move(StepMove::new(10, 5, 0)).unwrap();

        s.advance(50);
        let pos = s.position();
        assert_eq!(pos, 5);

        s.stop();
        assert_eq!(s.position(), 5, "stop() must preserve accumulated position");
    }

    #[test]
    fn stop_reports_queue_empty() {
        let mut s = EmulatedStepper::new();
        s.set_enabled(true);
        s.reset_clock(0);
        s.queue_move(StepMove::new(10, 100, 0)).unwrap();
        s.queue_move(StepMove::new(5, 50, 0)).unwrap();

        // Advance partway through the first segment so count > 0.
        s.advance(50);
        assert!(!s.move_queue_empty(), "queue must not be empty mid-motion");

        s.stop();
        assert!(
            s.move_queue_empty(),
            "move_queue_empty() must be true after stop()"
        );
    }

    #[test]
    fn reset_position_zeros_and_clears_queue() {
        let mut s = EmulatedStepper::new();
        s.set_enabled(true);
        s.reset_clock(0);
        s.set_step_width(10);

        s.queue_move(StepMove::new(10, 50, 0)).unwrap();
        s.queue_move(StepMove::new(10, 50, 0)).unwrap();
        s.advance(200);
        assert!(s.position() > 0);

        s.reset_position();
        assert_eq!(s.position(), 0);
        assert_eq!(s.position_target(), 0);
        assert!(s.move_queue_empty());

        // Advance after reset must not move.
        s.advance(10000);
        assert_eq!(s.position(), 0);
    }

    #[test]
    fn stop_on_idle_stepper_is_noop() {
        let mut s = EmulatedStepper::new();
        s.set_enabled(true);

        // stop() on an already-idle stepper must not panic or corrupt state.
        s.stop();
        assert_eq!(s.position(), 0);
        assert!(s.move_queue_empty());
    }

    #[test]
    fn move_queue_empty_false_when_moves_queued() {
        let mut s = EmulatedStepper::new();
        s.set_enabled(true);
        s.reset_clock(0);

        assert!(s.move_queue_empty(), "fresh stepper queue must be empty");

        s.queue_move(StepMove::new(100, 10, 0)).unwrap();

        assert!(
            !s.move_queue_empty(),
            "queue must not be empty after queue_move"
        );
    }

    #[test]
    fn move_queue_empty_true_on_fresh_stepper() {
        let s = EmulatedStepper::new();
        assert!(
            s.move_queue_empty(),
            "fresh stepper must report queue empty"
        );
    }

    #[test]
    fn multiple_segments() {
        let mut s = EmulatedStepper::new();
        s.set_enabled(true);
        s.reset_clock(0);

        // Two segments back to back.
        s.queue_move(StepMove::new(100, 3, 0)).unwrap();
        s.queue_move(StepMove::new(50, 2, 0)).unwrap();

        s.advance(1000);
        assert_eq!(s.position(), 5);
    }

    #[test]
    fn clock_wraparound() {
        let mut s = EmulatedStepper::new();
        s.set_enabled(true);
        s.reset_clock(0xFFFF_FF00);

        s.queue_move(StepMove::new(100, 3, 0)).unwrap();

        // Steps at 0xFFFFFF64, 0x28, 0x8C (wrapping around)
        s.advance(0x0000_0100);
        assert_eq!(s.position(), 3);
    }
}

// ---------------------------------------------------------------------------
// Hardware step/dir stepper
// ---------------------------------------------------------------------------

/// Hardware abstraction for step/dir GPIO and timer control.
///
/// Implemented by board crates for their specific timer + GPIO combination.
/// All methods are called from interrupt context (P3) or with interrupts
/// disabled, so implementations must be fast and non-blocking.
pub trait StepperPin {
    /// Toggle the STEP GPIO pin (both-edge stepping).
    fn step_toggle(&mut self);
    /// Set the DIR GPIO pin level.
    fn set_direction(&mut self, forward: bool);
    /// Schedule a timer compare interrupt at `clock`.
    fn schedule_compare(&mut self, clock: u32);
    /// Disable the timer compare interrupt (quiesce P3).
    fn disable_compare_interrupt(&mut self);
    /// Enable the timer compare interrupt.
    fn enable_compare_interrupt(&mut self);
    /// Read the current timer counter value.
    fn read_clock(&self) -> u32;
}

/// Result of a single step event in the timer ISR.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StepEventResult {
    /// More steps remain. Schedule next compare at `clock`.
    ScheduleNext { clock: u32 },
    /// Current segment exhausted. Caller should dequeue next move.
    NeedMove,
}

/// Timer-driven hardware stepper scheduling engine.
///
/// This struct holds two categories of state:
/// - **ISR state** (`current_*`, `next_step_clock`, `was_active`, `dir_state`):
///   Only accessed by the P3 timer ISR via `load_move()` and `step_event()`.
/// - **P2 state** (`last_dir`, `next_dir`): Only accessed by P2 command
///   handlers via `set_next_step_dir()` and `prepare_move()`.
///
/// The move queue and atomic flags live outside this struct as statics
/// in the board crate (see spec: Concurrency Model).
pub struct HardwareStepper {
    // --- ISR state (P3 only) ---
    /// Current segment interval (ticks between steps).
    current_interval: u32,
    /// Steps remaining in current segment.
    current_count: u16,
    /// Interval change per step (acceleration).
    current_add: i16,
    /// Clock time of the next step.
    next_step_clock: u32,
    /// Whether the stepper was active before loading the current segment.
    /// Used for missed-deadline check (Klipper `was_active` semantics).
    was_active: bool,
    /// Current DIR pin state (true = forward).
    dir_state: bool,

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

    /// Prepare a move for queuing by attaching direction state.
    ///
    /// Called from P2 `queue_step` command handler. Returns the amended
    /// `StepMove` with `dir_changed` set if direction changed since the
    /// last queued move. The caller pushes the result to the SPSC queue.
    pub fn prepare_move(&mut self, m: StepMove) -> StepMove {
        let dir_changed = self.last_dir != self.next_dir;
        if dir_changed {
            self.last_dir = self.next_dir;
        }
        StepMove { dir_changed, ..m }
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

    // --- P3 methods (timer ISR) ---

    /// Load a move segment from the queue into the active segment.
    ///
    /// Called from P3 timer ISR when the previous segment is exhausted.
    /// Sets `was_active` based on whether there was a previous segment.
    /// Returns `true` if direction changed (caller must set DIR GPIO
    /// before the next step pulse).
    pub fn load_move(&mut self, m: StepMove) -> bool {
        self.was_active = self.was_active || self.current_count > 0;
        self.current_interval = m.interval;
        self.current_count = m.count;
        self.current_add = m.add;
        self.next_step_clock = self.next_step_clock.wrapping_add(m.interval);
        if m.dir_changed {
            self.dir_state = !self.dir_state;
        }
        m.dir_changed
    }

    /// Execute one step and advance scheduling.
    ///
    /// Called from P3 timer ISR after toggling the STEP GPIO and
    /// incrementing the position atomic. Returns what the ISR should
    /// do next: schedule the next compare, or dequeue a new move.
    ///
    /// Caller is responsible for:
    /// 1. Toggling STEP GPIO before calling this
    /// 2. Incrementing position atomic before calling this
    pub fn step_event(&mut self) -> StepEventResult {
        self.current_count -= 1;

        if self.current_count > 0 {
            // Advance interval by acceleration, clamp to minimum 1.
            self.current_interval = (self.current_interval as i32)
                .saturating_add(self.current_add as i32)
                .max(1) as u32;
            self.next_step_clock = self.next_step_clock.wrapping_add(self.current_interval);
            StepEventResult::ScheduleNext {
                clock: self.next_step_clock,
            }
        } else {
            StepEventResult::NeedMove
        }
    }

    /// Check if the scheduled step time has passed acceptable bounds.
    ///
    /// Matches Klipper `stepper_load_next()` missed-deadline logic:
    /// - `was_active = false` (first activation): no check, return Ok(clock)
    /// - `was_active = true` and >1ms past: return Err (caller shuts down)
    /// - `was_active = true` and 0-1ms past: nudge to `now + min_ticks`
    ///
    /// `clock_freq` is the timer frequency (84 MHz for TIM5). The 1ms
    /// threshold is `clock_freq / 1000`.
    pub fn check_deadline(&self, scheduled: u32, now: u32, clock_freq: u32) -> Result<u32, ()> {
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
            return Err(());
        }

        // Within 1ms: nudge to now + minimum offset.
        Ok(now.wrapping_add(2))
    }

    /// Returns the current DIR state.
    pub fn dir_state(&self) -> bool {
        self.dir_state
    }

    /// Returns whether the active segment has steps remaining.
    pub fn has_active_segment(&self) -> bool {
        self.current_count > 0
    }
}

#[cfg(test)]
mod hardware_stepper_tests {
    use super::*;

    /// Mock StepperPin for host-side testing.
    struct MockPin {
        step_count: u32,
        direction: bool,
        last_compare: u32,
        compare_enabled: bool,
        clock: u32,
    }

    impl MockPin {
        fn new() -> Self {
            Self {
                step_count: 0,
                direction: true,
                last_compare: 0,
                compare_enabled: false,
                clock: 0,
            }
        }
    }

    impl StepperPin for MockPin {
        fn step_toggle(&mut self) {
            self.step_count += 1;
        }
        fn set_direction(&mut self, dir: bool) {
            self.direction = dir;
        }
        fn schedule_compare(&mut self, clock: u32) {
            self.last_compare = clock;
            self.compare_enabled = true;
        }
        fn disable_compare_interrupt(&mut self) {
            self.compare_enabled = false;
        }
        fn enable_compare_interrupt(&mut self) {
            self.compare_enabled = true;
        }
        fn read_clock(&self) -> u32 {
            self.clock
        }
    }

    #[test]
    fn prepare_move_sets_dir_changed() {
        let mut s = HardwareStepper::new();

        // Same direction: no change.
        let m = s.prepare_move(StepMove::new(100, 5, 0));
        assert!(!m.dir_changed);

        // Change direction.
        s.set_next_step_dir(false);
        let m = s.prepare_move(StepMove::new(100, 5, 0));
        assert!(m.dir_changed);

        // Same (now false): no change.
        let m = s.prepare_move(StepMove::new(100, 5, 0));
        assert!(!m.dir_changed);
    }

    #[test]
    fn load_move_and_step_event_counts_down() {
        let mut s = HardwareStepper::new();
        s.reset_step_clock(0);

        let m = StepMove::new(100, 3, 0);
        let m = StepMove {
            dir_changed: false,
            ..m
        };
        s.load_move(m);

        // First step.
        let result = s.step_event();
        assert_eq!(result, StepEventResult::ScheduleNext { clock: 200 });

        // Second step.
        let result = s.step_event();
        assert_eq!(result, StepEventResult::ScheduleNext { clock: 300 });

        // Third step: segment exhausted.
        let result = s.step_event();
        assert_eq!(result, StepEventResult::NeedMove);
    }

    #[test]
    fn step_event_applies_acceleration() {
        let mut s = HardwareStepper::new();
        s.reset_step_clock(0);

        // 3 steps: intervals 100, 80, 60 (add = -20)
        let m = StepMove::new(100, 3, -20);
        let m = StepMove {
            dir_changed: false,
            ..m
        };
        s.load_move(m);

        let result = s.step_event();
        // After first step: interval becomes 80, next at 100+80=180
        assert_eq!(result, StepEventResult::ScheduleNext { clock: 180 });

        let result = s.step_event();
        // After second step: interval becomes 60, next at 180+60=240
        assert_eq!(result, StepEventResult::ScheduleNext { clock: 240 });

        let result = s.step_event();
        assert_eq!(result, StepEventResult::NeedMove);
    }

    #[test]
    fn interval_clamps_to_minimum_one() {
        let mut s = HardwareStepper::new();
        s.reset_step_clock(0);

        // interval=50, add=-100: would go negative without clamp
        let m = StepMove::new(50, 3, -100);
        let m = StepMove {
            dir_changed: false,
            ..m
        };
        s.load_move(m);

        // First step: interval 50 -> schedule at 50+50=100.
        // Then interval becomes max(50-100, 1) = 1.
        let result = s.step_event();
        // Next at 50 + 1 = 51? No. Let me trace:
        // load_move: next_step_clock = 0 + 50 = 50 (first step)
        // step_event: count 3->2, interval = max(50-100,1)=1, next = 50+1 = 51
        assert_eq!(result, StepEventResult::ScheduleNext { clock: 51 });

        // Second step: interval 1 -> max(1-100,1)=1, next = 51+1 = 52
        let result = s.step_event();
        assert_eq!(result, StepEventResult::ScheduleNext { clock: 52 });

        let result = s.step_event();
        assert_eq!(result, StepEventResult::NeedMove);
    }

    #[test]
    fn load_move_direction_change() {
        let mut s = HardwareStepper::new();
        s.reset_step_clock(0);

        // First move: no dir change.
        let m = StepMove {
            dir_changed: false,
            ..StepMove::new(100, 1, 0)
        };
        assert!(!s.load_move(m));
        assert!(s.dir_state());

        // Second move: direction changes.
        let m = StepMove {
            dir_changed: true,
            ..StepMove::new(100, 1, 0)
        };
        assert!(s.load_move(m));
        assert!(!s.dir_state());

        // Third move: changes back.
        let m = StepMove {
            dir_changed: true,
            ..StepMove::new(100, 1, 0)
        };
        assert!(s.load_move(m));
        assert!(s.dir_state());
    }

    #[test]
    fn check_deadline_first_activation_no_check() {
        let s = HardwareStepper::new();
        // was_active is false: any past time is OK.
        let result = s.check_deadline(100, 5000, 84_000_000);
        assert!(result.is_ok());
        // Should fire immediately (now + 2).
        assert_eq!(result.unwrap(), 5002);
    }

    #[test]
    fn check_deadline_future_step_ok() {
        let mut s = HardwareStepper::new();
        s.was_active = true;
        // Step is in the future.
        let result = s.check_deadline(5000, 1000, 84_000_000);
        assert_eq!(result, Ok(5000));
    }

    #[test]
    fn check_deadline_slightly_past_nudges() {
        let mut s = HardwareStepper::new();
        s.was_active = true;
        // Step is 100 ticks in the past (well within 1ms = 84000 ticks).
        let result = s.check_deadline(1000, 1100, 84_000_000);
        assert!(result.is_ok());
        assert_eq!(result.unwrap(), 1102); // now + 2
    }

    #[test]
    fn check_deadline_too_far_past_errors() {
        let mut s = HardwareStepper::new();
        s.was_active = true;
        // Step is 100000 ticks in the past (>1ms at 84 MHz).
        let result = s.check_deadline(1000, 101_000, 84_000_000);
        assert_eq!(result, Err(()));
    }

    #[test]
    fn clock_wraparound_scheduling() {
        let mut s = HardwareStepper::new();
        s.reset_step_clock(0xFFFF_FF00);

        let m = StepMove::new(0x100, 2, 0);
        let m = StepMove {
            dir_changed: false,
            ..m
        };
        s.load_move(m);

        // First step scheduled at 0xFFFFFF00 + 0x100 = 0x00000000 (wraps).
        let result = s.step_event();
        assert_eq!(result, StepEventResult::ScheduleNext { clock: 0x00000100 });
    }

    #[test]
    fn check_deadline_wraparound() {
        let s = HardwareStepper::new();
        // Scheduled at 0xFFFFFF00, now at 0x00000100 (wrapped past).
        // First activation: should be OK.
        let result = s.check_deadline(0xFFFF_FF00, 0x0000_0100, 84_000_000);
        assert!(result.is_ok());
    }

    #[test]
    fn has_active_segment() {
        let mut s = HardwareStepper::new();
        assert!(!s.has_active_segment());

        s.reset_step_clock(0);
        let m = StepMove {
            dir_changed: false,
            ..StepMove::new(100, 2, 0)
        };
        s.load_move(m);
        assert!(s.has_active_segment());

        s.step_event();
        assert!(s.has_active_segment());

        s.step_event(); // count -> 0
        assert!(!s.has_active_segment());
    }
}
