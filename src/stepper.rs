//! Emulated stepper motor for Klipper MCU protocol.
//!
//! Converts Klipper's `(interval, count, add)` step commands into accumulated
//! position values suitable for the TMC4671's position target register.

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
