//! Stepper motor types for Klipper MCU protocol.
//!
//! - `StepMove`: Move segment passed through the SPSC queue to the timer ISR.
//! - `HardwareStepper`: Timer-driven GPIO step/dir scheduling engine.
//!   Matches Klipper `stepper.c` scheduling semantics.

/// Scheduled step time was more than 1ms in the past.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct MissedDeadline;

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
    /// Set by `HardwareStepper::prepare_move` when `last_dir != next_dir`
    /// (matches Klipper's `MF_DIR` flag). When true, `load_move()` toggles
    /// the stepper's direction state.
    dir_changed: bool,
}

impl StepMove {
    /// Create a new move segment with `dir_changed = false`.
    ///
    /// The `dir_changed` flag is set by `HardwareStepper::prepare_move`.
    pub const fn new(interval: u32, count: u16, add: i16) -> Self {
        Self {
            interval,
            count,
            add,
            dir_changed: false,
        }
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

/// Minimum timer ticks between a STEP rising edge (step phase) and the
/// following STEP falling edge (unstep phase).
///
/// The TMC4671-LA datasheet caps STEP input frequency at half the chip
/// oscillator (12.5 MHz for a 25 MHz crystal), which gives a ~40 ns
/// minimum pulse high time. At 84 MHz TIM5 clock, 8 ticks ≈ 95 ns — well
/// above the minimum with margin for propagation delay and ISR jitter.
///
/// This replaces the host-configurable `step_pulse_ticks` argument of
/// Klipper C's `config_stepper` (currently ignored by this firmware).
pub const STEP_PULSE_TICKS: u32 = 8;

/// Phase of a single timer event in full-mode stepping.
///
/// In full mode (`STEPPER_STEP_BOTH_EDGE = 0`), each motor step is split
/// across two timer events. The phase returned from `step_event()` tells
/// the ISR what action it just completed so position accounting and
/// scheduling can branch accordingly:
///
/// - `Step`: the GPIO toggle drove STEP from LOW to HIGH. This is the
///   rising edge the driver IC counts as a motor step, so the firmware
///   position counter should advance by `dir`. The next timer event is
///   scheduled after `STEP_PULSE_TICKS` to complete the pulse.
/// - `Unstep`: the GPIO toggle drove STEP from HIGH to LOW. No position
///   change. The next timer event is scheduled at the current step
///   interval to begin the next motor step.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum StepPhase {
    /// GPIO just transitioned LOW → HIGH (rising edge). Position advances.
    Step,
    /// GPIO just transitioned HIGH → LOW. Position unchanged.
    Unstep,
}

/// Result of a single step event in the timer ISR.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StepEventResult {
    /// More events remain. Schedule next compare at `clock`.
    ///
    /// `phase` describes the event just processed (the toggle the ISR
    /// already applied before calling `step_event`).
    ScheduleNext { clock: u32, phase: StepPhase },
    /// Current segment exhausted. Caller should dequeue next move.
    ///
    /// The ISR's GPIO toggle has already driven STEP LOW for this final
    /// unstep event, so the pin is in the correct quiescent state before
    /// the next move loads.
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
    /// Timer events remaining in current segment.
    ///
    /// Each motor step requires two timer events in full mode
    /// (STEPPER_STEP_BOTH_EDGE=0): one to drive STEP HIGH (step phase),
    /// one to drive STEP LOW (unstep phase). `load_move` stores
    /// `move.count * 2` here. A `u16` move count can reach 65_535, so
    /// the doubled value requires `u32`.
    current_count: u32,
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

    /// Reset the ISR stepper engine to idle state.
    ///
    /// Clears the active segment so the next ISR entry dequeues a fresh
    /// move. Called by `stop_physical_stepper_no_lock()` after disabling
    /// the step timer to prevent stale partial segments from executing
    /// when the timer is re-armed.
    pub fn reset(&mut self) {
        self.current_count = 0;
        self.current_interval = 0;
        self.current_add = 0;
        self.was_active = false;
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
        // Full-mode scheduling: two timer events per motor step (step HIGH,
        // unstep LOW). Double the count so the ISR fires twice per step.
        // Matches Klipper C stepper_event_full: `s->count = move_count * 2`.
        self.current_count = (m.count as u32) * 2;
        self.current_add = m.add;
        self.next_step_clock = self.next_step_clock.wrapping_add(m.interval);
        if m.dir_changed {
            self.dir_state = !self.dir_state;
        }
        m.dir_changed
    }

    /// Return the phase of the next full-mode timer event.
    ///
    /// Full-pulse scheduling stores two events per motor step. Before
    /// `step_event()` decrements the count, an even count means the next event
    /// is a rising edge and an odd count means the next event is a falling edge.
    pub fn next_event_phase(&self) -> Option<StepPhase> {
        if self.current_count == 0 {
            return None;
        }
        if self.current_count & 1 == 0 {
            Some(StepPhase::Step)
        } else {
            Some(StepPhase::Unstep)
        }
    }

    /// Clamp an event-level deadline to the minimum safe compare time.
    ///
    /// This mirrors Klipper C `stepper_event_full()` `reschedule_min`: a
    /// full-mode event that is late or too close is pushed to `now + min_ticks`
    /// instead of using the segment-load missed-deadline policy.
    pub fn clamp_event_deadline(&self, scheduled: u32, now: u32, min_ticks: u32) -> u32 {
        let min_clock = now.wrapping_add(min_ticks);
        let delta = scheduled.wrapping_sub(min_clock) as i32;
        if delta < 0 { min_clock } else { scheduled }
    }

    /// Execute one timer event and advance scheduling.
    ///
    /// Called from the P3 timer ISR after toggling the STEP GPIO. Returns
    /// what the ISR should do next: schedule the next compare (with the
    /// `StepPhase` of the event just processed), or dequeue a new move.
    ///
    /// The event phase is derived from the parity of `current_count`
    /// after decrement, matching Klipper C `stepper_event_full`:
    ///
    /// - odd after decrement → the GPIO just went HIGH (step phase); the
    ///   ISR must increment STEPPER_POSITION by `dir_state()`. The next
    ///   event (the unstep) is scheduled at `step_clock + STEP_PULSE_TICKS`
    ///   to complete the pulse; the base `next_step_clock` is unchanged
    ///   so the following step lands at `base + interval`, not `base +
    ///   interval + pulse_ticks`.
    /// - even after decrement → the GPIO just went LOW (unstep phase); no
    ///   position change. `next_step_clock` advances by `current_interval`
    ///   and acceleration (`current_add`) is applied. If the count reaches
    ///   zero, the segment is exhausted and `NeedMove` is returned.
    ///
    /// Caller is responsible for:
    /// 1. Toggling STEP GPIO before calling this
    /// 2. Inspecting `StepPhase::Step` in the returned `ScheduleNext` and
    ///    incrementing the position atomic only in that branch
    pub fn step_event(&mut self) -> StepEventResult {
        self.current_count -= 1;

        // Odd remaining count → we just drove STEP HIGH (step phase).
        // Even remaining count → we just drove STEP LOW (unstep phase).
        if self.current_count & 1 == 1 {
            // Step phase: schedule the pair-matching unstep. The next_step
            // clock for the *following* motor step stays at the current
            // value so that after the unstep advances by `interval`, the
            // following step lands at exactly `step_clock + interval`.
            let unstep_clock = self.next_step_clock.wrapping_add(STEP_PULSE_TICKS);
            StepEventResult::ScheduleNext {
                clock: unstep_clock,
                phase: StepPhase::Step,
            }
        } else {
            // Unstep phase.
            if self.current_count == 0 {
                // Final unstep of this segment — segment exhausted.
                self.was_active = true;
                return StepEventResult::NeedMove;
            }
            // Advance interval by acceleration (clamped to ≥ 1), then
            // advance next_step_clock by the new interval to schedule the
            // next step. Acceleration applies once per motor step (at the
            // unstep → next-step transition), matching Klipper semantics
            // where `s->interval += s->add` happens inside the `count` arm
            // of `stepper_event_full`.
            self.current_interval = (self.current_interval as i32)
                .saturating_add(self.current_add as i32)
                .max(1) as u32;
            self.next_step_clock = self.next_step_clock.wrapping_add(self.current_interval);
            StepEventResult::ScheduleNext {
                clock: self.next_step_clock,
                phase: StepPhase::Unstep,
            }
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

    /// Remaining timer events in the active segment.
    ///
    /// Full-mode stepping uses two timer events per motor step, so this
    /// value is twice the remaining motor-step count except when stopped
    /// between a rising and falling edge.
    pub fn active_event_count(&self) -> u32 {
        self.current_count
    }
}

#[cfg(test)]
mod hardware_stepper_tests {
    use super::*;

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
    fn event_deadline_late_clamps_without_missed_deadline() {
        let mut s = HardwareStepper::new();
        s.was_active = true;

        let now = 200_000;
        let scheduled = now - 100_000;

        assert_eq!(
            s.clamp_event_deadline(scheduled, now, STEP_PULSE_TICKS),
            now + STEP_PULSE_TICKS
        );
    }

    #[test]
    fn event_deadline_too_close_clamps_to_minimum_time() {
        let s = HardwareStepper::new();

        let now = 10_000;
        let scheduled = now + 3;

        assert_eq!(
            s.clamp_event_deadline(scheduled, now, STEP_PULSE_TICKS),
            now + STEP_PULSE_TICKS
        );
    }

    #[test]
    fn event_deadline_future_clock_is_preserved() {
        let s = HardwareStepper::new();

        let now = 10_000;
        let scheduled = now + 100;

        assert_eq!(
            s.clamp_event_deadline(scheduled, now, STEP_PULSE_TICKS),
            scheduled
        );
    }

    #[test]
    fn next_event_phase_matches_full_pulse_count_parity() {
        let mut s = HardwareStepper::new();
        s.reset_step_clock(0);
        s.load_move(StepMove::new(100, 2, 0));

        assert_eq!(s.next_event_phase(), Some(StepPhase::Step));
        assert!(matches!(
            s.step_event(),
            StepEventResult::ScheduleNext {
                phase: StepPhase::Step,
                ..
            }
        ));

        assert_eq!(s.next_event_phase(), Some(StepPhase::Unstep));
        assert!(matches!(
            s.step_event(),
            StepEventResult::ScheduleNext {
                phase: StepPhase::Unstep,
                ..
            }
        ));

        assert_eq!(s.next_event_phase(), Some(StepPhase::Step));
        assert!(matches!(
            s.step_event(),
            StepEventResult::ScheduleNext {
                phase: StepPhase::Step,
                ..
            }
        ));

        assert_eq!(s.next_event_phase(), Some(StepPhase::Unstep));
        assert_eq!(s.step_event(), StepEventResult::NeedMove);
        assert_eq!(s.next_event_phase(), None);
    }

    #[test]
    fn full_mode_step_event_alternates_step_and_unstep_phase() {
        // In full mode, each motor step is a pair of timer events:
        //   event 1 (step):   GPIO toggles LOW → HIGH (rising edge seen by driver)
        //   event 2 (unstep): GPIO toggles HIGH → LOW
        // The phase returned by step_event() tells the ISR which event it
        // just processed so it can update STEPPER_POSITION only on the
        // step (rising-edge) events.
        let mut s = HardwareStepper::new();
        s.reset_step_clock(0);
        s.load_move(StepMove {
            dir_changed: false,
            ..StepMove::new(100, 2, 0)
        });

        // 2-step move = 4 timer events: Step, Unstep, Step, Unstep.
        // The final unstep exhausts the segment, so step_event returns
        // NeedMove on the 4th call (the ISR still has the GPIO toggle
        // already applied, which drives STEP LOW).
        let expected = [StepPhase::Step, StepPhase::Unstep, StepPhase::Step];
        for (i, exp) in expected.iter().enumerate() {
            match s.step_event() {
                StepEventResult::ScheduleNext { phase, .. } => {
                    assert_eq!(phase, *exp, "wrong phase at event {}", i);
                }
                StepEventResult::NeedMove => {
                    panic!("unexpected NeedMove at event {}", i);
                }
            }
        }
        // 4th event (final unstep) exhausts the segment.
        assert_eq!(s.step_event(), StepEventResult::NeedMove);
    }

    #[test]
    fn full_mode_load_move_doubles_count() {
        // In full mode (STEPPER_STEP_BOTH_EDGE=0, matching Klipper C
        // stepper_event_full), each motor step requires two timer events:
        // one to set STEP HIGH (step), one to set it LOW (unstep).
        // load_move must double the internal count so the ISR fires
        // twice per motor step.
        //
        // For a 3-motor-step move, we expect 6 step_event calls before
        // the segment is exhausted.
        let mut s = HardwareStepper::new();
        s.reset_step_clock(0);
        s.load_move(StepMove {
            dir_changed: false,
            ..StepMove::new(100, 3, 0)
        });

        // 5 non-terminal events (step, unstep, step, unstep, step).
        for i in 0..5 {
            let result = s.step_event();
            assert!(
                matches!(result, StepEventResult::ScheduleNext { .. }),
                "expected ScheduleNext at iteration {} but got {:?}",
                i,
                result
            );
        }
        // 6th event is the final unstep — segment exhausted.
        assert_eq!(s.step_event(), StepEventResult::NeedMove);
    }

    #[test]
    fn step_phase_unstep_clock_does_not_advance_step_cadence_anchor() {
        // Cadence contract: `next_step_clock` is the base time for the
        // NEXT rising edge. The Step phase returns an `unstep_clock`
        // (step_clock + STEP_PULSE_TICKS) intended solely to close the
        // pulse — this clock is a few tens of nanoseconds wide and will
        // typically already be in the past by the time the ISR checks
        // its deadline. Even under that condition the Step phase must
        // NOT mutate the cadence anchor; the next rising edge must land
        // at `original_step_clock + interval`, not
        // `nudged_unstep_clock + interval`.
        //
        // This test locks in the stepper-core behaviour the ISR depends
        // on: step_event() during a Step event does not touch
        // next_step_clock, so calling step_event() again for the Unstep
        // phase (without any external rebase) schedules the following
        // rising edge from the original step_clock.
        let mut s = HardwareStepper::new();
        s.reset_step_clock(0);
        s.load_move(StepMove {
            dir_changed: false,
            ..StepMove::new(100, 2, 0)
        });
        // After load_move: next_step_clock = 0 + 100 = 100 (first rising edge).

        // Step 1 rising — unstep scheduled close to the rising edge.
        assert_eq!(
            s.step_event(),
            StepEventResult::ScheduleNext {
                clock: 100 + STEP_PULSE_TICKS,
                phase: StepPhase::Step,
            }
        );
        // Cadence anchor still at the rising-edge time, not the unstep time.
        assert_eq!(s.next_step_clock(), 100);

        // Step 1 falling — the next rising edge MUST be at
        // original_step_clock (100) + interval (100) = 200. If the
        // stepper were to anchor off the unstep clock we'd get
        // 108 + 100 = 208, permanently stretching every subsequent step
        // period by the pulse-close latency.
        assert_eq!(
            s.step_event(),
            StepEventResult::ScheduleNext {
                clock: 200,
                phase: StepPhase::Unstep,
            }
        );
        assert_eq!(s.next_step_clock(), 200);
    }

    #[test]
    fn load_move_and_step_event_counts_down() {
        let mut s = HardwareStepper::new();
        s.reset_step_clock(0);

        // count=3 motor steps → 6 timer events (step/unstep pairs).
        // Load sets next_step_clock = 0 + 100 = 100 (first step).
        let m = StepMove {
            dir_changed: false,
            ..StepMove::new(100, 3, 0)
        };
        s.load_move(m);

        // Event 1 — step 1 rising: unstep scheduled at 100 + STEP_PULSE_TICKS.
        assert_eq!(
            s.step_event(),
            StepEventResult::ScheduleNext {
                clock: 100 + STEP_PULSE_TICKS,
                phase: StepPhase::Step,
            }
        );
        // Event 2 — step 1 falling: next step at 100 + 100 = 200.
        assert_eq!(
            s.step_event(),
            StepEventResult::ScheduleNext {
                clock: 200,
                phase: StepPhase::Unstep,
            }
        );
        // Event 3 — step 2 rising.
        assert_eq!(
            s.step_event(),
            StepEventResult::ScheduleNext {
                clock: 200 + STEP_PULSE_TICKS,
                phase: StepPhase::Step,
            }
        );
        // Event 4 — step 2 falling: next step at 200 + 100 = 300.
        assert_eq!(
            s.step_event(),
            StepEventResult::ScheduleNext {
                clock: 300,
                phase: StepPhase::Unstep,
            }
        );
        // Event 5 — step 3 rising.
        assert_eq!(
            s.step_event(),
            StepEventResult::ScheduleNext {
                clock: 300 + STEP_PULSE_TICKS,
                phase: StepPhase::Step,
            }
        );
        // Event 6 — step 3 falling: segment exhausted.
        assert_eq!(s.step_event(), StepEventResult::NeedMove);
    }

    #[test]
    fn step_event_applies_acceleration() {
        // Acceleration (`add`) is applied once per motor step, at the
        // unstep → next-step transition (even-count branch). Step events
        // (odd-count branch) do not change the interval, so the pulse
        // width is always STEP_PULSE_TICKS regardless of acceleration.
        let mut s = HardwareStepper::new();
        s.reset_step_clock(0);

        // 3 motor steps with add = -20: intervals 100, 80, 60.
        let m = StepMove {
            dir_changed: false,
            ..StepMove::new(100, 3, -20)
        };
        s.load_move(m);

        // Step 1 rising (count 6→5): unstep at 100 + pulse.
        assert_eq!(
            s.step_event(),
            StepEventResult::ScheduleNext {
                clock: 100 + STEP_PULSE_TICKS,
                phase: StepPhase::Step,
            }
        );
        // Step 1 falling (count 5→4): interval becomes 80, next step at 180.
        assert_eq!(
            s.step_event(),
            StepEventResult::ScheduleNext {
                clock: 180,
                phase: StepPhase::Unstep,
            }
        );
        // Step 2 rising (count 4→3).
        assert_eq!(
            s.step_event(),
            StepEventResult::ScheduleNext {
                clock: 180 + STEP_PULSE_TICKS,
                phase: StepPhase::Step,
            }
        );
        // Step 2 falling (count 3→2): interval becomes 60, next step at 240.
        assert_eq!(
            s.step_event(),
            StepEventResult::ScheduleNext {
                clock: 240,
                phase: StepPhase::Unstep,
            }
        );
        // Step 3 rising (count 2→1).
        assert_eq!(
            s.step_event(),
            StepEventResult::ScheduleNext {
                clock: 240 + STEP_PULSE_TICKS,
                phase: StepPhase::Step,
            }
        );
        // Step 3 falling (count 1→0): segment exhausted.
        assert_eq!(s.step_event(), StepEventResult::NeedMove);
    }

    #[test]
    fn interval_clamps_to_minimum_one() {
        // Large negative `add` would drive interval below 1 without the
        // saturating_add + .max(1) clamp. The clamp is applied on every
        // unstep → next-step transition.
        let mut s = HardwareStepper::new();
        s.reset_step_clock(0);

        // interval=50, add=-100 (would become -50, clamped to 1).
        let m = StepMove {
            dir_changed: false,
            ..StepMove::new(50, 3, -100)
        };
        s.load_move(m);

        // Step 1 rising: unstep at 50 + pulse.
        assert_eq!(
            s.step_event(),
            StepEventResult::ScheduleNext {
                clock: 50 + STEP_PULSE_TICKS,
                phase: StepPhase::Step,
            }
        );
        // Step 1 falling: interval = max(50 - 100, 1) = 1. Next step at 50 + 1 = 51.
        assert_eq!(
            s.step_event(),
            StepEventResult::ScheduleNext {
                clock: 51,
                phase: StepPhase::Unstep,
            }
        );
        // Step 2 rising: unstep at 51 + pulse.
        assert_eq!(
            s.step_event(),
            StepEventResult::ScheduleNext {
                clock: 51 + STEP_PULSE_TICKS,
                phase: StepPhase::Step,
            }
        );
        // Step 2 falling: interval = max(1 - 100, 1) = 1 (still clamped).
        // Next step at 51 + 1 = 52.
        assert_eq!(
            s.step_event(),
            StepEventResult::ScheduleNext {
                clock: 52,
                phase: StepPhase::Unstep,
            }
        );
        // Step 3 rising: unstep at 52 + pulse.
        assert_eq!(
            s.step_event(),
            StepEventResult::ScheduleNext {
                clock: 52 + STEP_PULSE_TICKS,
                phase: StepPhase::Step,
            }
        );
        // Step 3 falling: segment exhausted.
        assert_eq!(s.step_event(), StepEventResult::NeedMove);
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
        assert_eq!(result, Err(MissedDeadline));
    }

    #[test]
    fn clock_wraparound_scheduling() {
        // Verify u32 wraparound propagates through both the step-phase
        // add (STEP_PULSE_TICKS) and the unstep-phase add (interval).
        let mut s = HardwareStepper::new();
        s.reset_step_clock(0xFFFF_FF00);

        let m = StepMove {
            dir_changed: false,
            ..StepMove::new(0x100, 2, 0)
        };
        s.load_move(m);
        // After load_move: next_step_clock = 0xFFFFFF00 + 0x100 = 0x00000000 (wrapped).

        // Step 1 rising: unstep clock = 0 + STEP_PULSE_TICKS (wrapped add).
        assert_eq!(
            s.step_event(),
            StepEventResult::ScheduleNext {
                clock: STEP_PULSE_TICKS,
                phase: StepPhase::Step,
            }
        );
        // Step 1 falling: next step clock = 0 + 0x100 = 0x100 (the original
        // wrap-case). Unstep phase re-exercises the wrapping add.
        assert_eq!(
            s.step_event(),
            StepEventResult::ScheduleNext {
                clock: 0x0000_0100,
                phase: StepPhase::Unstep,
            }
        );
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

        // 2 motor steps → 4 timer events. Segment is active until the 4th
        // (final unstep) decrements count to 0.
        s.reset_step_clock(0);
        let m = StepMove {
            dir_changed: false,
            ..StepMove::new(100, 2, 0)
        };
        s.load_move(m);
        assert!(s.has_active_segment());

        s.step_event(); // step 1 rising: count 4 → 3
        assert!(s.has_active_segment());
        s.step_event(); // step 1 falling: count 3 → 2
        assert!(s.has_active_segment());
        s.step_event(); // step 2 rising: count 2 → 1
        assert!(s.has_active_segment());

        s.step_event(); // step 2 falling: count 1 → 0
        assert!(!s.has_active_segment());
    }

    #[test]
    fn past_deadline_nudge_preserves_cadence_anchor() {
        // Klipper fires late events as soon as possible, but does not rebase
        // next_step_time. Keeping the stale cadence anchor is important after
        // a queue underrun: it lets the stepper catch up to the host schedule
        // instead of carrying the late arrival forward as permanent lag.
        let mut s = HardwareStepper::new();
        s.reset_step_clock(0);

        let m = StepMove {
            dir_changed: false,
            ..StepMove::new(100, 2, 0)
        };
        s.load_move(m);

        let adjusted = s.check_deadline(s.next_step_clock(), 1000, 84_000_000);
        assert_eq!(adjusted, Ok(1002));

        // Step 1 rising was executed late, but the unstep is still anchored
        // to the original first step time.
        assert_eq!(
            s.step_event(),
            StepEventResult::ScheduleNext {
                clock: 100 + STEP_PULSE_TICKS,
                phase: StepPhase::Step,
            }
        );
        // Step 1 falling keeps advancing from the original cadence anchor.
        assert_eq!(
            s.step_event(),
            StepEventResult::ScheduleNext {
                clock: 200,
                phase: StepPhase::Unstep,
            }
        );
    }

    #[test]
    fn contiguous_segment_load_keeps_active_deadline_checks() {
        let mut s = HardwareStepper::new();
        s.reset_step_clock(0);

        s.load_move(StepMove::new(100, 1, 0));
        assert_eq!(
            s.step_event(),
            StepEventResult::ScheduleNext {
                clock: 100 + STEP_PULSE_TICKS,
                phase: StepPhase::Step,
            }
        );
        assert_eq!(s.step_event(), StepEventResult::NeedMove);

        s.load_move(StepMove::new(100, 1, 0));
        assert_eq!(s.next_step_clock(), 200);

        // The second segment is contiguous with the just-finished segment.
        // Klipper's stepper_load_next() sees the previous count as still
        // active in this path, so a segment that is more than 1ms late must
        // trip the missed-deadline check instead of being treated as a fresh
        // idle activation.
        assert_eq!(
            s.check_deadline(200, 200 + 100_000, 84_000_000),
            Err(MissedDeadline)
        );
    }

    #[test]
    fn queue_empty_marks_next_segment_idle() {
        let mut s = HardwareStepper::new();
        s.reset_step_clock(0);

        s.load_move(StepMove::new(100, 1, 0));
        s.step_event();
        assert_eq!(s.step_event(), StepEventResult::NeedMove);
        s.mark_idle();

        s.load_move(StepMove::new(100, 1, 0));

        // After the queue actually went empty, the next queued segment is a
        // new activation and follows Klipper's no-check first-load behavior.
        assert_eq!(
            s.check_deadline(200, 200 + 100_000, 84_000_000),
            Ok(100_202)
        );
    }
}
