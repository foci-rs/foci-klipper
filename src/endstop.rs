//! Endstop state machine for Klipper homing.
//!
//! Implements debounced pin sampling during homing with TrSync trigger reporting,
//! ported from Klipper's `src/endstop.c`.
//!
//! # Sampling Algorithm
//!
//! The endstop uses a two-phase sampling approach:
//!
//! 1. **Scanning** (`sample_ticks`): The pin is sampled at a coarse rate. As soon as
//!    a sample matches `pin_value`, the state machine switches to oversampling mode.
//!
//! 2. **Oversampling** (`rest_ticks`): Consecutive matches are required to confirm the
//!    trigger. If the pin stops matching at any point, `consecutive` is reset and the
//!    state machine returns to scanning mode.
//!
//! Once `sample_count` consecutive matches are accumulated, [`SampleResult::Triggered`]
//! is returned and homing stops.

/// Result of a single endstop sample.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SampleResult {
    /// Homing is not active; sample was ignored.
    NotHoming,
    /// Homing is active and the trigger threshold has not been reached yet.
    Continue,
    /// The endstop has triggered after `sample_count` consecutive matches.
    Triggered {
        /// Trigger reason code to pass to TrSync.
        reason: u8,
        /// OID of the TrSync object to notify.
        trsync_oid: u8,
    },
}

/// Debounced endstop state machine.
///
/// Tracks homing state and performs consecutive-match debouncing. The caller is
/// responsible for scheduling sample calls at the appropriate clock times using
/// [`next_sample_clock`](Endstop::next_sample_clock).
#[derive(Debug, defmt::Format)]
pub struct Endstop {
    /// Pin state that indicates a trigger (active-high or active-low).
    pin_value: bool,
    /// Number of consecutive matching samples required to trigger.
    sample_count: u8,
    /// Ticks between oversampling checks (after first match, fast rate).
    /// Maps to Klipper C `sample_time` / protocol `sample_ticks`.
    sample_ticks: u32,
    /// Ticks between scanning attempts (no consecutive match, slow rate).
    /// Maps to Klipper C `rest_time` / protocol `rest_ticks`.
    rest_ticks: u32,
    /// Clock tick at which the next sample should be taken.
    next_sample_clock: u32,
    /// Current count of consecutive matching samples.
    consecutive: u8,
    /// Whether homing is currently active.
    homing: bool,
    /// OID of the TrSync object to notify on trigger, if any.
    trsync_oid: Option<u8>,
    /// Reason code to report when triggering TrSync.
    trigger_reason: u8,
}

impl Endstop {
    /// Create a new `Endstop` in the idle (not homing) state.
    pub const fn new() -> Self {
        Self {
            pin_value: false,
            sample_count: 0,
            sample_ticks: 0,
            rest_ticks: 0,
            next_sample_clock: 0,
            consecutive: 0,
            homing: false,
            trsync_oid: None,
            trigger_reason: 0,
        }
    }

    /// Returns `true` if homing is currently active.
    pub fn is_homing(&self) -> bool {
        self.homing
    }

    /// Returns the clock tick at which the next sample should be taken.
    ///
    /// Only meaningful when [`is_homing`](Self::is_homing) returns `true`.
    pub fn next_sample_clock(&self) -> u32 {
        self.next_sample_clock
    }

    /// Arm the endstop for homing.
    ///
    /// Configures all homing parameters and activates sampling. After this call,
    /// [`sample`](Self::sample) should be called at each scheduled clock tick.
    ///
    /// # Parameters
    ///
    /// - `clock`: Clock tick of the first sample.
    /// - `sample_ticks`: Ticks between samples during scanning.
    /// - `sample_count`: Consecutive matches required to trigger. A value of 0
    ///   stops homing (disarms the endstop without triggering).
    /// - `rest_ticks`: Ticks between samples during oversampling.
    /// - `pin_value`: The pin level that indicates a trigger.
    /// - `trsync_oid`: OID of the TrSync object to notify on trigger.
    /// - `trigger_reason`: Reason code to include in the TrSync trigger.
    #[allow(clippy::too_many_arguments)]
    pub fn start_homing(
        &mut self,
        clock: u32,
        sample_ticks: u32,
        sample_count: u8,
        rest_ticks: u32,
        pin_value: bool,
        trsync_oid: u8,
        trigger_reason: u8,
    ) {
        if sample_count == 0 {
            // A sample_count of zero disarms the endstop.
            self.homing = false;
            self.trsync_oid = None;
            return;
        }

        self.next_sample_clock = clock;
        self.sample_ticks = sample_ticks;
        self.sample_count = sample_count;
        self.rest_ticks = rest_ticks;
        self.pin_value = pin_value;
        self.trsync_oid = Some(trsync_oid);
        self.trigger_reason = trigger_reason;
        self.consecutive = sample_count;
        self.homing = true;
    }

    /// Process one pin sample at the given clock time.
    ///
    /// # Parameters
    ///
    /// - `pin_state`: Current logical state of the pin.
    /// - `clock`: Current clock tick (used to schedule the next sample).
    ///
    /// # Returns
    ///
    /// - [`SampleResult::NotHoming`] if homing is not active.
    /// - [`SampleResult::Triggered`] if `sample_count` consecutive matches have
    ///   been observed; homing is stopped automatically.
    /// - [`SampleResult::Continue`] otherwise; [`next_sample_clock`](Self::next_sample_clock)
    ///   has been updated for the next scheduled call.
    pub fn sample(&mut self, pin_state: bool, clock: u32) -> SampleResult {
        if !self.homing {
            return SampleResult::NotHoming;
        }

        if pin_state == self.pin_value {
            // Pin matches — accumulate toward trigger.
            self.consecutive -= 1;
            if self.consecutive == 0 {
                // Threshold reached: trigger and stop homing.
                let trsync_oid = self.trsync_oid.unwrap_or(0);
                let reason = self.trigger_reason;
                self.homing = false;
                self.consecutive = 0;
                return SampleResult::Triggered { reason, trsync_oid };
            }
            // Schedule next oversampling check (fast rate, matches C sample_time).
            self.next_sample_clock = clock.wrapping_add(self.sample_ticks);
        } else {
            // Pin does not match — reset consecutive counter and return to scanning
            // (slow rate, matches C rest_time).
            self.consecutive = self.sample_count;
            self.next_sample_clock = clock.wrapping_add(self.rest_ticks);
        }

        SampleResult::Continue
    }

    /// Stop homing and reset the consecutive match counter.
    ///
    /// Safe to call at any time, including when not homing.
    pub fn stop(&mut self) {
        self.homing = false;
        self.consecutive = 0;
    }

    /// Return the current homing state and next sample clock for a query response.
    ///
    /// Returns `(homing, next_sample_clock)`. Matches the data expected by
    /// Klipper's `endstop_state` response message.
    pub fn query_state(&self) -> (bool, u32) {
        (self.homing, self.next_sample_clock)
    }
}

impl Default for Endstop {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn armed_endstop() -> Endstop {
        let mut e = Endstop::new();
        e.start_homing(
            1000, // clock
            500,  // sample_ticks
            3,    // sample_count
            100,  // rest_ticks
            true, // pin_value (active-high)
            7,    // trsync_oid
            42,   // trigger_reason
        );
        e
    }

    #[test]
    fn new_endstop_not_homing() {
        let e = Endstop::new();
        assert!(!e.is_homing());
    }

    #[test]
    fn start_homing_sets_state() {
        let e = armed_endstop();
        assert!(e.is_homing());
        assert_eq!(e.next_sample_clock(), 1000);
    }

    #[test]
    fn start_homing_zero_sample_count_disarms() {
        let mut e = armed_endstop();
        assert!(e.is_homing());
        e.start_homing(0, 500, 0, 100, true, 7, 42);
        assert!(!e.is_homing());
    }

    #[test]
    fn sample_not_homing_returns_not_homing() {
        let mut e = Endstop::new();
        let result = e.sample(true, 1000);
        assert_eq!(result, SampleResult::NotHoming);
    }

    #[test]
    fn sample_trigger_after_consecutive_matches() {
        let mut e = armed_endstop();

        // First match — not yet triggered (need 3)
        let r = e.sample(true, 1000);
        assert_eq!(r, SampleResult::Continue);
        assert!(e.is_homing());

        // Second match
        let r = e.sample(true, 1100);
        assert_eq!(r, SampleResult::Continue);
        assert!(e.is_homing());

        // Third match — triggers
        let r = e.sample(true, 1200);
        assert_eq!(
            r,
            SampleResult::Triggered {
                reason: 42,
                trsync_oid: 7,
            }
        );
        assert!(!e.is_homing());
    }

    #[test]
    fn sample_resets_counter_on_mismatch() {
        let mut e = armed_endstop();

        // Two matches
        assert_eq!(e.sample(true, 1000), SampleResult::Continue);
        assert_eq!(e.sample(true, 1100), SampleResult::Continue);

        // Mismatch resets — next sample uses rest_ticks (scanning rate)
        let r = e.sample(false, 1200);
        assert_eq!(r, SampleResult::Continue);
        assert!(e.is_homing());
        // After mismatch the next clock should be clock + rest_ticks
        assert_eq!(e.next_sample_clock(), 1200 + 100);

        // Need 3 more consecutive matches to trigger now
        assert_eq!(e.sample(true, 1700), SampleResult::Continue);
        assert_eq!(e.sample(true, 1800), SampleResult::Continue);
        assert_eq!(
            e.sample(true, 1900),
            SampleResult::Triggered {
                reason: 42,
                trsync_oid: 7,
            }
        );
    }

    #[test]
    fn next_sample_clock_uses_sample_ticks_on_match() {
        let mut e = armed_endstop();
        e.sample(true, 1000);
        // After a match, oversample at sample_ticks rate = 1000 + 500
        assert_eq!(e.next_sample_clock(), 1500);
    }

    #[test]
    fn next_sample_clock_uses_rest_ticks_on_mismatch() {
        let mut e = armed_endstop();
        e.sample(false, 1000);
        // After a mismatch, scan at rest_ticks rate = 1000 + 100
        assert_eq!(e.next_sample_clock(), 1100);
    }

    #[test]
    fn stop_clears_homing_state() {
        let mut e = armed_endstop();
        assert!(e.is_homing());
        e.stop();
        assert!(!e.is_homing());
    }

    #[test]
    fn stop_when_not_homing_is_safe() {
        let mut e = Endstop::new();
        e.stop(); // should not panic
        assert!(!e.is_homing());
    }

    #[test]
    fn query_state_returns_homing_and_clock() {
        let e = armed_endstop();
        let (homing, clock) = e.query_state();
        assert!(homing);
        assert_eq!(clock, 1000);
    }

    #[test]
    fn query_state_not_homing() {
        let e = Endstop::new();
        let (homing, _clock) = e.query_state();
        assert!(!homing);
    }

    #[test]
    fn sample_after_stop_returns_not_homing() {
        let mut e = armed_endstop();
        e.stop();
        let r = e.sample(true, 5000);
        assert_eq!(r, SampleResult::NotHoming);
    }

    #[test]
    fn clock_wraparound_sample_ticks() {
        let mut e = Endstop::new();
        e.start_homing(0xFFFF_FF00, 500, 3, 100, true, 1, 1);
        // Match at near-wrap clock; oversample at sample_ticks, should wrap
        e.sample(true, 0xFFFF_FF00);
        assert_eq!(e.next_sample_clock(), 0xFFFF_FF00u32.wrapping_add(500));
    }

    #[test]
    fn clock_wraparound_rest_ticks() {
        let mut e = Endstop::new();
        e.start_homing(0xFFFF_FF00, 500, 3, 100, true, 1, 1);
        // Mismatch at near-wrap clock; scan at rest_ticks, should wrap
        e.sample(false, 0xFFFF_FF00);
        assert_eq!(e.next_sample_clock(), 0xFFFF_FF00u32.wrapping_add(100));
    }

    #[test]
    fn default_is_same_as_new() {
        let a = Endstop::new();
        let b = Endstop::default();
        assert_eq!(a.is_homing(), b.is_homing());
        assert_eq!(a.next_sample_clock(), b.next_sample_clock());
    }

    #[test]
    fn triggered_stops_homing() {
        let mut e = armed_endstop();
        e.sample(true, 1000);
        e.sample(true, 1100);
        let r = e.sample(true, 1200);
        assert!(matches!(r, SampleResult::Triggered { .. }));
        // Subsequent samples after trigger return NotHoming
        assert_eq!(e.sample(true, 2000), SampleResult::NotHoming);
    }
}
