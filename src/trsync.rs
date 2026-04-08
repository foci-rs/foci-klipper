//! TrSync (Trigger Synchronization) state machine.
//!
//! Coordinates synchronized stopping of steppers during homing. Mirrors the
//! logic from Klipper's C `trsync` implementation, adapted for `no_std` use
//! without signal callbacks (those are handled at the firmware level).

/// Operational state of a [`TrSync`] instance.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum TrSyncState {
    /// Not yet armed; ignores trigger calls.
    Idle,
    /// Armed and waiting for a trigger or timeout.
    Armed,
    /// Triggered with the given reason code.
    Triggered(u8),
}

/// TrSync (Trigger Synchronization) state machine.
///
/// Coordinates synchronized stopping of steppers during homing operations.
/// Provides clock-based timeout detection and periodic status reporting.
///
/// # Clock Arithmetic
///
/// All clock comparisons use signed 32-bit wraparound arithmetic via
/// `wrapping_sub` + `as i32` cast, matching Klipper's convention for
/// handling the 32-bit clock counter rollover.
pub struct TrSync {
    /// Current operational state.
    state: TrSyncState,
    /// Reason code reported when the timeout fires.
    expire_reason: u8,
    /// Absolute clock tick at which expiry triggers.
    expire_clock: u32,
    /// Absolute clock tick of the next required status report.
    report_clock: u32,
    /// Number of ticks between successive status reports.
    report_ticks: u32,
    /// Whether [`set_timeout`](Self::set_timeout) has been called since the
    /// last [`arm`](Self::arm). Mirrors Klipper C, where the expire timer
    /// is only added to the scheduler in `command_trsync_set_timeout`, not
    /// in `command_trsync_start`.
    timeout_set: bool,
}

impl TrSync {
    /// Create a new `TrSync` in the [`TrSyncState::Idle`] state.
    pub const fn new() -> Self {
        Self {
            state: TrSyncState::Idle,
            expire_reason: 0,
            expire_clock: 0,
            report_clock: 0,
            report_ticks: 0,
            timeout_set: false,
        }
    }

    /// Return the current state.
    pub fn state(&self) -> TrSyncState {
        self.state
    }

    /// Return the trigger reason if currently in the [`TrSyncState::Triggered`]
    /// state, or `0` otherwise.
    pub fn trigger_reason(&self) -> u8 {
        match self.state {
            TrSyncState::Triggered(reason) => reason,
            _ => 0,
        }
    }

    /// Arm the TrSync for homing.
    ///
    /// Transitions from any state to [`TrSyncState::Armed`] and configures
    /// the reporting interval and timeout reason.
    ///
    /// # Parameters
    ///
    /// - `report_clock`: Absolute clock tick for the first status report.
    /// - `report_ticks`: Interval (in ticks) between successive reports.
    /// - `expire_reason`: Reason code to use if a timeout fires.
    pub fn arm(&mut self, report_clock: u32, report_ticks: u32, expire_reason: u8) {
        self.state = TrSyncState::Armed;
        self.report_clock = report_clock;
        self.report_ticks = report_ticks;
        self.expire_reason = expire_reason;
        self.expire_clock = 0;
        // Suppress timeout until set_timeout() is called, matching Klipper C
        // where the expire timer is only started in command_trsync_set_timeout.
        self.timeout_set = false;
    }

    /// Trigger the TrSync with the given reason code.
    ///
    /// Only accepted when in the [`TrSyncState::Armed`] state. Returns `true`
    /// if the trigger was accepted, `false` if it was ignored (already
    /// triggered or idle).
    pub fn trigger(&mut self, reason: u8) -> bool {
        if self.state == TrSyncState::Armed {
            self.state = TrSyncState::Triggered(reason);
            true
        } else {
            false
        }
    }

    /// Update the expiry deadline.
    ///
    /// Sets the absolute clock tick at which [`check_timeout`](Self::check_timeout)
    /// will auto-trigger with `expire_reason`.
    pub fn set_timeout(&mut self, clock: u32) {
        self.expire_clock = clock;
        self.timeout_set = true;
    }

    /// Check whether the expiry deadline has passed.
    ///
    /// Returns `false` immediately if [`set_timeout`](Self::set_timeout) has
    /// not been called since the last [`arm`](Self::arm), matching Klipper C
    /// where the expire timer is only active after `trsync_set_timeout`.
    ///
    /// Otherwise, if `now` is at or past `expire_clock` (using signed
    /// wraparound comparison) and the state is [`TrSyncState::Armed`],
    /// triggers with `expire_reason` and returns `true`.
    pub fn check_timeout(&mut self, now: u32) -> bool {
        if !self.timeout_set {
            return false;
        }
        let diff = now.wrapping_sub(self.expire_clock) as i32;
        if diff >= 0 {
            self.trigger(self.expire_reason)
        } else {
            false
        }
    }

    /// Check whether a status report is due.
    ///
    /// Returns `false` immediately when in [`TrSyncState::Idle`] or when
    /// `report_ticks` is zero (matching Klipper C, which does not start the
    /// report timer when `report_ticks == 0`). Otherwise, if `now` is at or
    /// past `report_clock`, advances `report_clock` by `report_ticks` and
    /// returns `true`.
    pub fn report_due(&mut self, now: u32) -> bool {
        if self.state == TrSyncState::Idle || self.report_ticks == 0 {
            return false;
        }
        let diff = now.wrapping_sub(self.report_clock) as i32;
        if diff >= 0 {
            self.report_clock = self.report_clock.wrapping_add(self.report_ticks);
            true
        } else {
            false
        }
    }

    /// Stop report and expire timers without clearing the trigger reason.
    ///
    /// Mirrors Klipper C's `command_trsync_trigger` behavior: `ts->flags = 0`
    /// stops reports and timeouts but preserves `ts->trigger_reason`. This
    /// allows multiple `trsync_trigger` calls to return the original trigger
    /// reason (e.g., the trdispatch C helper sends one, then `stop()` sends
    /// another — both must see the endstop's trigger_reason).
    pub fn disarm(&mut self) {
        self.report_ticks = 0;
        self.timeout_set = false;
    }

    /// Reset to the [`TrSyncState::Idle`] state.
    ///
    /// Clears all state including trigger reason. Called by `trsync_start`
    /// (via `arm`) to prepare for a new homing cycle.
    pub fn reset(&mut self) {
        self.state = TrSyncState::Idle;
        self.expire_reason = 0;
        self.expire_clock = 0;
        self.report_clock = 0;
        self.report_ticks = 0;
        self.timeout_set = false;
    }
}

impl Default for TrSync {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // --- arm and trigger ---

    #[test]
    fn arm_transitions_to_armed() {
        let mut ts = TrSync::new();
        assert_eq!(ts.state(), TrSyncState::Idle);
        ts.arm(1000, 500, 0xAB);
        assert_eq!(ts.state(), TrSyncState::Armed);
    }

    #[test]
    fn trigger_when_armed_is_accepted() {
        let mut ts = TrSync::new();
        ts.arm(1000, 500, 1);
        let accepted = ts.trigger(42);
        assert!(accepted);
        assert_eq!(ts.state(), TrSyncState::Triggered(42));
    }

    #[test]
    fn trigger_preserves_reason_code() {
        let mut ts = TrSync::new();
        ts.arm(0, 0, 0);
        ts.trigger(0xFF);
        assert_eq!(ts.state(), TrSyncState::Triggered(0xFF));
    }

    // --- trigger when idle ---

    #[test]
    fn trigger_when_idle_is_ignored() {
        let mut ts = TrSync::new();
        let accepted = ts.trigger(1);
        assert!(!accepted);
        assert_eq!(ts.state(), TrSyncState::Idle);
    }

    // --- double trigger ---

    #[test]
    fn double_trigger_is_ignored() {
        let mut ts = TrSync::new();
        ts.arm(0, 0, 0);
        assert!(ts.trigger(1));
        // Second trigger must be ignored; reason must not change.
        let accepted = ts.trigger(2);
        assert!(!accepted);
        assert_eq!(ts.state(), TrSyncState::Triggered(1));
    }

    // --- timeout ---

    #[test]
    fn timeout_not_fired_before_deadline() {
        let mut ts = TrSync::new();
        ts.arm(0, 0, 5);
        ts.set_timeout(1000);
        // now < expire_clock → no trigger
        assert!(!ts.check_timeout(999));
        assert_eq!(ts.state(), TrSyncState::Armed);
    }

    #[test]
    fn timeout_fires_at_deadline() {
        let mut ts = TrSync::new();
        ts.arm(0, 0, 5);
        ts.set_timeout(1000);
        // now == expire_clock → trigger
        assert!(ts.check_timeout(1000));
        assert_eq!(ts.state(), TrSyncState::Triggered(5));
    }

    #[test]
    fn timeout_fires_past_deadline() {
        let mut ts = TrSync::new();
        ts.arm(0, 0, 7);
        ts.set_timeout(1000);
        // now > expire_clock → trigger
        assert!(ts.check_timeout(1001));
        assert_eq!(ts.state(), TrSyncState::Triggered(7));
    }

    #[test]
    fn timeout_uses_expire_reason() {
        let mut ts = TrSync::new();
        ts.arm(0, 0, 0xDE);
        ts.set_timeout(500);
        ts.check_timeout(500);
        assert_eq!(ts.state(), TrSyncState::Triggered(0xDE));
    }

    #[test]
    fn timeout_does_not_fire_when_idle() {
        let mut ts = TrSync::new();
        ts.set_timeout(0);
        // Idle → check_timeout must not accept (trigger returns false for idle)
        assert!(!ts.check_timeout(0));
        assert_eq!(ts.state(), TrSyncState::Idle);
    }

    // --- report timing ---

    #[test]
    fn report_not_due_before_clock() {
        let mut ts = TrSync::new();
        ts.arm(1000, 500, 0);
        assert!(!ts.report_due(999));
    }

    #[test]
    fn report_due_at_report_clock() {
        let mut ts = TrSync::new();
        ts.arm(1000, 500, 0);
        assert!(ts.report_due(1000));
    }

    #[test]
    fn report_advances_clock_by_ticks() {
        let mut ts = TrSync::new();
        ts.arm(1000, 500, 0);
        // First report at 1000; report_clock becomes 1500.
        assert!(ts.report_due(1000));
        // Not due yet at 1499.
        assert!(!ts.report_due(1499));
        // Due at 1500.
        assert!(ts.report_due(1500));
    }

    #[test]
    fn report_not_due_when_idle() {
        let mut ts = TrSync::new();
        // Even if now would satisfy any clock condition, Idle returns false.
        assert!(!ts.report_due(u32::MAX));
    }

    #[test]
    fn report_due_when_triggered() {
        let mut ts = TrSync::new();
        ts.arm(1000, 500, 0);
        ts.trigger(1);
        // Triggered state still reports (caller decides when to stop).
        assert!(ts.report_due(1000));
    }

    #[test]
    fn report_not_due_when_report_ticks_zero() {
        let mut ts = TrSync::new();
        // report_ticks=0 means no reports (matches Klipper C behavior).
        ts.arm(0, 0, 0);
        assert!(!ts.report_due(0));
        assert!(!ts.report_due(1000));
        // Also not due after triggering.
        ts.trigger(1);
        assert!(!ts.report_due(1000));
    }

    // --- trigger_reason ---

    #[test]
    fn trigger_reason_when_idle() {
        let ts = TrSync::new();
        assert_eq!(ts.trigger_reason(), 0);
    }

    #[test]
    fn trigger_reason_when_armed() {
        let mut ts = TrSync::new();
        ts.arm(0, 0, 0xAB);
        assert_eq!(ts.trigger_reason(), 0);
    }

    #[test]
    fn trigger_reason_when_triggered() {
        let mut ts = TrSync::new();
        ts.arm(0, 0, 0);
        ts.trigger(42);
        assert_eq!(ts.trigger_reason(), 42);
    }

    // --- disarm ---

    #[test]
    fn disarm_stops_reports_but_keeps_trigger_reason() {
        let mut ts = TrSync::new();
        ts.arm(1000, 500, 0);
        ts.trigger(1);
        assert!(ts.report_due(1000)); // reports fire before disarm
        ts.disarm();
        // After disarm: report_ticks=0, so report_due returns false
        assert!(!ts.report_due(1500));
        // But trigger reason is preserved
        assert_eq!(ts.trigger_reason(), 1);
        assert_eq!(ts.state(), TrSyncState::Triggered(1));
    }

    #[test]
    fn disarm_stops_timeout() {
        let mut ts = TrSync::new();
        ts.arm(0, 500, 5);
        ts.set_timeout(1000);
        ts.disarm();
        // timeout_set cleared by disarm
        assert!(!ts.check_timeout(1000));
        assert_eq!(ts.state(), TrSyncState::Armed);
    }

    // --- reset ---

    #[test]
    fn reset_returns_to_idle() {
        let mut ts = TrSync::new();
        ts.arm(1000, 500, 3);
        ts.trigger(42);
        ts.reset();
        assert_eq!(ts.state(), TrSyncState::Idle);
    }

    #[test]
    fn reset_clears_timeout() {
        let mut ts = TrSync::new();
        ts.arm(0, 0, 9);
        ts.set_timeout(100);
        ts.reset();
        // After reset the TrSync is Idle; check_timeout must not fire.
        assert!(!ts.check_timeout(100));
        assert_eq!(ts.state(), TrSyncState::Idle);
    }

    #[test]
    fn reset_allows_rearm() {
        let mut ts = TrSync::new();
        ts.arm(0, 0, 1);
        ts.trigger(1);
        ts.reset();
        ts.arm(2000, 200, 2);
        assert_eq!(ts.state(), TrSyncState::Armed);
    }

    // --- clock wraparound ---

    #[test]
    fn timeout_wraparound_fires_correctly() {
        let mut ts = TrSync::new();
        ts.arm(0, 0, 0xBB);
        // expire_clock just past the 32-bit boundary
        ts.set_timeout(0xFFFF_FF00u32);
        // now wraps around to 0x10 — in signed space: 0x10 - 0xFFFFFF00 = 0x110 > 0 (wait,
        // actually wrapping_sub gives 0x0000_0010u32.wrapping_sub(0xFFFF_FF00u32) = 0x0000_0110,
        // cast to i32 = 0x110 > 0 → fired)
        assert!(ts.check_timeout(0x0000_0010u32));
        assert_eq!(ts.state(), TrSyncState::Triggered(0xBB));
    }

    #[test]
    fn timeout_wraparound_not_fired_before() {
        let mut ts = TrSync::new();
        ts.arm(0, 0, 0xCC);
        // expire_clock is 0x0000_0010 — now = 0xFFFF_FF00 is BEFORE it in signed space
        // (0xFFFF_FF00u32.wrapping_sub(0x0000_0010u32) = 0xFFFF_FEF0, as i32 = -0x110 < 0)
        ts.set_timeout(0x0000_0010u32);
        assert!(!ts.check_timeout(0xFFFF_FF00u32));
        assert_eq!(ts.state(), TrSyncState::Armed);
    }

    #[test]
    fn report_wraparound_advances_correctly() {
        let mut ts = TrSync::new();
        // report_clock near the top of the u32 range
        ts.arm(0xFFFF_FF00u32, 0x200, 0);
        // Due at 0xFFFFFF00
        assert!(ts.report_due(0xFFFF_FF00u32));
        // Next report_clock wraps: 0xFFFFFF00 + 0x200 = 0x0000_0100
        assert!(!ts.report_due(0x0000_00FFu32));
        assert!(ts.report_due(0x0000_0100u32));
    }

    // --- timeout_set guard ---

    #[test]
    fn check_timeout_does_not_fire_without_set_timeout() {
        let mut ts = TrSync::new();
        // arm() does NOT enable the timeout — matching Klipper C where
        // the expire timer is only started in trsync_set_timeout.
        ts.arm(5000, 500, 1);
        assert!(!ts.check_timeout(5000));
        assert!(!ts.check_timeout(u32::MAX));
        assert_eq!(ts.state(), TrSyncState::Armed);
    }

    #[test]
    fn check_timeout_fires_after_set_timeout() {
        let mut ts = TrSync::new();
        ts.arm(1000, 500, 2);
        ts.set_timeout(5000);
        // Before deadline: no fire
        assert!(!ts.check_timeout(4999));
        assert_eq!(ts.state(), TrSyncState::Armed);
        // At deadline: fires
        assert!(ts.check_timeout(5000));
        assert_eq!(ts.state(), TrSyncState::Triggered(2));
    }

    #[test]
    fn arm_clears_timeout_set() {
        let mut ts = TrSync::new();
        ts.arm(0, 0, 1);
        ts.set_timeout(100);
        // Re-arm clears timeout_set
        ts.arm(2000, 200, 2);
        assert!(!ts.check_timeout(2000));
        assert_eq!(ts.state(), TrSyncState::Armed);
        // set_timeout re-enables
        ts.set_timeout(3000);
        assert!(ts.check_timeout(3000));
        assert_eq!(ts.state(), TrSyncState::Triggered(2));
    }
}
