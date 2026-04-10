//! Trigger domain types for Klipper-equivalent trsync atomicity.
//!
//! Provides pure-logic types used by the board crate's `TriggerDomain`
//! RTIC shared resource. No hardware access, no RTIC dependencies.

/// Snapshot of a trsync_state report, created inside the TriggerDomain
/// lock and sent outside it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ReportSnapshot {
    pub oid: u8,
    pub can_trigger: u8,
    pub trigger_reason: u8,
    pub clock: u32,
}

/// Result of a trigger attempt.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TriggerResult {
    /// First trigger accepted. Snapshot contains event-time clock.
    First { snapshot: ReportSnapshot },
    /// Already triggered. Returns preserved reason for command cleanup.
    AlreadyTriggered { reason: u8 },
    /// Unknown TrSync OID.
    UnknownOid,
}

/// Stop signal registration: links a stepper to a trsync for
/// synchronized stop on trigger.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct StopSignal {
    pub trsync_oid: u8,
    pub stepper_idx: usize,
}

/// Per-stepper stop/reset state, owned by the TriggerDomain.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct StepperStopRuntime {
    /// Set by trigger stop, cleared by reset_step_clock when !blocked_by_trigger.
    pub need_reset: bool,
    /// Set by trigger, cleared by trsync_trigger final cleanup.
    pub blocked_by_trigger: bool,
    /// Records which trsync OID caused the block.
    pub blocked_by_trsync: Option<u8>,
    /// Whether the step timer is armed.
    pub timer_active: bool,
}

impl StepperStopRuntime {
    pub const fn new() -> Self {
        Self {
            need_reset: false,
            blocked_by_trigger: false,
            blocked_by_trsync: None,
            timer_active: false,
        }
    }
}

impl Default for StepperStopRuntime {
    fn default() -> Self {
        Self::new()
    }
}

/// TrSync runtime state using flags (not enum), matching C's
/// TSF_CAN_TRIGGER / TSF_REPORT model.
pub struct TrSyncRuntime {
    /// Equivalent to TSF_CAN_TRIGGER. Set by arm(), cleared by first trigger.
    can_trigger: bool,
    /// First trigger reason, preserved across multiple trsync_trigger calls.
    trigger_reason: u8,
    /// Reason code for timeout expiry.
    expire_reason: u8,
    /// Report pending flag. Consumed by report sender task.
    report_pending: bool,
    /// Snapshot for the pending report.
    report_snapshot: Option<ReportSnapshot>,
    /// Expire timer deadline (MCU clock ticks).
    expire_clock: u32,
    /// Whether expire timer is active.
    expire_active: bool,
    /// Report timer interval. 0 = no reports.
    report_ticks: u32,
    /// Next report timer deadline.
    report_clock: u32,
}

impl TrSyncRuntime {
    pub const fn new() -> Self {
        Self {
            can_trigger: false,
            trigger_reason: 0,
            expire_reason: 0,
            report_pending: false,
            report_snapshot: None,
            expire_clock: 0,
            expire_active: false,
            report_ticks: 0,
            report_clock: 0,
        }
    }

    /// Arm for a new homing cycle.
    pub fn arm(&mut self, report_clock: u32, report_ticks: u32, expire_reason: u8) {
        self.can_trigger = true;
        self.trigger_reason = 0;
        self.expire_reason = expire_reason;
        self.report_pending = false;
        self.report_snapshot = None;
        self.expire_clock = 0;
        self.expire_active = false;
        self.report_ticks = report_ticks;
        self.report_clock = report_clock;
    }

    /// Attempt to trigger. Returns the result for the caller to act on.
    /// Does NOT cancel timers — that is deferred to cleanup.
    /// `oid` is the TrSync OID for the report snapshot.
    pub fn trigger(&mut self, oid: u8, reason: u8, clock: u32) -> TriggerResult {
        if !self.can_trigger {
            return TriggerResult::AlreadyTriggered {
                reason: self.trigger_reason,
            };
        }
        self.can_trigger = false;
        self.trigger_reason = reason;
        let snapshot = ReportSnapshot {
            oid,
            can_trigger: 0,
            trigger_reason: reason,
            clock,
        };
        self.report_pending = true;
        self.report_snapshot = Some(snapshot);
        TriggerResult::First { snapshot }
    }

    pub fn can_trigger(&self) -> bool {
        self.can_trigger
    }

    pub fn trigger_reason(&self) -> u8 {
        self.trigger_reason
    }

    pub fn expire_reason(&self) -> u8 {
        self.expire_reason
    }

    pub fn expire_clock(&self) -> u32 {
        self.expire_clock
    }

    pub fn expire_active(&self) -> bool {
        self.expire_active
    }

    pub fn report_ticks(&self) -> u32 {
        self.report_ticks
    }

    pub fn report_clock(&self) -> u32 {
        self.report_clock
    }

    pub fn set_report_clock(&mut self, clock: u32) {
        self.report_clock = clock;
    }

    pub fn report_pending(&self) -> bool {
        self.report_pending
    }

    pub fn report_snapshot(&self) -> Option<ReportSnapshot> {
        self.report_snapshot
    }

    pub fn set_timeout(&mut self, clock: u32) {
        self.expire_clock = clock;
        self.expire_active = true;
    }

    /// Check if expire deadline has passed. Returns true if triggered.
    /// `oid` is the TrSync OID for the report snapshot.
    pub fn check_timeout(&mut self, oid: u8, now: u32) -> bool {
        if !self.expire_active || !self.can_trigger {
            return false;
        }
        let diff = now.wrapping_sub(self.expire_clock) as i32;
        if diff >= 0 {
            self.trigger(oid, self.expire_reason, now);
            true
        } else {
            false
        }
    }

    /// Set report pending with a liveness snapshot. Only valid while
    /// can_trigger is true. Returns false if not armed.
    pub fn set_liveness_report(&mut self, oid: u8, clock: u32) -> bool {
        if !self.can_trigger {
            return false;
        }
        let snapshot = ReportSnapshot {
            oid,
            can_trigger: 1,
            trigger_reason: 0,
            clock,
        };
        self.report_pending = true;
        self.report_snapshot = Some(snapshot);
        true
    }

    /// Take the pending report snapshot, clearing the pending flag.
    pub fn take_report(&mut self) -> Option<ReportSnapshot> {
        if self.report_pending {
            self.report_pending = false;
            self.report_snapshot.take()
        } else {
            None
        }
    }

    /// Clear pending report (used by trsync_trigger cleanup to prevent
    /// stale async reports after the final reply).
    pub fn clear_pending_report(&mut self) {
        self.report_pending = false;
        self.report_snapshot = None;
    }

    /// Cancel timers (used by trsync_trigger cleanup and trsync_start).
    pub fn cancel_timers(&mut self) {
        self.expire_active = false;
        self.report_ticks = 0;
    }

    /// Full reset to unconfigured state.
    pub fn reset(&mut self) {
        *self = Self::new();
    }
}

impl Default for TrSyncRuntime {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn new_trsync_cannot_trigger() {
        let ts = TrSyncRuntime::new();
        assert!(!ts.can_trigger());
    }

    #[test]
    fn arm_enables_trigger() {
        let mut ts = TrSyncRuntime::new();
        ts.arm(1000, 500, 0xAB);
        assert!(ts.can_trigger());
        assert_eq!(ts.expire_reason(), 0xAB);
        assert_eq!(ts.report_ticks(), 500);
    }

    #[test]
    fn first_trigger_accepted() {
        let mut ts = TrSyncRuntime::new();
        ts.arm(1000, 500, 1);
        let result = ts.trigger(0, 42, 5000);
        assert!(matches!(result, TriggerResult::First { .. }));
        assert!(!ts.can_trigger());
        assert_eq!(ts.trigger_reason(), 42);
    }

    #[test]
    fn double_trigger_returns_preserved_reason() {
        let mut ts = TrSyncRuntime::new();
        ts.arm(0, 0, 0);
        ts.trigger(0, 42, 100);
        let result = ts.trigger(0, 99, 200);
        assert_eq!(result, TriggerResult::AlreadyTriggered { reason: 42 });
        assert_eq!(ts.trigger_reason(), 42);
    }

    #[test]
    fn trigger_sets_report_pending() {
        let mut ts = TrSyncRuntime::new();
        ts.arm(0, 0, 0);
        ts.trigger(0, 1, 1000);
        assert!(ts.report_pending());
        assert!(ts.report_snapshot().is_some());
    }

    #[test]
    fn take_report_clears_pending() {
        let mut ts = TrSyncRuntime::new();
        ts.arm(0, 0, 0);
        ts.trigger(0, 1, 1000);
        let snap = ts.take_report();
        assert!(snap.is_some());
        assert!(!ts.report_pending());
        assert!(ts.take_report().is_none());
    }

    #[test]
    fn clear_pending_report_prevents_stale() {
        let mut ts = TrSyncRuntime::new();
        ts.arm(0, 0, 0);
        ts.trigger(0, 1, 1000);
        ts.clear_pending_report();
        assert!(!ts.report_pending());
        assert!(ts.take_report().is_none());
    }

    #[test]
    fn timeout_not_fired_without_set_timeout() {
        let mut ts = TrSyncRuntime::new();
        ts.arm(0, 0, 5);
        assert!(!ts.check_timeout(0, u32::MAX));
        assert!(ts.can_trigger());
    }

    #[test]
    fn timeout_fires_at_deadline() {
        let mut ts = TrSyncRuntime::new();
        ts.arm(0, 0, 5);
        ts.set_timeout(1000);
        assert!(!ts.check_timeout(0, 999));
        assert!(ts.check_timeout(0, 1000));
        assert!(!ts.can_trigger());
        assert_eq!(ts.trigger_reason(), 5);
    }

    #[test]
    fn timeout_wraparound() {
        let mut ts = TrSyncRuntime::new();
        ts.arm(0, 0, 0xBB);
        ts.set_timeout(0xFFFF_FF00);
        assert!(ts.check_timeout(0, 0x0000_0010));
        assert_eq!(ts.trigger_reason(), 0xBB);
    }

    #[test]
    fn liveness_report_only_when_armed() {
        let mut ts = TrSyncRuntime::new();
        ts.arm(0, 500, 0);
        assert!(ts.set_liveness_report(5, 1000));
        ts.trigger(0, 1, 2000);
        assert!(!ts.set_liveness_report(5, 3000));
    }

    #[test]
    fn cancel_timers_disables_expire_and_report() {
        let mut ts = TrSyncRuntime::new();
        ts.arm(1000, 500, 5);
        ts.set_timeout(2000);
        ts.cancel_timers();
        assert!(!ts.expire_active());
        assert_eq!(ts.report_ticks(), 0);
    }

    #[test]
    fn stepper_stop_runtime_clearing_sequence() {
        let mut ss = StepperStopRuntime::new();
        ss.need_reset = true;
        ss.blocked_by_trigger = true;
        ss.blocked_by_trsync = Some(10);

        assert!(ss.blocked_by_trigger);

        ss.blocked_by_trigger = false;
        ss.blocked_by_trsync = None;
        assert!(ss.need_reset);

        assert!(!ss.blocked_by_trigger);
        ss.need_reset = false;
        assert!(!ss.need_reset);
    }
}
