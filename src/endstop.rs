//! Endstop state machine with two-phase scan/oversample sampling.
//!
//! Matches Klipper C's `endstop_event` / `endstop_oversample_event` split.
//! The caller (P3 ISR) drives the state machine by calling `scan()` or
//! `oversample()` based on the current phase, and performing the second
//! GPIO read on `EnterOversamplingNow`.

/// Current sampling phase.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum EndstopPhase {
    /// Scanning: sampling at rest_ticks rate, looking for first match.
    Scanning,
    /// Oversampling: sampling at sample_ticks rate, counting consecutive matches.
    Oversampling,
}

/// Result of a single endstop sample call.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EndstopSampleResult {
    /// Not homing; sample ignored.
    Inactive,
    /// Reschedule timer at returned clock tick.
    Reschedule { next_clock: u32 },
    /// Pin matches during scan phase. Caller must read GPIO again and
    /// call `oversample()` with the fresh read (C's double-read behavior).
    EnterOversamplingNow,
    /// Trigger confirmed. Caller must enter TriggerDomain and call trigger().
    Triggered { trsync_oid: u8, reason: u8 },
}

/// Two-phase endstop state machine.
#[derive(Debug, defmt::Format)]
pub struct EndstopRuntime {
    /// Current sampling phase.
    phase: EndstopPhase,
    /// Pin level that indicates trigger.
    pin_value: bool,
    /// ESF_HOMING equivalent. NOT cleared on trigger.
    homing: bool,
    /// Consecutive match countdown (starts at sample_count).
    trigger_count: u8,
    /// Total consecutive matches required.
    sample_count: u8,
    /// Fast oversampling interval (C: sample_time).
    sample_ticks: u32,
    /// Slow scanning interval (C: rest_time).
    rest_ticks: u32,
    /// Scan-rate fallback wake time. Returned by endstop_query_state.
    nextwake: u32,
    /// TrSync OID to trigger on confirmed match.
    trsync_oid: u8,
    /// Reason code for TrSync trigger.
    trigger_reason: u8,
    /// Configured GPIO pin ID (copied for query snapshot).
    pin_id: u8,
}

impl EndstopRuntime {
    pub const fn new() -> Self {
        Self {
            phase: EndstopPhase::Scanning,
            pin_value: false,
            homing: false,
            trigger_count: 0,
            sample_count: 0,
            sample_ticks: 0,
            rest_ticks: 0,
            nextwake: 0,
            trsync_oid: 0,
            trigger_reason: 0,
            pin_id: 0xFF,
        }
    }

    pub fn is_homing(&self) -> bool {
        self.homing
    }

    pub fn phase(&self) -> EndstopPhase {
        self.phase
    }

    pub fn nextwake(&self) -> u32 {
        self.nextwake
    }

    pub fn pin_id(&self) -> u8 {
        self.pin_id
    }

    /// Arm the endstop for homing. sample_count=0 disarms.
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
        pin_id: u8,
    ) {
        if sample_count == 0 {
            self.homing = false;
            return;
        }
        self.phase = EndstopPhase::Scanning;
        self.pin_value = pin_value;
        self.homing = true;
        self.trigger_count = sample_count;
        self.sample_count = sample_count;
        self.sample_ticks = sample_ticks;
        self.rest_ticks = rest_ticks;
        self.nextwake = clock;
        self.trsync_oid = trsync_oid;
        self.trigger_reason = trigger_reason;
        self.pin_id = pin_id;
    }

    /// Scanning phase. Called at rest_ticks intervals.
    pub fn scan(&mut self, pin_state: bool, waketime: u32) -> EndstopSampleResult {
        if !self.homing {
            return EndstopSampleResult::Inactive;
        }
        let next_scan = waketime.wrapping_add(self.rest_ticks);
        if pin_state != self.pin_value {
            return EndstopSampleResult::Reschedule {
                next_clock: next_scan,
            };
        }
        // Match: save nextwake, enter oversampling.
        self.nextwake = next_scan;
        self.phase = EndstopPhase::Oversampling;
        EndstopSampleResult::EnterOversamplingNow
    }

    /// Oversampling phase. Called at sample_ticks intervals.
    /// First call is immediate after scan() match (double-read).
    pub fn oversample(&mut self, pin_state: bool, waketime: u32) -> EndstopSampleResult {
        if pin_state != self.pin_value {
            // Mismatch: return to scanning at saved nextwake.
            self.phase = EndstopPhase::Scanning;
            self.trigger_count = self.sample_count;
            return EndstopSampleResult::Reschedule {
                next_clock: self.nextwake,
            };
        }
        self.trigger_count -= 1;
        if self.trigger_count == 0 {
            // Confirmed. homing stays true (ESF_HOMING lifecycle).
            return EndstopSampleResult::Triggered {
                trsync_oid: self.trsync_oid,
                reason: self.trigger_reason,
            };
        }
        EndstopSampleResult::Reschedule {
            next_clock: waketime.wrapping_add(self.sample_ticks),
        }
    }

    /// Return query state: (homing, nextwake).
    pub fn query_state(&self) -> (bool, u32) {
        (self.homing, self.nextwake)
    }

    /// Full reset to unconfigured state.
    pub fn reset(&mut self) {
        *self = Self::new();
    }
}

impl Default for EndstopRuntime {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn armed_endstop() -> EndstopRuntime {
        let mut e = EndstopRuntime::new();
        e.start_homing(1000, 50, 4, 500, true, 7, 42, 0x10);
        e
    }

    #[test]
    fn new_endstop_not_homing() {
        let e = EndstopRuntime::new();
        assert!(!e.is_homing());
    }

    #[test]
    fn start_homing_arms() {
        let e = armed_endstop();
        assert!(e.is_homing());
        assert_eq!(e.phase(), EndstopPhase::Scanning);
        assert_eq!(e.nextwake(), 1000);
    }

    #[test]
    fn start_homing_zero_disarms() {
        let mut e = armed_endstop();
        e.start_homing(0, 0, 0, 0, false, 0, 0, 0);
        assert!(!e.is_homing());
    }

    #[test]
    fn scan_no_match_reschedules_at_rest_ticks() {
        let mut e = armed_endstop();
        let result = e.scan(false, 1000);
        assert_eq!(result, EndstopSampleResult::Reschedule { next_clock: 1500 });
        assert_eq!(e.phase(), EndstopPhase::Scanning);
    }

    #[test]
    fn scan_match_enters_oversample() {
        let mut e = armed_endstop();
        let result = e.scan(true, 1000);
        assert_eq!(result, EndstopSampleResult::EnterOversamplingNow);
        assert_eq!(e.phase(), EndstopPhase::Oversampling);
        assert_eq!(e.nextwake(), 1500); // saved scan fallback
    }

    #[test]
    fn oversample_mismatch_returns_to_scanning() {
        let mut e = armed_endstop();
        e.scan(true, 1000); // enter oversample
        let result = e.oversample(false, 1000);
        assert_eq!(result, EndstopSampleResult::Reschedule { next_clock: 1500 });
        assert_eq!(e.phase(), EndstopPhase::Scanning);
    }

    #[test]
    fn oversample_counts_down_to_trigger() {
        let mut e = armed_endstop(); // sample_count=4
        e.scan(true, 1000); // enter oversample, nextwake=1500

        // First oversample (double-read at same instant)
        let r = e.oversample(true, 1000);
        assert_eq!(r, EndstopSampleResult::Reschedule { next_clock: 1050 });

        // Second oversample
        let r = e.oversample(true, 1050);
        assert_eq!(r, EndstopSampleResult::Reschedule { next_clock: 1100 });

        // Third oversample
        let r = e.oversample(true, 1100);
        assert_eq!(r, EndstopSampleResult::Reschedule { next_clock: 1150 });

        // Fourth oversample — triggers
        let r = e.oversample(true, 1150);
        assert_eq!(
            r,
            EndstopSampleResult::Triggered {
                trsync_oid: 7,
                reason: 42,
            }
        );
    }

    #[test]
    fn homing_stays_true_after_trigger() {
        let mut e = armed_endstop();
        e.scan(true, 1000);
        e.oversample(true, 1000);
        e.oversample(true, 1050);
        e.oversample(true, 1100);
        e.oversample(true, 1150); // triggers
        assert!(e.is_homing()); // ESF_HOMING stays true
    }

    #[test]
    fn scan_when_not_homing_returns_inactive() {
        let mut e = EndstopRuntime::new();
        assert_eq!(e.scan(true, 0), EndstopSampleResult::Inactive);
    }

    #[test]
    fn nextwake_initialized_on_arm() {
        let e = armed_endstop();
        assert_eq!(e.nextwake(), 1000); // first wake clock
    }

    #[test]
    fn nextwake_updated_on_oversample_entry() {
        let mut e = armed_endstop();
        e.scan(true, 2000); // nextwake = 2000 + 500 = 2500
        assert_eq!(e.nextwake(), 2500);
    }

    #[test]
    fn clock_wraparound_scan() {
        let mut e = EndstopRuntime::new();
        e.start_homing(0xFFFF_FF00, 50, 3, 100, true, 1, 1, 0x10);
        let result = e.scan(false, 0xFFFF_FF00);
        assert_eq!(
            result,
            EndstopSampleResult::Reschedule {
                next_clock: 0xFFFF_FF00u32.wrapping_add(100),
            }
        );
    }

    #[test]
    fn query_state_returns_homing_and_nextwake() {
        let e = armed_endstop();
        let (homing, nextwake) = e.query_state();
        assert!(homing);
        assert_eq!(nextwake, 1000);
    }
}
