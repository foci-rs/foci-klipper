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

/// Arguments of Klipper's `endstop_home` command, plus the board's pin id.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct HomingConfig {
    /// Clock of the first sample.
    pub clock: u32,
    /// Interval between oversampling reads.
    pub sample_ticks: u32,
    /// Consecutive matching reads required to trigger; 0 disarms.
    pub sample_count: u8,
    /// Interval between scan reads.
    pub rest_ticks: u32,
    /// Pin level that counts as triggered.
    pub pin_value: bool,
    /// Trsync signalled on trigger.
    pub trsync_oid: u8,
    /// Reason code reported to the trsync.
    pub trigger_reason: u8,
    /// Board pin id reported by query-state.
    pub pin_id: u8,
}

/// Two-phase endstop state machine.
#[derive(Debug, defmt::Format)]
pub struct EndstopRuntime {
    phase: EndstopPhase,
    pin_value: bool,
    armed: bool,
    trigger_count: u8,
    sample_count: u8,
    sample_ticks: u32,
    rest_ticks: u32,
    nextwake: u32,
    trsync_oid: u8,
    trigger_reason: u8,
    pin_id: u8,
}

impl EndstopRuntime {
    pub const fn new() -> Self {
        Self {
            phase: EndstopPhase::Scanning,
            pin_value: false,
            armed: false,
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

    /// True from a non-zero-count `start_homing` until a zero-count one.
    ///
    /// Stays true after the endstop triggers; Klipper's `endstop_query_state`
    /// reports it as `homing` in that state too.
    pub fn is_armed(&self) -> bool {
        self.armed
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

    /// Arm the endstop for homing. `sample_count == 0` disarms.
    pub fn start_homing(&mut self, cfg: HomingConfig) {
        if cfg.sample_count == 0 {
            self.armed = false;
            return;
        }
        self.phase = EndstopPhase::Scanning;
        self.pin_value = cfg.pin_value;
        self.armed = true;
        self.trigger_count = cfg.sample_count;
        self.sample_count = cfg.sample_count;
        self.sample_ticks = cfg.sample_ticks;
        self.rest_ticks = cfg.rest_ticks;
        self.nextwake = cfg.clock;
        self.trsync_oid = cfg.trsync_oid;
        self.trigger_reason = cfg.trigger_reason;
        self.pin_id = cfg.pin_id;
    }

    /// Scanning phase. Called at rest_ticks intervals.
    pub fn scan(&mut self, pin_state: bool, waketime: u32) -> EndstopSampleResult {
        if !self.armed {
            return EndstopSampleResult::Inactive;
        }
        let next_scan = waketime.wrapping_add(self.rest_ticks);
        if pin_state != self.pin_value {
            return EndstopSampleResult::Reschedule {
                next_clock: next_scan,
            };
        }
        self.nextwake = next_scan;
        self.phase = EndstopPhase::Oversampling;
        EndstopSampleResult::EnterOversamplingNow
    }

    /// Oversampling phase. Called at sample_ticks intervals.
    /// First call is immediate after scan() match (double-read).
    pub fn oversample(&mut self, pin_state: bool, waketime: u32) -> EndstopSampleResult {
        if pin_state != self.pin_value {
            self.phase = EndstopPhase::Scanning;
            self.trigger_count = self.sample_count;
            return EndstopSampleResult::Reschedule {
                next_clock: self.nextwake,
            };
        }
        self.trigger_count -= 1;
        if self.trigger_count == 0 {
            return EndstopSampleResult::Triggered {
                trsync_oid: self.trsync_oid,
                reason: self.trigger_reason,
            };
        }
        EndstopSampleResult::Reschedule {
            next_clock: waketime.wrapping_add(self.sample_ticks),
        }
    }

    /// Return query state: (armed, nextwake).
    pub fn query_state(&self) -> (bool, u32) {
        (self.armed, self.nextwake)
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
        e.start_homing(HomingConfig {
            clock: 1000,
            sample_ticks: 50,
            sample_count: 4,
            rest_ticks: 500,
            pin_value: true,
            trsync_oid: 7,
            trigger_reason: 42,
            pin_id: 0x10,
        });
        e
    }

    #[test]
    fn new_endstop_not_armed() {
        let e = EndstopRuntime::new();
        assert!(!e.is_armed());
    }

    #[test]
    fn start_homing_arms() {
        let e = armed_endstop();
        assert!(e.is_armed());
        assert_eq!(e.phase(), EndstopPhase::Scanning);
        assert_eq!(e.nextwake(), 1000);
    }

    #[test]
    fn start_homing_zero_disarms() {
        let mut e = armed_endstop();
        e.start_homing(HomingConfig {
            clock: 0,
            sample_ticks: 0,
            sample_count: 0,
            rest_ticks: 0,
            pin_value: false,
            trsync_oid: 0,
            trigger_reason: 0,
            pin_id: 0,
        });
        assert!(!e.is_armed());
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
        assert_eq!(e.nextwake(), 1500);
    }

    #[test]
    fn oversample_mismatch_returns_to_scanning() {
        let mut e = armed_endstop();
        e.scan(true, 1000);
        let result = e.oversample(false, 1000);
        assert_eq!(result, EndstopSampleResult::Reschedule { next_clock: 1500 });
        assert_eq!(e.phase(), EndstopPhase::Scanning);
    }

    #[test]
    fn oversample_counts_down_to_trigger() {
        let mut e = armed_endstop();
        e.scan(true, 1000);

        let r = e.oversample(true, 1000);
        assert_eq!(r, EndstopSampleResult::Reschedule { next_clock: 1050 });

        let r = e.oversample(true, 1050);
        assert_eq!(r, EndstopSampleResult::Reschedule { next_clock: 1100 });

        let r = e.oversample(true, 1100);
        assert_eq!(r, EndstopSampleResult::Reschedule { next_clock: 1150 });

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
    fn armed_stays_true_after_trigger() {
        let mut e = armed_endstop();
        e.scan(true, 1000);
        e.oversample(true, 1000);
        e.oversample(true, 1050);
        e.oversample(true, 1100);
        e.oversample(true, 1150);
        assert!(e.is_armed());
    }

    #[test]
    fn scan_when_not_homing_returns_inactive() {
        let mut e = EndstopRuntime::new();
        assert_eq!(e.scan(true, 0), EndstopSampleResult::Inactive);
    }

    #[test]
    fn nextwake_initialized_on_arm() {
        let e = armed_endstop();
        assert_eq!(e.nextwake(), 1000);
    }

    #[test]
    fn nextwake_updated_on_oversample_entry() {
        let mut e = armed_endstop();
        e.scan(true, 2000);
        assert_eq!(e.nextwake(), 2500);
    }

    #[test]
    fn clock_wraparound_scan() {
        let mut e = EndstopRuntime::new();
        e.start_homing(HomingConfig {
            clock: 0xFFFF_FF00,
            sample_ticks: 50,
            sample_count: 3,
            rest_ticks: 100,
            pin_value: true,
            trsync_oid: 1,
            trigger_reason: 1,
            pin_id: 0x10,
        });
        let result = e.scan(false, 0xFFFF_FF00);
        assert_eq!(
            result,
            EndstopSampleResult::Reschedule {
                next_clock: 0xFFFF_FF00u32.wrapping_add(100),
            }
        );
    }

    #[test]
    fn query_state_returns_armed_and_nextwake() {
        let e = armed_endstop();
        let (armed, nextwake) = e.query_state();
        assert!(armed);
        assert_eq!(nextwake, 1000);
    }
}
