//! Software timer scheduler for Klipper-clock events.
//!
//! Manages a fixed-capacity event table driven by one hardware compare
//! channel. All clock comparisons use signed 32-bit wraparound.

/// Maximum scheduler events: NUM_STEPPERS + NUM_ENDSTOPS + 2 * NUM_TRSYNC.
/// With 1 physical stepper, 4 endstops, and 4 trsync = 1 + 4 + 8 = 13.
pub const MAX_SCHEDULER_EVENTS: usize = 13;

/// Scheduler event kind.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SchedulerEventKind {
    Stepper(usize),
    EndstopSample(usize),
    TrSyncExpire(usize),
    TrSyncReport(usize),
}

/// A scheduled event.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SchedulerEvent {
    pub kind: SchedulerEventKind,
    pub wake_clock: u32,
}

/// Result of inserting a scheduler event.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ScheduleResult {
    /// Event was inserted. `new_earliest` means hardware compare must move.
    Inserted { new_earliest: bool },
    /// The fixed scheduler table had no free slot.
    Full,
}

/// Return the hardware compare clock to arm for a scheduler deadline.
///
/// If `deadline` is farther than `min_lead_ticks` in the future, callers can
/// arm the hardware timer at the exact deadline. If it is already due or too
/// close to arm reliably, this returns `now + min_lead_ticks` instead. The
/// comparison uses signed 32-bit wraparound semantics for Klipper clocks.
pub fn timer_arm_clock(now: u32, deadline: u32, min_lead_ticks: u32) -> u32 {
    let delta = deadline.wrapping_sub(now) as i32;
    if delta <= min_lead_ticks as i32 {
        now.wrapping_add(min_lead_ticks)
    } else {
        deadline
    }
}

/// Software scheduler over a fixed event table.
pub struct TriggerScheduler {
    events: [Option<SchedulerEvent>; MAX_SCHEDULER_EVENTS],
    /// Whether the hardware compare (CC2) is currently enabled.
    cc2_active: bool,
}

impl TriggerScheduler {
    pub const fn new() -> Self {
        Self {
            events: [None; MAX_SCHEDULER_EVENTS],
            cc2_active: false,
        }
    }

    pub fn cc2_active(&self) -> bool {
        self.cc2_active
    }

    pub fn set_cc2_active(&mut self, active: bool) {
        self.cc2_active = active;
    }

    /// Schedule an event with explicit full-table reporting.
    pub fn try_schedule(&mut self, event: SchedulerEvent) -> ScheduleResult {
        // Find empty slot index first so we can call earliest_deadline() without
        // holding a mutable reference into self.events at the same time.
        let slot_index = self.events.iter().position(|e| e.is_none());
        if let Some(index) = slot_index {
            let new_earliest = self.earliest_deadline().is_none_or(|earliest| {
                let delta = event.wake_clock.wrapping_sub(earliest) as i32;
                delta < 0 // new event is earlier
            });
            self.events[index] = Some(event);
            ScheduleResult::Inserted { new_earliest }
        } else {
            // Table full — should not happen with correct capacity
            ScheduleResult::Full
        }
    }

    /// Schedule an event. Temporary compatibility wrapper for migration only.
    pub fn schedule(&mut self, event: SchedulerEvent) -> bool {
        match self.try_schedule(event) {
            ScheduleResult::Inserted { new_earliest } => new_earliest,
            ScheduleResult::Full => false,
        }
    }

    /// Cancel all events matching a filter. Returns number cancelled.
    pub fn cancel<F: Fn(&SchedulerEventKind) -> bool>(&mut self, filter: F) -> usize {
        let mut count = 0;
        for slot in self.events.iter_mut() {
            if let Some(event) = slot
                && filter(&event.kind)
            {
                *slot = None;
                count += 1;
            }
        }
        count
    }

    /// Find the earliest deadline across all active events.
    pub fn earliest_deadline(&self) -> Option<u32> {
        let mut earliest: Option<u32> = None;
        for event in self.events.iter().flatten() {
            earliest = Some(match earliest {
                None => event.wake_clock,
                Some(e) => {
                    let delta = event.wake_clock.wrapping_sub(e) as i32;
                    if delta < 0 { event.wake_clock } else { e }
                }
            });
        }
        earliest
    }

    /// Iterate over due events (wake_clock <= now in wraparound sense).
    /// Returns indices of due events for the caller to process.
    pub fn find_due(&self, now: u32) -> heapless::Vec<usize, MAX_SCHEDULER_EVENTS> {
        let mut due = heapless::Vec::new();
        for (i, slot) in self.events.iter().enumerate() {
            if let Some(event) = slot {
                let delta = now.wrapping_sub(event.wake_clock) as i32;
                if delta >= 0 {
                    let _ = due.push(i);
                }
            }
        }
        due
    }

    /// Remove and return the earliest due event.
    ///
    /// If multiple events are due at `now`, this follows the same ordering
    /// used by [`earliest_deadline`], so callers can repeatedly pop until it
    /// returns `None` and process all ready work in scheduler order.
    pub fn pop_due(&mut self, now: u32) -> Option<SchedulerEvent> {
        let mut earliest_index: Option<usize> = None;
        let mut earliest_clock: Option<u32> = None;
        for (index, slot) in self.events.iter().enumerate() {
            let Some(event) = slot else {
                continue;
            };
            let due_delta = now.wrapping_sub(event.wake_clock) as i32;
            if due_delta < 0 {
                continue;
            }
            if earliest_clock.is_none_or(|earliest| {
                let delta = event.wake_clock.wrapping_sub(earliest) as i32;
                delta < 0
            }) {
                earliest_index = Some(index);
                earliest_clock = Some(event.wake_clock);
            }
        }
        let index = earliest_index?;
        let event = self.events[index];
        self.events[index] = None;
        event
    }

    /// Get event at index (for processing).
    pub fn get(&self, index: usize) -> Option<&SchedulerEvent> {
        self.events[index].as_ref()
    }

    /// Update the wake_clock of an event at index.
    pub fn reschedule(&mut self, index: usize, new_clock: u32) {
        if let Some(event) = &mut self.events[index] {
            event.wake_clock = new_clock;
        }
    }

    /// Remove event at index.
    pub fn remove(&mut self, index: usize) {
        self.events[index] = None;
    }

    /// Check if any events are active.
    pub fn is_empty(&self) -> bool {
        self.events.iter().all(|e| e.is_none())
    }

    /// Clear all events.
    pub fn clear(&mut self) {
        for slot in &mut self.events {
            *slot = None;
        }
        self.cc2_active = false;
    }
}

impl Default for TriggerScheduler {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn insert(scheduler: &mut TriggerScheduler, event: SchedulerEvent) {
        assert!(matches!(
            scheduler.try_schedule(event),
            ScheduleResult::Inserted { .. }
        ));
    }

    #[test]
    fn new_scheduler_empty() {
        let s = TriggerScheduler::new();
        assert!(s.is_empty());
        assert_eq!(s.earliest_deadline(), None);
    }

    #[test]
    fn schedule_and_find_due() {
        let mut s = TriggerScheduler::new();
        insert(&mut s, SchedulerEvent {
            kind: SchedulerEventKind::EndstopSample(0),
            wake_clock: 1000,
        });
        assert!(!s.is_empty());
        let due = s.find_due(1000);
        assert_eq!(due.len(), 1);
        let due = s.find_due(999);
        assert_eq!(due.len(), 0);
    }

    #[test]
    fn earliest_deadline_correct() {
        let mut s = TriggerScheduler::new();
        insert(&mut s, SchedulerEvent {
            kind: SchedulerEventKind::TrSyncExpire(0),
            wake_clock: 5000,
        });
        insert(&mut s, SchedulerEvent {
            kind: SchedulerEventKind::EndstopSample(0),
            wake_clock: 2000,
        });
        assert_eq!(s.earliest_deadline(), Some(2000));
    }

    #[test]
    fn cancel_by_kind() {
        let mut s = TriggerScheduler::new();
        insert(&mut s, SchedulerEvent {
            kind: SchedulerEventKind::TrSyncExpire(0),
            wake_clock: 1000,
        });
        insert(&mut s, SchedulerEvent {
            kind: SchedulerEventKind::TrSyncReport(0),
            wake_clock: 2000,
        });
        insert(&mut s, SchedulerEvent {
            kind: SchedulerEventKind::EndstopSample(1),
            wake_clock: 3000,
        });
        let cancelled = s.cancel(|k| {
            matches!(
                k,
                SchedulerEventKind::TrSyncExpire(0) | SchedulerEventKind::TrSyncReport(0)
            )
        });
        assert_eq!(cancelled, 2);
        assert_eq!(s.earliest_deadline(), Some(3000));
    }

    #[test]
    fn wraparound_due_check() {
        let mut s = TriggerScheduler::new();
        insert(&mut s, SchedulerEvent {
            kind: SchedulerEventKind::EndstopSample(0),
            wake_clock: 0xFFFF_FF00,
        });
        // now=0x10 is past 0xFFFFFF00 in wraparound
        let due = s.find_due(0x0000_0010);
        assert_eq!(due.len(), 1);
    }

    #[test]
    fn wraparound_earliest_deadline() {
        let mut s = TriggerScheduler::new();
        insert(&mut s, SchedulerEvent {
            kind: SchedulerEventKind::EndstopSample(0),
            wake_clock: 0x0000_0100,
        });
        insert(&mut s, SchedulerEvent {
            kind: SchedulerEventKind::TrSyncExpire(0),
            wake_clock: 0xFFFF_FF00,
        });
        // 0xFFFFFF00 is earlier than 0x100 in signed wraparound
        assert_eq!(s.earliest_deadline(), Some(0xFFFF_FF00));
    }

    #[test]
    fn reschedule_updates_clock() {
        let mut s = TriggerScheduler::new();
        insert(&mut s, SchedulerEvent {
            kind: SchedulerEventKind::TrSyncReport(0),
            wake_clock: 1000,
        });
        let due = s.find_due(1000);
        assert_eq!(due.len(), 1);
        s.reschedule(due[0], 2000);
        let due = s.find_due(1500);
        assert_eq!(due.len(), 0);
        let due = s.find_due(2000);
        assert_eq!(due.len(), 1);
    }

    #[test]
    fn clear_removes_all() {
        let mut s = TriggerScheduler::new();
        insert(&mut s, SchedulerEvent {
            kind: SchedulerEventKind::EndstopSample(0),
            wake_clock: 1000,
        });
        s.clear();
        assert!(s.is_empty());
        assert!(!s.cc2_active());
    }

    #[test]
    fn try_schedule_reports_new_earliest() {
        let mut s = TriggerScheduler::new();
        assert_eq!(
            s.try_schedule(SchedulerEvent {
                kind: SchedulerEventKind::EndstopSample(0),
                wake_clock: 5000,
            }),
            ScheduleResult::Inserted { new_earliest: true }
        );
        assert_eq!(
            s.try_schedule(SchedulerEvent {
                kind: SchedulerEventKind::EndstopSample(1),
                wake_clock: 3000,
            }),
            ScheduleResult::Inserted { new_earliest: true }
        );
        assert_eq!(
            s.try_schedule(SchedulerEvent {
                kind: SchedulerEventKind::TrSyncExpire(0),
                wake_clock: 4000,
            }),
            ScheduleResult::Inserted {
                new_earliest: false,
            }
        );
    }

    #[test]
    fn try_schedule_returns_full_when_table_is_full() {
        let mut s = TriggerScheduler::new();
        for idx in 0..MAX_SCHEDULER_EVENTS {
            assert!(matches!(
                s.try_schedule(SchedulerEvent {
                    kind: SchedulerEventKind::EndstopSample(idx),
                    wake_clock: idx as u32,
                }),
                ScheduleResult::Inserted { .. }
            ));
        }

        assert_eq!(
            s.try_schedule(SchedulerEvent {
                kind: SchedulerEventKind::Stepper(0),
                wake_clock: 100,
            }),
            ScheduleResult::Full
        );
    }

    #[test]
    fn pop_due_processes_stepper_and_trigger_events_in_clock_order() {
        let mut s = TriggerScheduler::new();
        insert(&mut s, SchedulerEvent {
            kind: SchedulerEventKind::EndstopSample(0),
            wake_clock: 1100,
        });
        insert(&mut s, SchedulerEvent {
            kind: SchedulerEventKind::Stepper(0),
            wake_clock: 1000,
        });
        insert(&mut s, SchedulerEvent {
            kind: SchedulerEventKind::TrSyncReport(0),
            wake_clock: 1050,
        });

        assert_eq!(
            s.pop_due(1200),
            Some(SchedulerEvent {
                kind: SchedulerEventKind::Stepper(0),
                wake_clock: 1000,
            })
        );
        assert_eq!(
            s.pop_due(1200),
            Some(SchedulerEvent {
                kind: SchedulerEventKind::TrSyncReport(0),
                wake_clock: 1050,
            })
        );
        assert_eq!(
            s.pop_due(1200),
            Some(SchedulerEvent {
                kind: SchedulerEventKind::EndstopSample(0),
                wake_clock: 1100,
            })
        );
        assert_eq!(s.pop_due(1200), None);
    }

    #[test]
    fn pop_due_respects_wraparound_clock_order() {
        let mut s = TriggerScheduler::new();
        insert(&mut s, SchedulerEvent {
            kind: SchedulerEventKind::EndstopSample(0),
            wake_clock: 0x0000_0040,
        });
        insert(&mut s, SchedulerEvent {
            kind: SchedulerEventKind::Stepper(0),
            wake_clock: 0xFFFF_FFF0,
        });

        assert_eq!(
            s.pop_due(0x0000_0080),
            Some(SchedulerEvent {
                kind: SchedulerEventKind::Stepper(0),
                wake_clock: 0xFFFF_FFF0,
            })
        );
        assert_eq!(
            s.pop_due(0x0000_0080),
            Some(SchedulerEvent {
                kind: SchedulerEventKind::EndstopSample(0),
                wake_clock: 0x0000_0040,
            })
        );
        assert_eq!(s.pop_due(0x0000_0080), None);
    }

    #[test]
    fn cancel_removes_stale_stepper_event_without_clearing_triggers() {
        let mut s = TriggerScheduler::new();
        insert(&mut s, SchedulerEvent {
            kind: SchedulerEventKind::Stepper(0),
            wake_clock: 1000,
        });
        insert(&mut s, SchedulerEvent {
            kind: SchedulerEventKind::TrSyncExpire(0),
            wake_clock: 1200,
        });

        let cancelled = s.cancel(|kind| matches!(kind, SchedulerEventKind::Stepper(0)));

        assert_eq!(cancelled, 1);
        assert_eq!(
            s.pop_due(1300),
            Some(SchedulerEvent {
                kind: SchedulerEventKind::TrSyncExpire(0),
                wake_clock: 1200,
            })
        );
        assert_eq!(s.pop_due(1300), None);
    }

    #[test]
    fn timer_arm_clock_keeps_safe_future_deadline() {
        assert_eq!(timer_arm_clock(1000, 1200, 128), 1200);
    }

    #[test]
    fn timer_arm_clock_clamps_near_future_deadline() {
        assert_eq!(timer_arm_clock(1000, 1050, 128), 1128);
    }

    #[test]
    fn timer_arm_clock_clamps_past_deadline() {
        assert_eq!(timer_arm_clock(1000, 990, 128), 1128);
    }

    #[test]
    fn timer_arm_clock_handles_wraparound_deadline() {
        assert_eq!(timer_arm_clock(0xFFFF_FF00, 0x0000_0010, 128), 0x0000_0010);
        assert_eq!(timer_arm_clock(0xFFFF_FF00, 0xFFFF_FF10, 128), 0xFFFF_FF80);
    }
}
