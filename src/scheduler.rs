//! Software timer scheduler for Klipper-clock events.
//!
//! Manages a fixed-capacity event table driven by one hardware compare
//! channel. All clock comparisons use signed 32-bit wraparound.

/// Maximum scheduler events: NUM_STEPPERS + NUM_ENDSTOPS + 2 * NUM_TRSYNC.
/// With 1 physical stepper, 4 endstops, and 4 trsync = 1 + 4 + 8 = 13.
pub const MAX_SCHEDULER_EVENTS: usize = 13;

const NIL: u8 = u8::MAX;

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

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct ScheduledNode {
    event: SchedulerEvent,
    next: u8,
}

/// Result of inserting a scheduler event.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ScheduleResult {
    /// Event was inserted. `new_earliest` means hardware compare must move.
    Inserted { new_earliest: bool },
    /// The fixed scheduler table had no free slot.
    Full,
}

/// Token for a popped scheduler slot that can be requeued by the same dispatch.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ScheduledSlot {
    index: u8,
    event: SchedulerEvent,
}

impl ScheduledSlot {
    /// Return the event carried by this popped slot.
    pub fn event(&self) -> SchedulerEvent {
        self.event
    }
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

fn is_due(now: u32, wake_clock: u32) -> bool {
    (now.wrapping_sub(wake_clock) as i32) >= 0
}

fn is_before(a: u32, b: u32) -> bool {
    (a.wrapping_sub(b) as i32) < 0
}

fn is_before_or_equal(a: u32, b: u32) -> bool {
    (a.wrapping_sub(b) as i32) <= 0
}

/// Single-owner, externally synchronized timer queue.
///
/// All mutation must hold the trigger-domain lock (`with_trigger_domain_site`
/// on OpenFFBoard, equivalent on other boards). No internal atomics; no `Sync`
/// guarantee beyond the lock.
///
/// Invariants:
/// - `head == NIL` means the queue is empty.
/// - A slot is either `None` and free, or `Some` and reachable from `head`.
/// - There are no orphan `Some` nodes outside the `head` chain.
pub struct TriggerScheduler {
    nodes: [Option<ScheduledNode>; MAX_SCHEDULER_EVENTS],
    head: u8,
    /// Whether the hardware compare (CC2) is currently enabled.
    cc2_active: bool,
}

impl TriggerScheduler {
    pub const fn new() -> Self {
        Self {
            nodes: [None; MAX_SCHEDULER_EVENTS],
            head: NIL,
            cc2_active: false,
        }
    }

    pub fn cc2_active(&self) -> bool {
        self.cc2_active
    }

    pub fn set_cc2_active(&mut self, active: bool) {
        self.cc2_active = active;
    }

    fn insert_node(&mut self, index: u8, event: SchedulerEvent) -> bool {
        let old_head = self.head;
        self.nodes[index as usize] = Some(ScheduledNode { event, next: NIL });

        if self.head == NIL {
            self.head = index;
            return true;
        }

        let head_event = self.nodes[self.head as usize]
            .as_ref()
            .expect("head node exists")
            .event;
        if is_before(event.wake_clock, head_event.wake_clock) {
            self.nodes[index as usize]
                .as_mut()
                .expect("inserted node exists")
                .next = self.head;
            self.head = index;
            return true;
        }

        let mut prev = self.head;
        let mut next = self.nodes[prev as usize]
            .as_ref()
            .expect("linked node exists")
            .next;
        while next != NIL {
            let next_node = self.nodes[next as usize]
                .as_ref()
                .expect("linked node exists");
            if !is_before_or_equal(next_node.event.wake_clock, event.wake_clock) {
                break;
            }
            prev = next;
            next = next_node.next;
        }

        self.nodes[index as usize]
            .as_mut()
            .expect("inserted node exists")
            .next = next;
        self.nodes[prev as usize]
            .as_mut()
            .expect("previous node exists")
            .next = index;
        old_head != self.head
    }

    /// Schedule an event with explicit full-table reporting.
    pub fn try_schedule(&mut self, event: SchedulerEvent) -> ScheduleResult {
        let Some(index) = self.nodes.iter().position(|node| node.is_none()) else {
            return ScheduleResult::Full;
        };
        let new_earliest = self.insert_node(index as u8, event);
        ScheduleResult::Inserted { new_earliest }
    }

    /// Cancel all events matching a filter. Returns number cancelled.
    pub fn cancel<F: Fn(&SchedulerEventKind) -> bool>(&mut self, filter: F) -> usize {
        let mut count = 0;
        let mut prev = NIL;
        let mut current = self.head;

        while current != NIL {
            let node = self.nodes[current as usize].expect("linked node exists");
            let next = node.next;
            if filter(&node.event.kind) {
                if prev == NIL {
                    self.head = next;
                } else {
                    self.nodes[prev as usize]
                        .as_mut()
                        .expect("previous node exists")
                        .next = next;
                }
                self.nodes[current as usize] = None;
                count += 1;
            } else {
                prev = current;
            }
            current = next;
        }

        count
    }

    /// Return the earliest active deadline.
    pub fn earliest_deadline(&self) -> Option<u32> {
        if self.head == NIL {
            None
        } else {
            self.nodes[self.head as usize].map(|node| node.event.wake_clock)
        }
    }

    /// Remove and return the earliest due event.
    ///
    /// If multiple events are due at `now`, this follows the same ordering
    /// used by [`Self::earliest_deadline`], so callers can repeatedly pop until it
    /// returns `None` and process all ready work in scheduler order.
    pub fn pop_due(&mut self, now: u32) -> Option<SchedulerEvent> {
        self.pop_due_slot(now).map(|slot| slot.event())
    }

    /// Remove and return the earliest due event with its reusable slot token.
    pub fn pop_due_slot(&mut self, now: u32) -> Option<ScheduledSlot> {
        if self.head == NIL {
            return None;
        }
        let index = self.head;
        let node = self.nodes[index as usize]?;
        if !is_due(now, node.event.wake_clock) {
            return None;
        }
        self.head = node.next;
        self.nodes[index as usize] = None;
        Some(ScheduledSlot {
            index,
            event: node.event,
        })
    }

    /// Requeue a previously popped slot with the same event kind and new clock.
    pub fn requeue_slot(&mut self, slot: ScheduledSlot, new_clock: u32) -> ScheduleResult {
        let index = slot.index as usize;
        if index >= MAX_SCHEDULER_EVENTS || self.nodes[index].is_some() {
            return self.try_schedule(SchedulerEvent {
                kind: slot.event.kind,
                wake_clock: new_clock,
            });
        }
        let event = SchedulerEvent {
            kind: slot.event.kind,
            wake_clock: new_clock,
        };
        let new_earliest = self.insert_node(slot.index, event);
        ScheduleResult::Inserted { new_earliest }
    }

    /// Check if any events are active.
    pub fn is_empty(&self) -> bool {
        self.head == NIL
    }

    /// Check whether every scheduler slot is occupied.
    pub fn is_full(&self) -> bool {
        self.nodes.iter().all(Option::is_some)
    }

    /// Clear all events.
    pub fn clear(&mut self) {
        for slot in &mut self.nodes {
            *slot = None;
        }
        self.head = NIL;
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
    fn earliest_deadline_correct() {
        let mut s = TriggerScheduler::new();
        insert(
            &mut s,
            SchedulerEvent {
                kind: SchedulerEventKind::TrSyncExpire(0),
                wake_clock: 5000,
            },
        );
        insert(
            &mut s,
            SchedulerEvent {
                kind: SchedulerEventKind::EndstopSample(0),
                wake_clock: 2000,
            },
        );
        assert_eq!(s.earliest_deadline(), Some(2000));
    }

    #[test]
    fn cancel_by_kind() {
        let mut s = TriggerScheduler::new();
        insert(
            &mut s,
            SchedulerEvent {
                kind: SchedulerEventKind::TrSyncExpire(0),
                wake_clock: 1000,
            },
        );
        insert(
            &mut s,
            SchedulerEvent {
                kind: SchedulerEventKind::TrSyncReport(0),
                wake_clock: 2000,
            },
        );
        insert(
            &mut s,
            SchedulerEvent {
                kind: SchedulerEventKind::EndstopSample(1),
                wake_clock: 3000,
            },
        );
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
    fn wraparound_earliest_deadline() {
        let mut s = TriggerScheduler::new();
        insert(
            &mut s,
            SchedulerEvent {
                kind: SchedulerEventKind::EndstopSample(0),
                wake_clock: 0x0000_0100,
            },
        );
        insert(
            &mut s,
            SchedulerEvent {
                kind: SchedulerEventKind::TrSyncExpire(0),
                wake_clock: 0xFFFF_FF00,
            },
        );
        assert_eq!(s.earliest_deadline(), Some(0xFFFF_FF00));
    }

    #[test]
    fn clear_removes_all() {
        let mut s = TriggerScheduler::new();
        insert(
            &mut s,
            SchedulerEvent {
                kind: SchedulerEventKind::EndstopSample(0),
                wake_clock: 1000,
            },
        );
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
    fn equal_wake_clock_events_fire_in_insertion_order() {
        let mut s = TriggerScheduler::new();
        assert!(matches!(
            s.try_schedule(SchedulerEvent {
                kind: SchedulerEventKind::EndstopSample(0),
                wake_clock: 1000,
            }),
            ScheduleResult::Inserted { .. }
        ));
        assert!(matches!(
            s.try_schedule(SchedulerEvent {
                kind: SchedulerEventKind::TrSyncReport(0),
                wake_clock: 1000,
            }),
            ScheduleResult::Inserted { .. }
        ));

        assert_eq!(
            s.pop_due(1000),
            Some(SchedulerEvent {
                kind: SchedulerEventKind::EndstopSample(0),
                wake_clock: 1000,
            })
        );
        assert_eq!(
            s.pop_due(1000),
            Some(SchedulerEvent {
                kind: SchedulerEventKind::TrSyncReport(0),
                wake_clock: 1000,
            })
        );
    }

    #[test]
    fn pop_due_checks_head_only() {
        let mut s = TriggerScheduler::new();
        assert!(matches!(
            s.try_schedule(SchedulerEvent {
                kind: SchedulerEventKind::Stepper(0),
                wake_clock: 2000,
            }),
            ScheduleResult::Inserted { .. }
        ));
        assert!(matches!(
            s.try_schedule(SchedulerEvent {
                kind: SchedulerEventKind::EndstopSample(0),
                wake_clock: 3000,
            }),
            ScheduleResult::Inserted { .. }
        ));

        assert_eq!(s.pop_due(1500), None);
        assert_eq!(s.earliest_deadline(), Some(2000));
    }

    #[test]
    fn try_schedule_inserts_at_middle_when_clock_between_others() {
        let mut s = TriggerScheduler::new();
        for (kind, wake_clock) in [
            (SchedulerEventKind::EndstopSample(0), 1000),
            (SchedulerEventKind::TrSyncReport(0), 3000),
            (SchedulerEventKind::Stepper(0), 2000),
        ] {
            assert!(matches!(
                s.try_schedule(SchedulerEvent { kind, wake_clock }),
                ScheduleResult::Inserted { .. }
            ));
        }

        assert_eq!(s.pop_due(4000).unwrap().wake_clock, 1000);
        assert_eq!(s.pop_due(4000).unwrap().wake_clock, 2000);
        assert_eq!(s.pop_due(4000).unwrap().wake_clock, 3000);
    }

    #[test]
    fn sorted_insert_preserves_signed_wraparound_order() {
        let mut s = TriggerScheduler::new();
        assert!(matches!(
            s.try_schedule(SchedulerEvent {
                kind: SchedulerEventKind::EndstopSample(0),
                wake_clock: 0x0000_0040,
            }),
            ScheduleResult::Inserted { .. }
        ));
        assert!(matches!(
            s.try_schedule(SchedulerEvent {
                kind: SchedulerEventKind::Stepper(0),
                wake_clock: 0xFFFF_FFF0,
            }),
            ScheduleResult::Inserted { new_earliest: true }
        ));

        assert_eq!(s.pop_due(0x0000_0080).unwrap().wake_clock, 0xFFFF_FFF0);
        assert_eq!(s.pop_due(0x0000_0080).unwrap().wake_clock, 0x0000_0040);
    }

    #[test]
    fn cancel_of_head_updates_head_pointer() {
        let mut s = TriggerScheduler::new();
        for wake_clock in [1000, 2000, 3000] {
            assert!(matches!(
                s.try_schedule(SchedulerEvent {
                    kind: SchedulerEventKind::EndstopSample((wake_clock / 1000) as usize),
                    wake_clock,
                }),
                ScheduleResult::Inserted { .. }
            ));
        }

        assert_eq!(
            s.cancel(|kind| matches!(kind, SchedulerEventKind::EndstopSample(1))),
            1
        );
        assert_eq!(s.earliest_deadline(), Some(2000));
    }

    #[test]
    fn cancel_of_middle_preserves_chain() {
        let mut s = TriggerScheduler::new();
        for wake_clock in [1000, 2000, 3000] {
            assert!(matches!(
                s.try_schedule(SchedulerEvent {
                    kind: SchedulerEventKind::EndstopSample((wake_clock / 1000) as usize),
                    wake_clock,
                }),
                ScheduleResult::Inserted { .. }
            ));
        }

        assert_eq!(
            s.cancel(|kind| matches!(kind, SchedulerEventKind::EndstopSample(2))),
            1
        );
        assert_eq!(s.pop_due(4000).unwrap().wake_clock, 1000);
        assert_eq!(s.pop_due(4000).unwrap().wake_clock, 3000);
        assert_eq!(s.pop_due(4000), None);
    }

    #[test]
    fn cancel_of_tail_keeps_head_valid() {
        let mut s = TriggerScheduler::new();
        for wake_clock in [1000, 2000, 3000] {
            assert!(matches!(
                s.try_schedule(SchedulerEvent {
                    kind: SchedulerEventKind::EndstopSample((wake_clock / 1000) as usize),
                    wake_clock,
                }),
                ScheduleResult::Inserted { .. }
            ));
        }

        assert_eq!(
            s.cancel(|kind| matches!(kind, SchedulerEventKind::EndstopSample(3))),
            1
        );
        assert_eq!(s.earliest_deadline(), Some(1000));
        assert_eq!(s.pop_due(4000).unwrap().wake_clock, 1000);
        assert_eq!(s.pop_due(4000).unwrap().wake_clock, 2000);
        assert_eq!(s.pop_due(4000), None);
    }

    #[test]
    fn pop_due_slot_requeue_slot_preserves_event_order() {
        let mut s = TriggerScheduler::new();
        assert!(matches!(
            s.try_schedule(SchedulerEvent {
                kind: SchedulerEventKind::Stepper(0),
                wake_clock: 1000,
            }),
            ScheduleResult::Inserted { .. }
        ));
        let slot = s.pop_due_slot(1000).expect("stepper due");
        assert_eq!(slot.event().kind, SchedulerEventKind::Stepper(0));
        assert!(matches!(
            s.requeue_slot(slot, 2000),
            ScheduleResult::Inserted { new_earliest: true }
        ));
        assert_eq!(s.earliest_deadline(), Some(2000));
    }

    #[test]
    fn requeue_slot_inserts_after_earlier_event() {
        let mut s = TriggerScheduler::new();
        assert!(matches!(
            s.try_schedule(SchedulerEvent {
                kind: SchedulerEventKind::Stepper(0),
                wake_clock: 1000,
            }),
            ScheduleResult::Inserted { .. }
        ));
        assert!(matches!(
            s.try_schedule(SchedulerEvent {
                kind: SchedulerEventKind::TrSyncReport(0),
                wake_clock: 1500,
            }),
            ScheduleResult::Inserted { .. }
        ));
        let slot = s.pop_due_slot(1000).expect("stepper due");
        assert!(matches!(
            s.requeue_slot(slot, 2000),
            ScheduleResult::Inserted {
                new_earliest: false,
            }
        ));
        assert_eq!(s.pop_due(2500).unwrap().wake_clock, 1500);
        assert_eq!(s.pop_due(2500).unwrap().wake_clock, 2000);
    }

    #[test]
    fn pop_due_processes_stepper_and_trigger_events_in_clock_order() {
        let mut s = TriggerScheduler::new();
        insert(
            &mut s,
            SchedulerEvent {
                kind: SchedulerEventKind::EndstopSample(0),
                wake_clock: 1100,
            },
        );
        insert(
            &mut s,
            SchedulerEvent {
                kind: SchedulerEventKind::Stepper(0),
                wake_clock: 1000,
            },
        );
        insert(
            &mut s,
            SchedulerEvent {
                kind: SchedulerEventKind::TrSyncReport(0),
                wake_clock: 1050,
            },
        );

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
        insert(
            &mut s,
            SchedulerEvent {
                kind: SchedulerEventKind::EndstopSample(0),
                wake_clock: 0x0000_0040,
            },
        );
        insert(
            &mut s,
            SchedulerEvent {
                kind: SchedulerEventKind::Stepper(0),
                wake_clock: 0xFFFF_FFF0,
            },
        );

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
        insert(
            &mut s,
            SchedulerEvent {
                kind: SchedulerEventKind::Stepper(0),
                wake_clock: 1000,
            },
        );
        insert(
            &mut s,
            SchedulerEvent {
                kind: SchedulerEventKind::TrSyncExpire(0),
                wake_clock: 1200,
            },
        );

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
