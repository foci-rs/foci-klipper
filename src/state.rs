//! MCU application state container.

use crate::endstop::Endstop;
use crate::shutdown::McuState;
use crate::trsync::TrSync;
use heapless::LinearMap;

/// Number of stepper objects supported.
pub const NUM_STEPPERS: usize = 2;
/// Number of TrSync objects supported.
pub const NUM_TRSYNC: usize = 4;
/// Number of endstop objects supported.
pub const NUM_ENDSTOPS: usize = 4;

/// MCU application state.
///
/// Contains all objects managed by the Klipper protocol:
/// digital outputs, endstops, TrSync instances, and configuration state.
/// Stepper scheduling is handled by the board crate via lock-free statics
/// and a hardware timer ISR.
pub struct State {
    /// Current MCU lifecycle state.
    pub mcu_state: McuState,
    /// Configuration CRC from finalize_config.
    pub config_crc: Option<u32>,
    /// Map from Klipper OID to stepper array index.
    pub steppers_by_oid: LinearMap<u8, usize, NUM_STEPPERS>,
    /// Map from digital output OID to stepper array index (for enable/disable).
    pub enable_by_oid: LinearMap<u8, usize, NUM_STEPPERS>,
    /// TrSync (Trigger Synchronization) objects.
    pub trsync: [TrSync; NUM_TRSYNC],
    /// Map from Klipper OID to TrSync array index.
    pub trsync_by_oid: LinearMap<u8, usize, NUM_TRSYNC>,
    /// Endstop objects.
    pub endstops: [Endstop; NUM_ENDSTOPS],
    /// Map from Klipper OID to endstop array index.
    pub endstop_by_oid: LinearMap<u8, usize, NUM_ENDSTOPS>,
    /// Stepper-to-TrSync linkage for `stepper_stop_on_trigger`.
    ///
    /// Each entry is `(stepper_oid, trsync_oid)`.
    pub stepper_trsync_links: heapless::Vec<(u8, u8), 8>,
}

impl State {
    /// Create a new state with all objects in their default configuration.
    pub fn new() -> Self {
        Self {
            mcu_state: McuState::Configuring,
            config_crc: None,
            steppers_by_oid: LinearMap::new(),
            enable_by_oid: LinearMap::new(),
            trsync: core::array::from_fn(|_| TrSync::new()),
            trsync_by_oid: LinearMap::new(),
            endstops: core::array::from_fn(|_| Endstop::new()),
            endstop_by_oid: LinearMap::new(),
            stepper_trsync_links: heapless::Vec::new(),
        }
    }

    /// Look up a TrSync by OID.
    pub fn trsync(&self, oid: u8) -> Option<&TrSync> {
        self.trsync_by_oid.get(&oid).map(|&idx| &self.trsync[idx])
    }

    /// Look up a mutable TrSync by OID.
    pub fn trsync_mut(&mut self, oid: u8) -> Option<&mut TrSync> {
        self.trsync_by_oid
            .get(&oid)
            .copied()
            .map(move |idx| &mut self.trsync[idx])
    }

    /// Look up an endstop by OID.
    pub fn endstop(&self, oid: u8) -> Option<&Endstop> {
        self.endstop_by_oid
            .get(&oid)
            .map(|&idx| &self.endstops[idx])
    }

    /// Look up a mutable endstop by OID.
    pub fn endstop_mut(&mut self, oid: u8) -> Option<&mut Endstop> {
        self.endstop_by_oid
            .get(&oid)
            .copied()
            .map(move |idx| &mut self.endstops[idx])
    }

    /// Reset all state for reconfiguration (after shutdown recovery).
    pub fn reset(&mut self) {
        self.mcu_state = McuState::Configuring;
        self.config_crc = None;
        self.steppers_by_oid.clear();
        self.enable_by_oid.clear();
        self.trsync_by_oid.clear();
        for t in &mut self.trsync {
            t.reset();
        }
        self.endstop_by_oid.clear();
        for e in &mut self.endstops {
            *e = Endstop::new();
        }
        self.stepper_trsync_links.clear();
    }
}

impl anchor::ShutdownState for State {
    fn is_shutdown(&self) -> bool {
        self.mcu_state.is_shutdown()
    }
}

impl Default for State {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn new_state_is_configuring() {
        let state = State::new();
        assert_eq!(state.mcu_state, McuState::Configuring);
        assert!(state.config_crc.is_none());
    }

    #[test]
    fn trsync_oid_lookup() {
        let mut state = State::new();
        state.trsync_by_oid.insert(10, 0).unwrap();
        state.trsync_by_oid.insert(11, 1).unwrap();

        assert!(state.trsync(10).is_some());
        assert!(state.trsync(11).is_some());
        assert!(state.trsync(12).is_none());
    }

    #[test]
    fn trsync_mut_oid_lookup() {
        let mut state = State::new();
        state.trsync_by_oid.insert(20, 0).unwrap();

        let ts = state.trsync_mut(20).unwrap();
        ts.arm(1000, 500, 1);
        assert_eq!(state.trsync[0].state(), crate::trsync::TrSyncState::Armed);
    }

    #[test]
    fn endstop_oid_lookup() {
        let mut state = State::new();
        state.endstop_by_oid.insert(30, 0).unwrap();
        state.endstop_by_oid.insert(31, 1).unwrap();

        assert!(state.endstop(30).is_some());
        assert!(state.endstop(31).is_some());
        assert!(state.endstop(32).is_none());
    }

    #[test]
    fn endstop_mut_oid_lookup() {
        let mut state = State::new();
        state.endstop_by_oid.insert(40, 0).unwrap();

        let e = state.endstop_mut(40).unwrap();
        e.start_homing(1000, 500, 3, 100, true, 0, 1);
        assert!(state.endstops[0].is_homing());
    }

    #[test]
    fn reset_clears_everything() {
        let mut state = State::new();
        state.mcu_state = McuState::Shutdown { reason: 1 };
        state.config_crc = Some(0xDEAD);
        state.steppers_by_oid.insert(0, 0).unwrap();
        state.enable_by_oid.insert(1, 0).unwrap();
        state.trsync_by_oid.insert(10, 0).unwrap();
        state.trsync[0].arm(1000, 500, 1);
        state.endstop_by_oid.insert(20, 0).unwrap();
        state.endstops[0].start_homing(1000, 500, 3, 100, true, 0, 1);
        state.stepper_trsync_links.push((0, 10)).unwrap();

        state.reset();

        assert_eq!(state.mcu_state, McuState::Configuring);
        assert!(state.config_crc.is_none());
        assert!(state.steppers_by_oid.is_empty());
        assert!(state.enable_by_oid.is_empty());
        assert!(state.trsync_by_oid.is_empty());
        assert_eq!(state.trsync[0].state(), crate::trsync::TrSyncState::Idle);
        assert!(state.endstop_by_oid.is_empty());
        assert!(!state.endstops[0].is_homing());
        assert!(state.stepper_trsync_links.is_empty());
    }

    #[test]
    fn stepper_trsync_links_stores_pairs() {
        let mut state = State::new();
        state.stepper_trsync_links.push((0, 10)).unwrap();
        state.stepper_trsync_links.push((1, 11)).unwrap();

        assert_eq!(state.stepper_trsync_links.len(), 2);
        assert_eq!(state.stepper_trsync_links[0], (0, 10));
        assert_eq!(state.stepper_trsync_links[1], (1, 11));
    }

    #[test]
    fn default_is_same_as_new() {
        let a = State::new();
        let b = State::default();
        assert_eq!(a.mcu_state, b.mcu_state);
        assert_eq!(a.config_crc, b.config_crc);
    }

    #[test]
    fn shutdown_state_configuring() {
        use anchor::ShutdownState;
        let state = State::new();
        assert!(!state.is_shutdown());
    }

    #[test]
    fn shutdown_state_running() {
        use anchor::ShutdownState;
        let mut state = State::new();
        state.mcu_state = McuState::Running;
        assert!(!state.is_shutdown());
    }

    #[test]
    fn shutdown_state_shutdown() {
        use anchor::ShutdownState;
        let mut state = State::new();
        state.mcu_state = McuState::Shutdown { reason: 0 };
        assert!(state.is_shutdown());
    }
}
