//! MCU lifecycle and protocol query commands.

use crate::commands::ShutdownRequired;
use crate::shutdown::McuState;
use crate::state::State;

/// Return protocol config state for `get_config` reply.
///
/// Returns `(is_config, crc, is_shutdown, move_count)`.
pub fn get_config(state: &State) -> (bool, u32, bool, u16) {
    let is_config = state.config_crc.is_some();
    let crc = state.config_crc.unwrap_or(0);
    let is_shutdown = state.mcu_state.is_shutdown();
    (is_config, crc, is_shutdown, 1024)
}

/// Clear shutdown state, transitioning back to Running.
///
/// Returns `Err(ShutdownRequired)` if the MCU is not in shutdown state.
/// The firmware wrapper should trigger a shutdown in that case.
pub fn clear_shutdown(state: &mut State) -> Result<(), ShutdownRequired> {
    if !state.mcu_state.is_shutdown() {
        return Err(ShutdownRequired);
    }
    state.mcu_state = McuState::Running;
    Ok(())
}

/// Transition to shutdown state.
///
/// Returns `true` if this call actually entered shutdown (first call).
/// Returns `false` if already in shutdown. The firmware wrapper should
/// only send `klipper_shutdown!` when this returns `true`.
pub fn enter_shutdown(state: &mut State) -> bool {
    if state.mcu_state.is_shutdown() {
        return false;
    }
    state.mcu_state = McuState::Shutdown { reason: 0 };
    true
}

/// Full config reset: clear all OIDs and reset to Configuring state.
///
/// Only valid during shutdown. The firmware wrapper must check
/// `state.mcu_state.is_shutdown()` before calling this and handle
/// the error (trigger shutdown if not in shutdown state).
pub fn config_reset(state: &mut State) {
    state.reset();
}

/// Store config CRC and transition to Running.
pub fn finalize_config(state: &mut State, crc: u32) {
    state.config_crc = Some(crc);
    state.mcu_state = McuState::Running;
}

/// No-op: static allocation is used.
pub fn allocate_oids(_count: u8) {}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn get_config_no_crc() {
        let state = State::new();
        let (is_config, crc, is_shutdown, move_count) = get_config(&state);
        assert!(!is_config);
        assert_eq!(crc, 0);
        assert!(!is_shutdown);
        assert_eq!(move_count, 1024);
    }

    #[test]
    fn get_config_with_crc() {
        let mut state = State::new();
        state.config_crc = Some(0xDEAD_BEEF);
        state.mcu_state = McuState::Running;
        let (is_config, crc, is_shutdown, _) = get_config(&state);
        assert!(is_config);
        assert_eq!(crc, 0xDEAD_BEEF);
        assert!(!is_shutdown);
    }

    #[test]
    fn get_config_in_shutdown() {
        let mut state = State::new();
        state.config_crc = Some(0x1234);
        state.mcu_state = McuState::Shutdown { reason: 0 };
        let (_, _, is_shutdown, _) = get_config(&state);
        assert!(is_shutdown);
    }

    #[test]
    fn enter_shutdown_first_call_returns_true() {
        let mut state = State::new();
        state.mcu_state = McuState::Running;
        assert!(enter_shutdown(&mut state));
        assert!(state.mcu_state.is_shutdown());
    }

    #[test]
    fn enter_shutdown_idempotent() {
        let mut state = State::new();
        state.mcu_state = McuState::Running;
        assert!(enter_shutdown(&mut state));
        assert!(!enter_shutdown(&mut state));
    }

    #[test]
    fn clear_shutdown_when_shutdown() {
        let mut state = State::new();
        state.mcu_state = McuState::Shutdown { reason: 0 };
        assert!(clear_shutdown(&mut state).is_ok());
        assert_eq!(state.mcu_state, McuState::Running);
    }

    #[test]
    fn clear_shutdown_when_not_shutdown() {
        let mut state = State::new();
        state.mcu_state = McuState::Running;
        assert_eq!(clear_shutdown(&mut state), Err(ShutdownRequired));
    }

    #[test]
    fn config_reset_clears_state() {
        let mut state = State::new();
        state.config_crc = Some(0xBEEF);
        state.mcu_state = McuState::Shutdown { reason: 0 };
        state.steppers_by_oid.insert(0, 0).unwrap();
        config_reset(&mut state);
        assert_eq!(state.mcu_state, McuState::Configuring);
        assert!(state.config_crc.is_none());
        assert!(state.steppers_by_oid.is_empty());
    }

    #[test]
    fn finalize_config_stores_crc() {
        let mut state = State::new();
        finalize_config(&mut state, 0xCAFE);
        assert_eq!(state.config_crc, Some(0xCAFE));
        assert_eq!(state.mcu_state, McuState::Running);
    }
}
