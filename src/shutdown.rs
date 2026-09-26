//! MCU shutdown state machine matching Klipper's lifecycle.

/// MCU operational state matching Klipper's lifecycle.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum McuState {
    /// Accepting configuration commands (before finalize_config).
    Configuring,
    /// Normal operation (after finalize_config).
    Running,
    /// Shutdown state. Still responds to get_clock, get_config, emergency_stop.
    ///
    /// The reason is a static string ID matching Klipper's convention.
    Shutdown { reason: u16 },
}

impl McuState {
    /// Returns true if the MCU is in shutdown state.
    pub fn is_shutdown(&self) -> bool {
        matches!(self, McuState::Shutdown { .. })
    }

    /// Returns true if the MCU has been configured (Running or Shutdown).
    pub fn is_configured(&self) -> bool {
        !matches!(self, McuState::Configuring)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn initial_state_is_configuring() {
        let state = McuState::Configuring;
        assert!(!state.is_shutdown());
        assert!(!state.is_configured());
    }

    #[test]
    fn running_state() {
        let state = McuState::Running;
        assert!(!state.is_shutdown());
        assert!(state.is_configured());
    }

    #[test]
    fn shutdown_state() {
        let state = McuState::Shutdown { reason: 42 };
        assert!(state.is_shutdown());
        assert!(state.is_configured());
    }
}
