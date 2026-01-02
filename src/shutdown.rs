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

/// Static shutdown reason strings.
///
/// These are registered with anchor's static string system and
/// referenced by numeric ID in the `shutdown` message to the host.
pub mod reasons {
    /// Host sent `emergency_stop` command.
    pub const COMMAND_REQUEST: &str = "Command request";
    /// TMC4671 CHIPINFO_SI_TYPE did not match expected value.
    pub const TMC_CHIP_ID_MISMATCH: &str = "TMC4671 chip identification failed";
    /// TMC4671 STATUS_FLAGS ADC_I_CLIPPED bit set.
    pub const TMC_ADC_CLIPPED: &str = "TMC4671 ADC current clipped";
    /// TMC4671 SPI communication failed after retries.
    pub const TMC_SPI_ERROR: &str = "TMC4671 SPI communication failed";
    /// Step move queue is full, cannot accept more moves.
    pub const MOVE_QUEUE_OVERFLOW: &str = "Move queue overflow";
    /// Stepper next step time is more than 1000us in the past.
    pub const STEPPER_TOO_FAR_PAST: &str = "Stepper too far in past";
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

    #[test]
    fn shutdown_reason_strings_are_not_empty() {
        assert!(!reasons::COMMAND_REQUEST.is_empty());
        assert!(!reasons::TMC_CHIP_ID_MISMATCH.is_empty());
        assert!(!reasons::TMC_ADC_CLIPPED.is_empty());
        assert!(!reasons::TMC_SPI_ERROR.is_empty());
        assert!(!reasons::MOVE_QUEUE_OVERFLOW.is_empty());
        assert!(!reasons::STEPPER_TOO_FAR_PAST.is_empty());
    }
}
