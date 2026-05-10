//! Shared Klipper command handler logic.
//!
//! These are plain functions called by firmware `#[klipper_command]` wrappers.
//! They handle state mutation and return data; firmware wrappers handle
//! `klipper_reply!`, `klipper_shutdown!`, and hardware I/O.

pub mod digital_out;
pub mod foci;
pub mod lifecycle;
pub mod spi;
pub mod stepper;

/// Errors from shared command handler logic.
#[derive(Debug, Clone, PartialEq, Eq, defmt::Format)]
pub enum CommandError {
    /// OID not found in lookup map.
    UnknownOid(u8),
    /// OID already registered (duplicate config command).
    DuplicateOid(u8),
    /// No slots available for new object registration.
    SlotsFull,
    /// Move queue overflow.
    QueueFull,
    /// Stepper is active (timer armed) when reset attempted.
    StepperActive,
    /// Zero-count move rejected.
    ZeroCount,
    /// Configured channel is outside the board's supported channel range.
    InvalidChannel {
        /// Requested channel.
        channel: u8,
        /// Number of channels supported by the board.
        channel_count: u8,
    },
    /// Stepper OID is already owned by a FOCI binding.
    DuplicateStepper(u8),
    /// TMC4671 channel is already owned by a FOCI binding.
    DuplicateChannel(u8),
    /// Configured FOCI channel does not match the stepper's channel.
    ChannelMismatch {
        /// Stepper OID whose configured channel was checked.
        stepper_oid: u8,
        /// Requested FOCI channel.
        channel: u8,
    },
}

/// Returned by [`lifecycle::clear_shutdown`] when called outside shutdown state.
///
/// The firmware wrapper should send `klipper_shutdown!` in response.
#[derive(Debug, Clone, PartialEq, Eq, defmt::Format)]
pub struct ShutdownRequired;
