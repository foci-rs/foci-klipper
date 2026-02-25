//! Shared Klipper command handler logic.
//!
//! These are plain functions called by firmware `#[klipper_command]` wrappers.
//! They handle state mutation and return data; firmware wrappers handle
//! `klipper_reply!`, `klipper_shutdown!`, and hardware I/O.

pub mod digital_out;
pub mod endstop;
pub mod lifecycle;
pub mod spi;
pub mod stepper;
pub mod trsync;

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
}

/// Returned by [`lifecycle::clear_shutdown`] when called outside shutdown state.
///
/// The firmware wrapper should send `klipper_shutdown!` in response.
#[derive(Debug, Clone, PartialEq, Eq, defmt::Format)]
pub struct ShutdownRequired;
