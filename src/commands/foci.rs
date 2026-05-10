//! FOCI motor binding config commands.

use crate::commands::CommandError;
pub use crate::commands::digital_out::config_enable_channel;
pub use crate::commands::stepper::config_stepper_channel;
use crate::state::{FociMotorBinding, NUM_FOCI_MOTORS, State};

/// Register a FOCI TMC driver object and bind it to a stepper/channel.
pub fn config_foci_tmc(
    state: &mut State,
    oid: u8,
    stepper_oid: u8,
    channel: u8,
    channel_count: u8,
) -> Result<FociMotorBinding, CommandError> {
    if channel >= channel_count {
        return Err(CommandError::InvalidChannel {
            channel,
            channel_count,
        });
    }
    if state.foci_by_oid.contains_key(&oid) {
        return Err(CommandError::DuplicateOid(oid));
    }
    match state.stepper_channel(stepper_oid) {
        Some(stepper_channel) if stepper_channel == channel => {}
        Some(_) => {
            return Err(CommandError::ChannelMismatch {
                stepper_oid,
                channel,
            });
        }
        None => return Err(CommandError::UnknownOid(stepper_oid)),
    }
    if state
        .foci_bindings
        .iter()
        .flatten()
        .any(|binding| binding.stepper_oid == stepper_oid)
    {
        return Err(CommandError::DuplicateStepper(stepper_oid));
    }
    if state
        .foci_bindings
        .iter()
        .flatten()
        .any(|binding| binding.channel == channel)
    {
        return Err(CommandError::DuplicateChannel(channel));
    }

    let idx = state.foci_by_oid.len();
    if idx >= NUM_FOCI_MOTORS {
        return Err(CommandError::SlotsFull);
    }
    let binding = FociMotorBinding {
        oid,
        stepper_oid,
        enable_oid: enable_oid_for_channel(state, channel),
        channel,
    };
    state
        .foci_by_oid
        .insert(oid, idx)
        .map_err(|_| CommandError::SlotsFull)?;
    state.foci_bindings[idx] = Some(binding);
    Ok(binding)
}

fn enable_oid_for_channel(state: &State, channel: u8) -> Option<u8> {
    state.enable_by_oid.iter().find_map(|(oid, idx)| {
        if state.enable_channels[*idx] == Some(channel) {
            Some(*oid)
        } else {
            None
        }
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::state::State;

    #[test]
    fn config_foci_tmc_accepts_matching_stepper_and_channel() {
        let mut state = State::new();
        config_stepper_channel(&mut state, 10, 0).unwrap();
        config_enable_channel(&mut state, 11, 0).unwrap();

        let binding = config_foci_tmc(&mut state, 20, 10, 0, 1).unwrap();

        assert_eq!(binding.oid, 20);
        assert_eq!(binding.stepper_oid, 10);
        assert_eq!(binding.channel, 0);
        assert_eq!(state.foci_channel_for_oid(20), Some(0));
    }

    #[test]
    fn config_foci_tmc_rejects_channel_mismatch() {
        let mut state = State::new();
        config_stepper_channel(&mut state, 10, 0).unwrap();
        config_enable_channel(&mut state, 11, 0).unwrap();

        assert_eq!(
            config_foci_tmc(&mut state, 20, 10, 1, 2),
            Err(CommandError::ChannelMismatch {
                stepper_oid: 10,
                channel: 1
            })
        );
    }

    #[test]
    fn config_foci_tmc_rejects_duplicate_channel_and_stepper() {
        let mut state = State::new();
        config_stepper_channel(&mut state, 10, 0).unwrap();
        config_enable_channel(&mut state, 11, 0).unwrap();
        config_foci_tmc(&mut state, 20, 10, 0, 1).unwrap();

        assert_eq!(
            config_foci_tmc(&mut state, 21, 10, 0, 1),
            Err(CommandError::DuplicateStepper(10))
        );
    }

    #[test]
    fn config_foci_tmc_rejects_channel_outside_board_range() {
        let mut state = State::new();
        config_stepper_channel(&mut state, 10, 1).unwrap();

        assert_eq!(
            config_foci_tmc(&mut state, 20, 10, 1, 1),
            Err(CommandError::InvalidChannel {
                channel: 1,
                channel_count: 1
            })
        );
    }

    #[test]
    fn config_foci_tmc_rejects_unknown_stepper_oid() {
        let mut state = State::new();

        assert_eq!(
            config_foci_tmc(&mut state, 20, 10, 0, 1),
            Err(CommandError::UnknownOid(10))
        );
    }

    #[test]
    fn config_foci_tmc_rejects_duplicate_foci_oid() {
        let mut state = State::new();
        config_stepper_channel(&mut state, 10, 0).unwrap();
        config_stepper_channel(&mut state, 12, 1).unwrap();
        config_foci_tmc(&mut state, 20, 10, 0, 2).unwrap();

        assert_eq!(
            config_foci_tmc(&mut state, 20, 12, 1, 2),
            Err(CommandError::DuplicateOid(20))
        );
    }

    #[test]
    fn config_foci_tmc_rejects_duplicate_channel_ownership() {
        let mut state = State::new();
        config_stepper_channel(&mut state, 10, 0).unwrap();
        config_stepper_channel(&mut state, 12, 0).unwrap();
        config_foci_tmc(&mut state, 20, 10, 0, 1).unwrap();

        assert_eq!(
            config_foci_tmc(&mut state, 21, 12, 0, 1),
            Err(CommandError::DuplicateChannel(0))
        );
    }
}
