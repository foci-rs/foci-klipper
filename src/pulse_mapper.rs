/// Mechanical-position units in one TMC4671 revolution.
pub const POSITION_UNITS_PER_REV: u32 = 65_536;

/// Largest accepted logical planner-step count per mechanical revolution.
pub const MAX_PLANNER_STEPS_PER_REV: u32 = 1 << 24;

/// Errors returned when constructing a [`PulseMapperConfig`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum PulseMapperConfigError {
    /// The planner-step count was zero.
    ZeroPlannerStepsPerRev,
    /// The planner-step count exceeded [`MAX_PLANNER_STEPS_PER_REV`].
    PlannerStepsPerRevTooLarge,
}

/// Exact planner-to-TMC pulse-grid configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct PulseMapperConfig {
    planner_steps_per_rev: u32,
    tmc_grid: u32,
    step_width: u32,
}

impl PulseMapperConfig {
    /// Builds a pulse-grid configuration for a logical planner scale.
    ///
    /// # Errors
    ///
    /// Returns [`PulseMapperConfigError::ZeroPlannerStepsPerRev`] when
    /// `planner_steps_per_rev` is zero, or
    /// [`PulseMapperConfigError::PlannerStepsPerRevTooLarge`] when it exceeds
    /// [`MAX_PLANNER_STEPS_PER_REV`].
    pub fn new(planner_steps_per_rev: u32) -> Result<Self, PulseMapperConfigError> {
        if planner_steps_per_rev == 0 {
            return Err(PulseMapperConfigError::ZeroPlannerStepsPerRev);
        }
        if planner_steps_per_rev > MAX_PLANNER_STEPS_PER_REV {
            return Err(PulseMapperConfigError::PlannerStepsPerRevTooLarge);
        }

        let tmc_grid = if planner_steps_per_rev < POSITION_UNITS_PER_REV {
            planner_steps_per_rev.next_power_of_two()
        } else {
            POSITION_UNITS_PER_REV
        };

        Ok(Self {
            planner_steps_per_rev,
            tmc_grid,
            step_width: POSITION_UNITS_PER_REV / tmc_grid,
        })
    }

    /// Returns the logical planner steps per mechanical revolution.
    pub const fn planner_steps_per_rev(&self) -> u32 {
        self.planner_steps_per_rev
    }

    /// Returns the selected physical TMC pulse grid per mechanical revolution.
    pub const fn tmc_grid(&self) -> u32 {
        self.tmc_grid
    }

    /// Returns the exact TMC4671 `STEP_WIDTH` for each physical pulse.
    pub const fn step_width(&self) -> u32 {
        self.step_width
    }

    /// Returns twice the instantaneous position-error bound, in position units.
    ///
    /// Expressing the bound in half-units preserves the exact `STEP_WIDTH / 2`
    /// value when `STEP_WIDTH` is one.
    pub const fn position_error_bound_half_units(&self) -> u32 {
        self.step_width
    }
}

/// Stateful, reversible mapping from logical planner steps to physical TMC pulses.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct ReversiblePulseMapper {
    config: PulseMapperConfig,
    phase: i64,
}

impl ReversiblePulseMapper {
    /// Establishes a mapper at a common logical and physical target origin.
    pub const fn new(config: PulseMapperConfig) -> Self {
        Self {
            phase: config.planner_steps_per_rev as i64 / 2,
            config,
        }
    }

    /// Reconfigures the scale while establishing a new common target origin.
    ///
    /// Callers must only reconfigure when the logical and physical targets are
    /// known to share an origin.
    pub fn reconfigure(&mut self, config: PulseMapperConfig) {
        *self = Self::new(config);
    }

    /// Maps one logical planner step to a signed physical TMC pulse count.
    ///
    /// `forward` selects a positive logical step when true and a negative step
    /// when false. The result is always in `-2..=2`.
    pub fn map_step(&mut self, forward: bool) -> i8 {
        let direction = if forward { 1_i64 } else { -1_i64 };
        let planner_steps_per_rev = i64::from(self.config.planner_steps_per_rev);
        let total = self.phase + direction * i64::from(self.config.tmc_grid);
        let physical_pulses = total.div_euclid(planner_steps_per_rev);
        self.phase = total.rem_euclid(planner_steps_per_rev);
        physical_pulses as i8
    }

    /// Returns the logical planner steps per mechanical revolution.
    pub const fn planner_steps_per_rev(&self) -> u32 {
        self.config.planner_steps_per_rev()
    }

    /// Returns the selected physical TMC pulse grid per mechanical revolution.
    pub const fn tmc_grid(&self) -> u32 {
        self.config.tmc_grid()
    }

    /// Returns the exact TMC4671 `STEP_WIDTH` for each physical pulse.
    pub const fn step_width(&self) -> u32 {
        self.config.step_width()
    }

    /// Returns the current Euclidean phase remainder in planner-scale units.
    pub const fn phase(&self) -> i64 {
        self.phase
    }

    /// Returns twice the instantaneous position-error bound, in position units.
    ///
    /// Expressing the bound in half-units preserves the exact `STEP_WIDTH / 2`
    /// value when `STEP_WIDTH` is one.
    pub const fn position_error_bound_half_units(&self) -> u32 {
        self.config.position_error_bound_half_units()
    }
}

#[cfg(test)]
mod tests {
    use super::{
        MAX_PLANNER_STEPS_PER_REV, POSITION_UNITS_PER_REV, PulseMapperConfig,
        PulseMapperConfigError, ReversiblePulseMapper,
    };

    const REPRESENTATIVE_STEPS: [u32; 10] = [
        200, 400, 3_200, 4_000, 4_096, 25_600, 51_200, 65_536, 102_400, 16_777_216,
    ];

    fn expected_grid(planner_steps_per_rev: u32) -> u32 {
        if planner_steps_per_rev < POSITION_UNITS_PER_REV {
            planner_steps_per_rev.next_power_of_two()
        } else {
            POSITION_UNITS_PER_REV
        }
    }

    fn mapper(planner_steps_per_rev: u32) -> ReversiblePulseMapper {
        ReversiblePulseMapper::new(PulseMapperConfig::new(planner_steps_per_rev).unwrap())
    }

    #[test]
    fn configuration_selects_normative_grid_for_representative_scales() {
        for planner_steps_per_rev in REPRESENTATIVE_STEPS {
            let config = PulseMapperConfig::new(planner_steps_per_rev).unwrap();
            let grid = expected_grid(planner_steps_per_rev);

            assert_eq!(config.planner_steps_per_rev(), planner_steps_per_rev);
            assert_eq!(config.tmc_grid(), grid);
            assert_eq!(config.step_width(), POSITION_UNITS_PER_REV / grid);
        }
    }

    #[test]
    fn configuration_covers_common_full_step_and_microstep_matrix() {
        for full_steps in [200, 400] {
            for microsteps in [1, 2, 4, 8, 16, 32, 64, 128, 256] {
                let planner_steps_per_rev = full_steps * microsteps;
                let config = PulseMapperConfig::new(planner_steps_per_rev).unwrap();

                assert_eq!(config.tmc_grid(), expected_grid(planner_steps_per_rev));
                assert_eq!(
                    config.tmc_grid() * config.step_width(),
                    POSITION_UNITS_PER_REV
                );
            }
        }
    }

    #[test]
    fn configuration_rejects_invalid_bounds_before_power_of_two_rounding() {
        assert_eq!(
            PulseMapperConfig::new(0),
            Err(PulseMapperConfigError::ZeroPlannerStepsPerRev)
        );
        assert_eq!(
            PulseMapperConfig::new(MAX_PLANNER_STEPS_PER_REV + 1),
            Err(PulseMapperConfigError::PlannerStepsPerRevTooLarge)
        );
        assert_eq!(
            PulseMapperConfig::new(u32::MAX),
            Err(PulseMapperConfigError::PlannerStepsPerRevTooLarge)
        );
    }

    #[test]
    fn exact_forward_and_reverse_revolutions_restore_phase() {
        for planner_steps_per_rev in REPRESENTATIVE_STEPS {
            let initial_phase = i64::from(planner_steps_per_rev / 2);
            let grid = i64::from(expected_grid(planner_steps_per_rev));

            let mut forward = mapper(planner_steps_per_rev);
            let forward_pulses: i64 = (0..planner_steps_per_rev)
                .map(|_| i64::from(forward.map_step(true)))
                .sum();
            assert_eq!(forward_pulses, grid);
            assert_eq!(forward.phase(), initial_phase);

            let mut reverse = mapper(planner_steps_per_rev);
            let reverse_pulses: i64 = (0..planner_steps_per_rev)
                .map(|_| i64::from(reverse.map_step(false)))
                .sum();
            assert_eq!(reverse_pulses, -grid);
            assert_eq!(reverse.phase(), initial_phase);
        }
    }

    #[test]
    fn multiple_revolutions_have_zero_accumulated_error() {
        for planner_steps_per_rev in REPRESENTATIVE_STEPS {
            let mut pulse_mapper = mapper(planner_steps_per_rev);
            let revolutions = 3;
            let pulses: i64 = (0..planner_steps_per_rev * revolutions)
                .map(|_| i64::from(pulse_mapper.map_step(true)))
                .sum();

            assert_eq!(
                pulses * i64::from(pulse_mapper.step_width()),
                i64::from(POSITION_UNITS_PER_REV * revolutions)
            );
            assert_eq!(pulse_mapper.phase(), i64::from(planner_steps_per_rev / 2));
        }
    }

    #[test]
    fn arbitrary_sequence_and_exact_inverse_restore_pulses_and_phase() {
        let directions = [
            true, true, false, true, false, false, true, true, true, false,
        ];
        let mut pulse_mapper = mapper(4_000);
        let initial_phase = pulse_mapper.phase();
        let mut pulses = 0_i64;

        for direction in directions {
            pulses += i64::from(pulse_mapper.map_step(direction));
        }
        for direction in directions.into_iter().rev() {
            pulses += i64::from(pulse_mapper.map_step(!direction));
        }

        assert_eq!(pulses, 0);
        assert_eq!(pulse_mapper.phase(), initial_phase);
    }

    #[test]
    fn scales_above_position_units_emit_only_zero_or_one_pulse() {
        let mut pulse_mapper = mapper(102_400);

        for direction in [true, false] {
            for _ in 0..200_000 {
                let pulses = pulse_mapper.map_step(direction);
                assert!(matches!(
                    (direction, pulses),
                    (true, 0 | 1) | (false, -1 | 0)
                ));
            }
        }
    }

    #[test]
    fn accessors_report_mapper_configuration_phase_and_error_bound() {
        let pulse_mapper = mapper(3_200);

        assert_eq!(pulse_mapper.planner_steps_per_rev(), 3_200);
        assert_eq!(pulse_mapper.tmc_grid(), 4_096);
        assert_eq!(pulse_mapper.step_width(), 16);
        assert_eq!(pulse_mapper.phase(), 1_600);
        assert_eq!(pulse_mapper.position_error_bound_half_units(), 16);
    }

    #[test]
    fn reconfigure_establishes_a_new_common_origin() {
        let mut pulse_mapper = mapper(3_200);
        pulse_mapper.map_step(true);

        pulse_mapper.reconfigure(PulseMapperConfig::new(4_000).unwrap());

        assert_eq!(pulse_mapper.planner_steps_per_rev(), 4_000);
        assert_eq!(pulse_mapper.phase(), 2_000);
    }

    #[test]
    fn deterministic_random_walk_matches_rational_reference_and_its_inverse() {
        for planner_steps_per_rev in REPRESENTATIVE_STEPS {
            let mut pulse_mapper = mapper(planner_steps_per_rev);
            let planner_scale = i64::from(planner_steps_per_rev);
            let grid = i64::from(pulse_mapper.tmc_grid());
            let initial_phase = pulse_mapper.phase();
            let mut reference_phase = initial_phase;
            let mut cumulative_pulses = 0_i64;
            let mut reference_pulses = 0_i64;
            let mut directions = [false; 257];
            let mut state = 0x6d2b_79f5_u32;

            for direction in &mut directions {
                state ^= state << 13;
                state ^= state >> 17;
                state ^= state << 5;
                *direction = state & 1 != 0;
                let signed_direction = if *direction { 1_i64 } else { -1_i64 };
                let total = reference_phase + signed_direction * grid;
                reference_pulses += total.div_euclid(planner_scale);
                reference_phase = total.rem_euclid(planner_scale);
                let pulses = pulse_mapper.map_step(*direction);
                cumulative_pulses += i64::from(pulses);

                assert_eq!(cumulative_pulses, reference_pulses);
                assert_eq!(pulse_mapper.phase(), reference_phase);
                assert!((-2..=2).contains(&pulses));
            }

            for direction in directions.into_iter().rev() {
                let inverse = !direction;
                let signed_direction = if inverse { 1_i64 } else { -1_i64 };
                let total = reference_phase + signed_direction * grid;
                reference_pulses += total.div_euclid(planner_scale);
                reference_phase = total.rem_euclid(planner_scale);
                cumulative_pulses += i64::from(pulse_mapper.map_step(inverse));

                assert_eq!(cumulative_pulses, reference_pulses);
                assert_eq!(pulse_mapper.phase(), reference_phase);
            }

            assert_eq!(cumulative_pulses, 0);
            assert_eq!(pulse_mapper.phase(), initial_phase);
        }
    }
}
