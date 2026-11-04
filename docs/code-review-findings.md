# foci-klipper code review: simplification, performance, dead code, modularization

Reviewed at `4ad93a9` (v0.4.1). All ~3,700 lines of `src/` were read; `cargo test`
passed (135 tests) before any change. No code was changed as part of this review.

Items marked **✅ verified** were checked with a throwaway prototype (details in
[Verification](#verification)); the rest come from reading the code.

> **Scope caveat.** The firmware crate that consumes this library was not
> available. Public items listed as "unused" are unused *inside this crate*;
> confirm against the firmware before deleting any of them.

## Summary

| # | Finding | Category | Priority | API change |
|---|---------|----------|----------|------------|
| 1 | `map_step` does a software 64-bit divide per step in the ISR | Performance | High | No |
| 2 | `EndstopRuntime::oversample` can underflow `trigger_count` | Correctness | High | No |
| 3 | Step-interval cast misbehaves above `i32::MAX` | Correctness | Medium | No |
| 4 | `MAX_SCHEDULER_EVENTS` is hardcoded and contradicts its comment | Correctness | Medium | No |
| 5 | Redundant state and branches (`report_pending`, `old_head`, …) | Simplification | Medium | Mostly no |
| 6 | Clock wraparound arithmetic is scattered | Modularization | Medium | No |
| 7 | `HardwareStepper` mixes ISR and P2 state | Modularization | Medium | Yes |
| 8 | Stepper / digital-out OID registration is duplicated | Modularization | Medium | Yes |
| 9 | Items unused in this crate | Dead code | Low | Yes |
| 10 | Test boilerplate, CI and tooling gaps | Tooling | Low | No |

## 1. Performance

### 1.1 ✅ `ReversiblePulseMapper::map_step` calls `__aeabi_ldivmod` on every step

`src/pulse_mapper.rs:106`

`map_step` is called by `HardwareStepper::step_event`, so it runs once per logical
step inside the P3 timer ISR. It uses `i64::div_euclid` / `rem_euclid`, which on
the Cortex-M4F target compiles to a call to `__aeabi_ldivmod` (the software
64-bit divide routine, typically ~100+ cycles) plus a divide-by-zero panic check.

The division is unnecessary. The invariants already enforced by
`PulseMapperConfig::new` are:

- `0 <= phase < P`, with `P <= 2^24`
- `tmc_grid < 2P` (it is `next_power_of_two(P)` below 65,536, otherwise 65,536)

So one step needs at most two compare-and-subtract corrections in 32-bit
arithmetic:

```rust
pub fn map_step(&mut self, forward: bool) -> i8 {
    let p = self.config.planner_steps_per_rev;
    let grid = self.config.tmc_grid;
    let mut pulses = 0i8;
    if forward {
        let mut t = self.phase + grid;           // < 3P, fits u32
        while t >= p { t -= p; pulses += 1; }
        self.phase = t;
    } else {
        let mut t = self.phase as i32 - grid as i32; // > -2P, fits i32
        while t < 0 { t += p as i32; pulses -= 1; }
        self.phase = t as u32;
    }
    pulses
}
```

`phase` can then be stored as `u32`; `phase()` can keep returning `i64` via
`i64::from`, so there is no API change.

### 1.2 Minor

- `HardwareStepper::check_deadline` (`src/stepper.rs:340`) computes
  `clock_freq / 1000` on every call. Cheap on the M4's hardware divider, but
  taking a precomputed `max_late_ticks` is both simpler and faster.
- `TriggerScheduler::try_schedule` and `is_full` (`src/scheduler.rs:168`, `:264`)
  scan all slots to find a free one. A 16-bit free-slot bitmask
  (`trailing_zeros()`) would make both O(1). Low priority at 13 slots.

## 2. Correctness hazards

### 2.1 ✅ `EndstopRuntime::oversample` can underflow `trigger_count`

`src/endstop.rs:150` (`self.trigger_count -= 1`)

After a trigger, `phase` stays `Oversampling` and `trigger_count` is 0. A fresh,
never-armed runtime is in the same state, and unlike `scan`, `oversample` does not
check `armed`. Any further `oversample()` call:

- **debug:** panics with `attempt to subtract with overflow`;
- **release:** wraps to 255, so the endstop silently needs 255 more matching reads
  before it triggers again.

Klipper stops the timer after a trigger, so this only bites if the ISR ever
dispatches on `phase()` again. Suggested fix: return `Inactive` when not armed,
and either reset `phase` on trigger or use `checked_sub`.

### 2.2 Step-interval cast misbehaves above `i32::MAX`

`src/stepper.rs:316`

```rust
self.current_interval = (self.current_interval as i32)
    .saturating_add(self.current_add as i32)
    .max(1) as u32;
```

An interval above `i32::MAX` becomes negative and is then clamped to 1 tick.
Shorter and correct:

```rust
self.current_interval = self
    .current_interval
    .saturating_add_signed(i32::from(self.current_add))
    .max(1);
```

### 2.3 `MAX_SCHEDULER_EVENTS` is hardcoded

`src/scheduler.rs:8`

The comment says the value is `NUM_STEPPERS + NUM_ENDSTOPS + 2 * NUM_TRSYNC`,
but with the constants in `state.rs` that is 4 + 4 + 8 = 16, not 13. The 13
assumes one physical stepper. If a board ever drives more, scheduling fails at
runtime with `ScheduleResult::Full` instead of at compile time. Derive it from a
`NUM_PHYSICAL_STEPPERS` constant, or add a `const` assertion.

## 3. Simplifications

| Location | Finding |
|----------|---------|
| `src/trigger.rs:70` | `report_pending` always equals `report_snapshot.is_some()` (checked at every write site: `new`, `arm`, `trigger`, `set_liveness_report`, `take_report`, `clear_pending_report`). Drop the bool; `take_report()` becomes `self.report_snapshot.take()`. |
| `src/trigger.rs:96` | `arm()` rewrites every field by hand. Use `*self = Self { can_trigger: true, expire_reason, report_ticks, report_clock, ..Self::new() }`. |
| `src/scheduler.rs:164` | `old_head != self.head` is always `false`: the branches that change `head` have already returned. Return `false`. |
| `src/scheduler.rs:242` | `requeue_slot` builds the same `SchedulerEvent` twice. `index >= MAX_SCHEDULER_EVENTS` cannot happen, since `ScheduledSlot` is only constructed inside the crate. |
| `src/scheduler.rs:229` | `pop_due_slot` uses `?` on the head node while every other method uses `expect("… exists")` for the same invariant. Pick one. |
| `src/stepper.rs:242` | Both match arms in `configure_motion_scale` produce `Some(ReversiblePulseMapper::new(config))`. Assign directly; `ReversiblePulseMapper::reconfigure` is then test-only. |
| `src/pulse_mapper.rs` | `ReversiblePulseMapper` re-exposes four `PulseMapperConfig` accessors one-for-one. Expose `config()` instead. `position_error_bound_half_units()` just returns `step_width`. |
| `src/endstop.rs:106` | `start_homing` copies eight `HomingConfig` fields one by one into the runtime. Store `cfg: HomingConfig` directly. `query_state()` duplicates `is_armed()` + `nextwake()`. |
| `src/commands/lifecycle.rs:13` | `get_config` returns an unnamed `(bool, u32, bool, u16)`. Use a named `ConfigReply` struct. |
| `src/commands/lifecycle.rs:50` | `config_reset`'s doc says the caller must check for shutdown first. Enforce it by returning `Result<(), ShutdownRequired>`, as `clear_shutdown` does. |
| `src/state.rs:64` | `State::reset` can be `*self = Self::new()`, matching the other runtime types. |

## 4. Unused or vestigial code

Unused inside this crate; **confirm against the firmware before removing**:

- `trigger::StopSignal`
- `state::NUM_TRSYNC`, `state::NUM_ENDSTOPS` (only referenced by a comment in `scheduler.rs`)
- `TriggerResult::UnknownOid` (never constructed here)
- `TriggerScheduler::pop_due`, `TriggerScheduler::is_full` (test-only callers)
- `commands::lifecycle::allocate_oids` and all of `commands/spi.rs` (no-ops). If the
  firmware wrappers are already empty, these can go.
- `McuState::Shutdown { reason }`: `enter_shutdown` always writes 0 and nothing
  reads it. Either thread a real reason through `enter_shutdown(state, reason)` or
  drop the field.

Other leftovers:

- `reset_stepper_clocks` (`src/commands/stepper.rs:89`) calls `reset_step_clock`
  on the P2 stepper, which only sets `next_step_clock` and `was_active`. The P2
  side only ever reads `next_dir`, so the call appears to do nothing (unless the
  firmware reads `p2_stepper.next_step_clock()`).
- The `stepper_stop_runtime_clearing_sequence` test (`src/trigger.rs:316`) only
  assigns struct fields and asserts them back; it tests no logic.
- The `State` doc comment (`src/state.rs:13-20`) describes what "has moved"
  rather than what the type is.

## 5. Modularization

### 5.1 Split `HardwareStepper` by execution context

`HardwareStepper`'s own doc comment (`src/stepper.rs:109-118`) says it holds ISR
state *and* separate P2 state (`next_dir`), and the firmware keeps two instances
of it. Make that split visible in the types:

- `StepDirCommand { next_dir }` with `set_next_step_dir` and `prepare_move` (P2).
- `HardwareStepper` as the ISR engine only.

This also removes the no-op P2 reset described in section 4.

### 5.2 Add an `OidRegistry<const N>`

`config_stepper` / `config_stepper_channel` (`src/commands/stepper.rs:11`, `:27`)
and `config_digital_out` / `config_enable_channel`
(`src/commands/digital_out.rs:10`, `:26`) are line-for-line copies over
different `State` fields, and `State::stepper_channel` / `enable_channel` repeat
the pattern. Each copy also:

- repeats the duplicate-OID check (outer and inner function);
- repeats the capacity check that `LinearMap::insert` already performs;
- has an unreachable `UnknownOid` lookup straight after the insert.

An `OidRegistry<N>` holding the `LinearMap<u8, usize, N>` and `[Option<u8>; N]`
channel array would let `State` become `steppers: OidRegistry`,
`enables: OidRegistry`. `queue_digital_out` and `update_digital_out` then share
one lookup helper.

### 5.3 A `clock` module

Signed-wraparound clock comparisons are implemented three times:

- `is_due`, `is_before`, `is_before_or_equal` in `src/scheduler.rs:72-82`;
- `timer_arm_clock` in `src/scheduler.rs:63`;
- `check_deadline` in `src/stepper.rs:340`, as `(now as i32).wrapping_sub(scheduled as i32)`.

One `clock` module (`clock_diff`, `is_before`, `is_due`) would give this logic a
single home and a single set of wraparound tests.

### 5.4 Smaller items

- `cc2_active` names a specific hardware timer channel (TIM CC2) inside a generic
  scheduler; `hw_compare_armed` or similar fits better.
- `SchedulerEventKind` indices could be `u8` instead of `usize`, shrinking each
  scheduler node from about 16 to 12 bytes on the target.
- `pulse_mapper.rs` is the only module without a `//!` header; its doc line lives
  in `lib.rs` instead.

## 6. Tests and tooling

- **Test boilerplate:** `scheduler.rs` tests are ~500 lines, mostly repeated
  `SchedulerEvent { … }` literals and inline
  `assert!(matches!(s.try_schedule(…), …))`. The existing `insert` helper is used
  inconsistently; using it everywhere plus an `ev(kind, clock)` helper would cut
  about half. Endstop tests would benefit from `Default` on `HomingConfig`.
- **Literals:** `step_queue.rs` tests hardcode 1024 / 1023 instead of
  `STEP_QUEUE_DEPTH` / `STEP_QUEUE_CAPACITY`.
- **CI lint coverage:** `cargo clippy -- -D warnings` omits `--all-targets`, so
  test code is never linted.
- **CI target coverage:** CI never builds for `thumbv7em-none-eabihf`, so 32-bit
  and `usize`-width assumptions are unchecked.
- **Toolchain:** CI pins Rust 1.98.1, but there is no `rust-toolchain.toml`, so
  local builds can drift.
- **Docs lint:** `#![warn(missing_docs)]` would flag the many undocumented
  `pub fn`s in `scheduler.rs` and `trigger.rs`.
- **Pedantic clippy:** mostly `must_use` and doc-backtick noise; the cast warnings
  it raises are the ones covered in sections 1 and 2.
- **`.gitignore`:** none exists, so `target/` shows as untracked after a build.
  Also decide whether to commit `Cargo.lock`, which would pin the exact `ankyra`
  git revision.

## Suggested order

1. Division-free pulse mapper (1.1) and endstop underflow fix (2.1): high value,
   small, already prototyped, no API change.
2. Interval cast (2.2), scheduler constant (2.3), and the simplifications in
   section 3 that do not change public signatures.
3. `OidRegistry` (5.2) and the `HardwareStepper` split (5.1): API changes that
   need a matching firmware update.
4. Dead-code removal (section 4), once confirmed against the firmware.
5. Test cleanup and CI hardening (section 6).

## Verification

Prototypes ran in a throwaway git worktree; nothing was committed from them.

**1.1 Pulse mapper**

- Compiled the current and division-free `map_step` for `thumbv7em-none-eabihf`
  at `opt-level = 3`. The current version calls `__aeabi_ldivmod` and the
  divide-by-zero panic handler; the new version makes no calls.
- Replaced `map_step` in the crate and ran the existing `pulse_mapper` tests:
  all passed.
- Added a comparison test against the original `div_euclid` / `rem_euclid`
  formula: 74 scales (including 1, 2, 3, 65,535, 65,537, 2^24, the
  representative scales, and random values up to 2^24), 20,000 random-direction
  steps each, checking pulses and phase after every step. All matched.

**2.1 Endstop underflow**

- Calling `oversample()` after a completed trigger, and on a never-armed runtime,
  both panicked in a debug build at `src/endstop.rs:150` with
  `attempt to subtract with overflow`.
- In a release build the post-trigger call did not panic. The count wraps to 255,
  so per the code the call returns `Reschedule` instead of triggering.
