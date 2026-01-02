//! Clock domain utilities for Klipper MCU protocol.
//!
//! Klipper uses 32-bit unsigned clock values that wrap. All comparisons
//! use signed arithmetic to handle wraparound correctly:
//! `(a as i32).wrapping_sub(b as i32) > 0` means a is after b.

/// Check if clock `a` is before clock `b` (handles 32-bit wraparound).
pub fn clock_is_before(a: u32, b: u32) -> bool {
    (a as i32).wrapping_sub(b as i32) < 0
}

/// Convert a duration in microseconds to clock ticks.
pub const fn clock_from_us(us: u32, clock_freq: u32) -> u32 {
    (us as u64 * clock_freq as u64 / 1_000_000) as u32
}

/// Convert clock ticks to microseconds.
pub const fn clock_to_us(ticks: u32, clock_freq: u32) -> u32 {
    (ticks as u64 * 1_000_000 / clock_freq as u64) as u32
}

/// 32-bit to 64-bit clock extension.
///
/// Given a known recent 64-bit clock value and a new 32-bit sample,
/// extend the sample to 64-bit by assuming it is close to the known value.
pub fn clock32_to_clock64(last_clock64: u64, clock32: u32) -> u64 {
    let high = last_clock64 & !0xFFFF_FFFF;
    let candidate = high | clock32 as u64;
    if candidate > last_clock64.wrapping_add(0x8000_0000) {
        candidate.wrapping_sub(0x1_0000_0000)
    } else if candidate < last_clock64.wrapping_sub(0x8000_0000) {
        candidate.wrapping_add(0x1_0000_0000)
    } else {
        candidate
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn clock_is_before_normal() {
        assert!(clock_is_before(100, 200));
        assert!(!clock_is_before(200, 100));
        assert!(!clock_is_before(100, 100));
    }

    #[test]
    fn clock_is_before_wraparound() {
        assert!(clock_is_before(0xFFFF_FFF0, 0x0000_0010));
        assert!(!clock_is_before(0x0000_0010, 0xFFFF_FFF0));
    }

    #[test]
    fn clock_from_us_168mhz() {
        assert_eq!(clock_from_us(1, 168_000_000), 168);
        assert_eq!(clock_from_us(1000, 168_000_000), 168_000);
    }

    #[test]
    fn clock_from_us_520mhz() {
        assert_eq!(clock_from_us(1, 520_000_000), 520);
        assert_eq!(clock_from_us(1000, 520_000_000), 520_000);
    }

    #[test]
    fn clock_to_us_round_trip() {
        let freq = 168_000_000;
        let ticks = clock_from_us(1000, freq);
        let us = clock_to_us(ticks, freq);
        assert_eq!(us, 1000);
    }

    #[test]
    fn clock32_to_64_no_wrap() {
        let last = 0x0000_0001_0000_0000u64;
        let sample = 0x0000_1000u32;
        let result = clock32_to_clock64(last, sample);
        assert_eq!(result, 0x0000_0001_0000_1000);
    }

    #[test]
    fn clock32_to_64_wrap_forward() {
        let last = 0x0000_0000_FFFF_FF00u64;
        let sample = 0x0000_0100u32;
        let result = clock32_to_clock64(last, sample);
        assert_eq!(result, 0x0000_0001_0000_0100);
    }

    #[test]
    fn clock32_to_64_wrap_backward() {
        let last = 0x0000_0001_0000_0100u64;
        let sample = 0xFFFF_FF00u32;
        let result = clock32_to_clock64(last, sample);
        assert_eq!(result, 0x0000_0000_FFFF_FF00);
    }
}
