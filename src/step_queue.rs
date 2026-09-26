//! Explicit producer/consumer ownership for hardware step/dir move queues.
//!
//! Replaces deriving a fresh `&'static mut Queue` from a `static mut`
//! array on every access, which is unsound under Rust's aliasing model
//! regardless of the RTIC-priority access discipline it relied on. This
//! type never depends on `cortex_m` and stays fully host-testable — the
//! interrupt masking that makes `take_consumer`/`give_consumer`/
//! `borrow_consumer_mut` sound is supplied by a small board-local wrapper,
//! not by this type.

use core::cell::UnsafeCell;

use heapless::spsc::Queue;
// Re-exported: board crates reference these as `foci_klipper::step_queue::{Producer, Consumer}`.
pub use heapless::spsc::{Consumer, Producer};

use crate::stepper::StepMove;

/// Depth of each board's hardware step/dir move queue.
pub const STEP_QUEUE_DEPTH: usize = 1024;

/// Owns one channel's step/dir move queue and its detachable `Consumer`.
pub struct StepQueueChannel<const N: usize> {
    queue: UnsafeCell<Queue<StepMove, N>>,
    consumer: UnsafeCell<Option<Consumer<'static, StepMove>>>,
}

// SAFETY: every method is either a lock-free atomic-only read (`len`,
// `is_empty`) or requires the caller to already hold exclusive access
// (documented per method) before touching `consumer`.
unsafe impl<const N: usize> Sync for StepQueueChannel<N> {}

impl<const N: usize> Default for StepQueueChannel<N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const N: usize> StepQueueChannel<N> {
    /// Create an empty, unsplit channel. Call `split_once` exactly once,
    /// from RTIC `init`, to obtain the `Producer` half before any other
    /// method is used.
    pub const fn new() -> Self {
        Self {
            queue: UnsafeCell::new(Queue::new()),
            consumer: UnsafeCell::new(None),
        }
    }

    /// Split the backing queue once, at RTIC `init`, and return the
    /// `Producer` half. The `Consumer` half is retained internally.
    ///
    /// # Safety
    ///
    /// Must be called at most once per channel, before any other method on
    /// this type. `self` must have `'static` storage duration (true for a
    /// board's `StepQueueChannel`, which is always a `static`).
    pub unsafe fn split_once(&'static self) -> Producer<'static, StepMove> {
        // SAFETY: caller guarantees this runs exactly once, before any
        // other access, on a `'static` object -- so `split()`'s
        // `'static`-lifetime borrow can never alias a later caller,
        // because everything else in this type accesses `queue` only
        // through the `Producer`/`Consumer` halves this call produces.
        let queue = unsafe { &mut *self.queue.get() };
        let (producer, consumer) = queue.split();
        // SAFETY: `consumer` has never been written before this call.
        unsafe {
            *self.consumer.get() = Some(consumer);
        }
        producer
    }

    /// Lock-free length read. Sound from any context (P1/P2/P3, masked or
    /// not, consumer attached or detached): `heapless::spsc`'s length
    /// computation is atomic-load-only by design.
    pub fn len(&self) -> usize {
        // SAFETY: read-only; never constructs `&mut Queue`.
        unsafe { &*self.queue.get() }.len()
    }

    /// Lock-free emptiness read. See `len`.
    pub fn is_empty(&self) -> bool {
        // SAFETY: read-only; never constructs `&mut Queue`.
        unsafe { &*self.queue.get() }.is_empty()
    }

    /// Dequeue directly from the ISR-owned `Consumer`, with no locking.
    ///
    /// Returns `None` while the consumer is detached: the detached holder owns
    /// draining the queue, so the ISR must see it as empty and go idle.
    ///
    /// # Safety
    ///
    /// Callable only from the P3 ISR context that owns this channel's timer
    /// dispatch.
    pub unsafe fn isr_dequeue(&self) -> Option<StepMove> {
        // SAFETY: caller guarantees exclusive P3 access.
        unsafe { (*self.consumer.get()).as_mut() }?.dequeue()
    }

    /// Detach the `Consumer`, for a caller about to drain it outside any
    /// masked region. Returns `None` if already detached.
    ///
    /// # Safety
    ///
    /// Caller must already hold an interrupt-masked or otherwise
    /// exclusive-access critical section.
    pub unsafe fn take_consumer(&self) -> Option<Consumer<'static, StepMove>> {
        // SAFETY: see doc comment.
        unsafe { (*self.consumer.get()).take() }
    }

    /// Reattach a previously detached `Consumer`.
    ///
    /// # Safety
    ///
    /// Caller must already hold an interrupt-masked or otherwise
    /// exclusive-access critical section, and `consumer` must be the exact
    /// value most recently returned by `take_consumer()` on this same
    /// channel -- `Consumer` carries no channel identity of its own, so
    /// giving a different channel's consumer here would let this channel's
    /// ISR dequeue from the wrong queue.
    ///
    /// # Panics
    ///
    /// Panics (in all build profiles) if a consumer is already attached --
    /// that would mean two live consumers for the same channel.
    pub unsafe fn give_consumer(&self, consumer: Consumer<'static, StepMove>) {
        // SAFETY: see doc comment.
        let slot = unsafe { &mut *self.consumer.get() };
        assert!(
            slot.is_none(),
            "give_consumer called with a consumer already attached"
        );
        *slot = Some(consumer);
    }

    /// Borrow the `Consumer` in place, without moving it out. Returns
    /// `None` (without calling `f`) if currently detached.
    ///
    /// # Safety
    ///
    /// Caller must already hold an interrupt-masked or otherwise
    /// exclusive-access critical section.
    pub unsafe fn borrow_consumer_mut<R>(
        &self,
        f: impl FnOnce(&mut Consumer<'static, StepMove>) -> R,
    ) -> Option<R> {
        // SAFETY: see doc comment.
        let slot = unsafe { &mut *self.consumer.get() };
        slot.as_mut().map(f)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn split_once_yields_a_working_producer_and_consumer() {
        static CHANNEL: StepQueueChannel<4> = StepQueueChannel::new();
        // SAFETY: single call on a `'static` channel, first thing this test does.
        let mut producer = unsafe { CHANNEL.split_once() };
        producer.enqueue(StepMove::new(100, 1, 0)).unwrap();
        // SAFETY: host test is single-threaded; nothing else touches CHANNEL.
        let dequeued = unsafe { CHANNEL.isr_dequeue() };
        assert_eq!(dequeued, Some(StepMove::new(100, 1, 0)));
    }

    #[test]
    fn len_and_is_empty_are_correct_before_and_after_enqueue() {
        static CHANNEL: StepQueueChannel<4> = StepQueueChannel::new();
        let mut producer = unsafe { CHANNEL.split_once() };
        assert!(CHANNEL.is_empty());
        assert_eq!(CHANNEL.len(), 0);
        producer.enqueue(StepMove::new(50, 1, 0)).unwrap();
        assert!(!CHANNEL.is_empty());
        assert_eq!(CHANNEL.len(), 1);
    }

    #[test]
    fn len_and_is_empty_are_correct_while_consumer_is_detached() {
        static CHANNEL: StepQueueChannel<4> = StepQueueChannel::new();
        let mut producer = unsafe { CHANNEL.split_once() };
        producer.enqueue(StepMove::new(50, 1, 0)).unwrap();
        let consumer = unsafe { CHANNEL.take_consumer() };
        assert!(consumer.is_some());
        // Length is still readable with no consumer attached.
        assert_eq!(CHANNEL.len(), 1);
        unsafe { CHANNEL.give_consumer(consumer.unwrap()) };
        assert_eq!(CHANNEL.len(), 1);
    }

    #[test]
    fn take_consumer_returns_none_on_the_second_call() {
        static CHANNEL: StepQueueChannel<4> = StepQueueChannel::new();
        let _producer = unsafe { CHANNEL.split_once() };
        assert!(unsafe { CHANNEL.take_consumer() }.is_some());
        assert!(unsafe { CHANNEL.take_consumer() }.is_none());
    }

    #[test]
    #[should_panic(expected = "give_consumer called with a consumer already attached")]
    fn give_consumer_panics_when_the_slot_is_already_occupied() {
        static CHANNEL: StepQueueChannel<4> = StepQueueChannel::new();
        let _producer = unsafe { CHANNEL.split_once() };
        // CHANNEL's own consumer is still attached (never taken) -- the slot
        // is genuinely occupied.

        // Mint an unrelated Consumer<'static, StepMove> from a throwaway
        // queue, just to have a second, distinct Consumer value to attempt
        // to give -- `Consumer` isn't `Clone`, so there is no other way to
        // get a second value to pass while the first is still attached.
        static OTHER_QUEUE: StepQueueChannel<4> = StepQueueChannel::new();
        let _other_producer = unsafe { OTHER_QUEUE.split_once() };
        let other_consumer = unsafe { OTHER_QUEUE.take_consumer() }.unwrap();

        // CHANNEL's slot is still occupied by its own consumer -- this must panic.
        unsafe { CHANNEL.give_consumer(other_consumer) };
    }

    #[test]
    fn borrow_consumer_mut_returns_none_while_detached_and_some_while_attached() {
        static CHANNEL: StepQueueChannel<4> = StepQueueChannel::new();
        let mut producer = unsafe { CHANNEL.split_once() };
        producer.enqueue(StepMove::new(50, 1, 0)).unwrap();

        let result = unsafe { CHANNEL.borrow_consumer_mut(|c| c.dequeue()) };
        assert_eq!(result, Some(Some(StepMove::new(50, 1, 0))));

        let consumer = unsafe { CHANNEL.take_consumer() }.unwrap();
        let none_result = unsafe { CHANNEL.borrow_consumer_mut(|c| c.dequeue()) };
        assert_eq!(none_result, None);
        unsafe { CHANNEL.give_consumer(consumer) };
    }

    #[test]
    fn isr_dequeue_reports_no_move_while_consumer_is_detached() {
        static CHANNEL: StepQueueChannel<4> = StepQueueChannel::new();
        let mut producer = unsafe { CHANNEL.split_once() };
        let consumer = unsafe { CHANNEL.take_consumer() }.unwrap();
        producer.enqueue(StepMove::new(50, 1, 0)).unwrap();

        let dequeued = unsafe { CHANNEL.isr_dequeue() };

        assert_eq!(dequeued, None);
        assert_eq!(CHANNEL.len(), 1);
        unsafe { CHANNEL.give_consumer(consumer) };
    }

    #[test]
    fn detach_and_drain_a_queue_filled_to_its_full_usable_capacity() {
        static CHANNEL: StepQueueChannel<1024> = StepQueueChannel::new();
        let mut producer = unsafe { CHANNEL.split_once() };

        // count=2 (not 1) so the step-total assertion below is distinct
        // from the segment-count assertion -- summing 1s would not catch
        // a regression that dropped the count accumulation entirely.
        let mut enqueued = 0usize;
        while producer.enqueue(StepMove::new(10, 2, 0)).is_ok() {
            enqueued += 1;
        }
        // heapless::spsc reserves one slot for full/empty disambiguation,
        // so usable capacity is N - 1.
        assert_eq!(enqueued, 1023);
        assert_eq!(CHANNEL.len(), 1023);

        let mut consumer = unsafe { CHANNEL.take_consumer() }.expect("consumer attached");
        let mut drained_segments = 0usize;
        let mut drained_steps = 0u32;
        while let Some(m) = consumer.dequeue() {
            drained_segments += 1;
            drained_steps += u32::from(m.count);
        }
        assert_eq!(drained_segments, 1023);
        assert_eq!(drained_steps, 1023 * 2);
        assert!(CHANNEL.is_empty());

        unsafe { CHANNEL.give_consumer(consumer) };

        // Prove the channel is still fully usable after a full-capacity
        // detach/reattach cycle, not just that the cycle itself completes.
        producer.enqueue(StepMove::new(20, 1, 0)).unwrap();
        let redequeued = unsafe { CHANNEL.isr_dequeue() };
        assert_eq!(redequeued, Some(StepMove::new(20, 1, 0)));
    }
}
