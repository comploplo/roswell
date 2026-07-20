//! Machine-checked pure cores for roswell's real-time hot paths.
//!
//! Each function here is the **production** implementation used by `roswell`
//! (`dynamic::round_up`, the CDR padding formula) and `roswell-ros2-compat`
//! (`node`'s timer scheduling, `tunnel`'s bounded-queue policy) — not a
//! parallel copy. They are isolated in this dependency-free crate purely so
//! Creusot can translate them in full: the surrounding crates pull in nom,
//! `unsafe` pointer code, RustDDS, and recursive IR types that fall outside
//! Creusot's supported subset.
//!
//! The `#[requires]`/`#[ensures]`/`#[invariant]`/`#[variant]` contracts are
//! active only under `--cfg creusot` (set by `cargo creusot --features verify`)
//! and erased in every normal build, so this crate compiles as plain Rust with
//! no dependencies. Discharge the proofs with `scripts/verify-rt.sh`; the
//! properties and their limits are catalogued in `docs/RT.md`.

#![cfg_attr(creusot, allow(unexpected_cfgs))]

#[cfg(creusot)]
extern crate creusot_std;
#[cfg(creusot)]
use creusot_std::prelude::*;

/// Padding bytes to advance an offset `off` (measured from the CDR alignment
/// origin) up to the next multiple of `a`. `a` is a CDR primitive alignment
/// (1, 2, 4, or 8) — a nonzero power of two — at every call site.
///
/// Proven: for `a > 0` this never panics (no division-by-zero; the subtraction
/// `a - off % a` never underflows because `off % a < a`), the result is
/// strictly less than `a`, and `off + result` is an exact multiple of `a`.
#[cfg_attr(creusot, requires(a@ > 0))]
#[cfg_attr(creusot, ensures(result@ < a@))]
#[cfg_attr(creusot, ensures((off@ + result@) % a@ == 0))]
#[must_use]
pub fn pad_to(off: usize, a: usize) -> usize {
    let r = off % a;
    let pad = if r == 0 { 0 } else { a - r };
    // Make the aligned position an explicit multiple of `a` so the prover can
    // discharge `(off + pad) % a == 0` from the Euclidean identity
    // `off == a*(off/a) + off%a` rather than reasoning through nested `%`.
    #[cfg(creusot)]
    proof_assert! {
        off@ + pad@ == a@ * (if r@ == 0 { off@ / a@ } else { off@ / a@ + 1 })
    };
    pad
}

/// Round `off` up to the next multiple of `align` (`>= off`). For a power-of-two
/// `align` this equals the classic `(off + align - 1) & !(align - 1)` mask; the
/// arithmetic form here is correct for every `align > 0` and is the shape
/// Creusot can reason about.
///
/// Proven: for `align > 0` and enough headroom (`off + align <= usize::MAX`)
/// the computation never overflows, `off <= result < off + align`, and
/// `result` is an exact multiple of `align`.
#[cfg_attr(creusot, requires(align@ > 0))]
#[cfg_attr(creusot, requires(off@ + align@ <= usize::MAX@))]
#[cfg_attr(creusot, ensures(result@ >= off@))]
#[cfg_attr(creusot, ensures(result@ < off@ + align@))]
#[cfg_attr(creusot, ensures(result@ % align@ == 0))]
#[must_use]
pub fn round_up(off: usize, align: usize) -> usize {
    off + pad_to(off, align)
}

/// Advance a timer's `next_fire` deadline (nanoseconds) past `now`, catching up
/// in whole `period` steps — the scheduling core of `node`'s `Timer::fire`.
///
/// Proven: for `period >= 1` and `now < i64::MAX` the catch-up loop always
/// terminates (variant `now - next_fire`), the returned deadline is `>= next_fire`
/// (monotonic — a timer never rewinds), and it is strictly `> now` unless the
/// `i64` nanosecond axis saturates. `now < i64::MAX` is the honest termination
/// precondition: a `now` pinned at `i64::MAX` (only reachable via the
/// `unwrap_or(i64::MAX)` clock fallback ~292 years past the epoch) would spin.
#[cfg_attr(creusot, requires(period@ >= 1))]
#[cfg_attr(creusot, requires(now@ < i64::MAX@))]
#[cfg_attr(creusot, ensures(result@ >= next_fire@))]
#[cfg_attr(creusot, ensures(result@ > now@))]
#[must_use]
pub fn next_fire_after(next_fire: i64, period: i64, now: i64) -> i64 {
    let mut cur = next_fire;
    #[cfg_attr(creusot, invariant(cur@ >= next_fire@))]
    #[cfg_attr(creusot, variant(now@ - cur@))]
    while cur <= now {
        cur = cur.saturating_add(period);
    }
    cur
}

/// How to admit one new frame into a bounded priority lane that currently holds
/// `len` frames for a channel configured with room for `max_pending`, under the
/// channel's drop policy (`drop_oldest` = evict the front; otherwise reject the
/// newcomer). The pure decision core of `tunnel`'s `OutboundQueue::enqueue`.
pub struct EnqueuePlan {
    /// Evict the oldest queued frame before pushing (drop-oldest, lane full).
    pub drop_front: bool,
    /// Push the new frame (false = reject it, drop-newest with a full lane).
    pub push: bool,
}

/// Decide admission for one frame.
///
/// Proven (`max_pending >= 1`): the resulting length
/// `len - drop_front + push` never rises above `max_pending` unless it was
/// already above it (a lane shared by channels with smaller `max_pending`), and
/// a *net* growth (`push` without a `drop_front`) happens only when the lane had
/// strict room (`len < max_pending`). So an enqueue can never push a channel's
/// own backlog past its configured bound, and never worsens an over-full lane.
/// `drop_front` is never requested on an empty lane.
#[cfg_attr(creusot, requires(max_pending@ >= 1))]
#[cfg_attr(creusot, ensures(result.drop_front ==> len@ >= 1))]
#[cfg_attr(creusot, ensures(
    len@ - (if result.drop_front { 1 } else { 0 }) + (if result.push { 1 } else { 0 })
        <= if len@ >= max_pending@ { len@ } else { max_pending@ }))]
#[cfg_attr(creusot, ensures(
    len@ - (if result.drop_front { 1 } else { 0 }) + (if result.push { 1 } else { 0 }) > len@
        ==> len@ < max_pending@))]
#[must_use]
pub fn plan_enqueue(len: usize, max_pending: usize, drop_oldest: bool) -> EnqueuePlan {
    if len >= max_pending {
        if drop_oldest {
            EnqueuePlan {
                drop_front: true,
                push: true,
            }
        } else {
            EnqueuePlan {
                drop_front: false,
                push: false,
            }
        }
    } else {
        EnqueuePlan {
            drop_front: false,
            push: true,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pad_to_aligns_and_bounds() {
        for a in [1usize, 2, 4, 8] {
            for off in 0usize..64 {
                let p = pad_to(off, a);
                assert!(p < a);
                assert_eq!((off + p) % a, 0);
                assert_eq!(round_up(off, a), off + p);
            }
        }
    }

    #[test]
    fn round_up_matches_bitwise_for_pow2() {
        for align in [1usize, 2, 4, 8, 16] {
            for off in 0usize..128 {
                assert_eq!(round_up(off, align), (off + align - 1) & !(align - 1));
            }
        }
    }

    #[test]
    fn next_fire_catches_up_past_now() {
        assert_eq!(next_fire_after(0, 10, 25), 30);
        assert_eq!(next_fire_after(100, 10, 50), 100); // already ahead
        assert_eq!(next_fire_after(5, 1, 5), 6);
    }

    #[test]
    fn plan_enqueue_never_exceeds_bound() {
        // room: push, no drop
        let p = plan_enqueue(3, 8, true);
        assert!(p.push && !p.drop_front);
        // full + drop_oldest: drop one, push one -> length unchanged
        let p = plan_enqueue(8, 8, true);
        assert!(p.push && p.drop_front);
        // full + drop_newest: reject
        let p = plan_enqueue(8, 8, false);
        assert!(!p.push && !p.drop_front);
    }
}
