//! Minimal tf2 message support and transform buffer.

use std::collections::{HashMap, VecDeque};
use std::ffi::c_char;

use crate::msgs::{
    geometry_msgs__Quaternion, geometry_msgs__Transform as TransformMsg,
    geometry_msgs__TransformStamped as TransformStampedMsg, geometry_msgs__Vector3,
    std_msgs__Header, tf2_msgs__TFMessage as TFMessage, RosSequence, RosString,
};
use crate::time::Time;
use crate::transport::{MsgPublisher, MsgSubscriber, Qos, Transport};

/// # Safety
/// `bytes` must be valid UTF-8 followed by a trailing NUL and must outlive every
/// read of the returned borrowed (non-owning, `capacity == 0`) `RosString`.
unsafe fn borrowed_string(bytes: &mut [u8]) -> RosString {
    debug_assert!(bytes.last() == Some(&0));
    // SAFETY: caller guarantees valid UTF-8 + NUL that outlives the borrowed view.
    unsafe {
        RosString::from_raw_parts(
            bytes.as_mut_ptr().cast::<c_char>(),
            bytes.len().saturating_sub(1),
            0,
        )
    }
}

fn nul_terminated(s: &str) -> Vec<u8> {
    let mut bytes = s.as_bytes().to_vec();
    bytes.push(0);
    bytes
}

/// A single-transform `TFMessage` whose frame strings borrow `parent`/`child`
/// for zero-copy publishing.
///
/// # Safety
/// `parent` and `child` must each be valid UTF-8 followed by a trailing NUL and
/// outlive every read of the returned message (its frame strings are borrowed
/// views, never freed).
unsafe fn one_borrowed(
    stamp: Time,
    parent: &mut [u8],
    child: &mut [u8],
    transform: Transform,
) -> TFMessage {
    // SAFETY: forwarded from this fn's contract on `parent`/`child`.
    let stamped = unsafe {
        TransformStampedMsg {
            header: std_msgs__Header {
                stamp: stamp.to_msg(),
                frame_id: borrowed_string(parent),
            },
            child_frame_id: borrowed_string(child),
            transform: transform.into_msg(),
        }
    };
    TFMessage {
        transforms: RosSequence::alloc(vec![stamped]),
    }
}

/// Native transform used by the in-memory buffer.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Transform {
    pub translation: [f64; 3],
    /// Quaternion `[x, y, z, w]`.
    pub rotation: [f64; 4],
}

impl Transform {
    pub const IDENTITY: Self = Self {
        translation: [0.0, 0.0, 0.0],
        rotation: [0.0, 0.0, 0.0, 1.0],
    };

    #[must_use]
    pub fn from_msg(msg: &TransformMsg) -> Self {
        Self {
            translation: [msg.translation.x, msg.translation.y, msg.translation.z],
            rotation: [
                msg.rotation.x,
                msg.rotation.y,
                msg.rotation.z,
                msg.rotation.w,
            ],
        }
    }

    #[must_use]
    pub fn into_msg(self) -> TransformMsg {
        TransformMsg {
            translation: geometry_msgs__Vector3 {
                x: self.translation[0],
                y: self.translation[1],
                z: self.translation[2],
            },
            rotation: geometry_msgs__Quaternion {
                x: self.rotation[0],
                y: self.rotation[1],
                z: self.rotation[2],
                w: self.rotation[3],
            },
        }
    }

    #[must_use]
    pub fn inverse(self) -> Self {
        let inv_rot = quat_conjugate(quat_normalize(self.rotation));
        let inv_trans = rotate(inv_rot, scale(self.translation, -1.0));
        Self {
            translation: inv_trans,
            rotation: inv_rot,
        }
    }

    #[must_use]
    pub fn then(self, child: Self) -> Self {
        Self {
            translation: add(self.translation, rotate(self.rotation, child.translation)),
            rotation: quat_normalize(quat_mul(self.rotation, child.rotation)),
        }
    }

    #[must_use]
    pub fn interpolate(a: Self, b: Self, ratio: f64) -> Self {
        let t = ratio.clamp(0.0, 1.0);
        Self {
            translation: [
                lerp(a.translation[0], b.translation[0], t),
                lerp(a.translation[1], b.translation[1], t),
                lerp(a.translation[2], b.translation[2], t),
            ],
            rotation: quat_nlerp(a.rotation, b.rotation, t),
        }
    }
}

#[derive(Clone, Debug)]
struct StampedTransform {
    stamp: Time,
    parent: String,
    child: String,
    transform: Transform,
}

/// Errors returned by [`TfBuffer::lookup`].
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum LookupError {
    EmptyFrame,
    NoPath { target: String, source: String },
    Extrapolation { parent: String, child: String },
}

/// Time-indexed transform tree.
#[derive(Default)]
pub struct TfBuffer {
    edges: HashMap<(String, String), Vec<StampedTransform>>,
    /// Neighbour index: `frame -> [(neighbour, forward)]`, where `forward` is
    /// true when the stored edge is keyed `(frame, neighbour)` (apply the
    /// transform directly) and false when keyed `(neighbour, frame)` (apply its
    /// inverse). One entry per directed edge, so BFS steps are O(degree).
    adjacency: HashMap<String, Vec<(String, bool)>>,
}

impl TfBuffer {
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    pub fn insert(&mut self, stamp: Time, parent: &str, child: &str, transform: Transform) {
        let key = (parent.to_string(), child.to_string());
        let is_new_edge = !self.edges.contains_key(&key);
        let edge = self.edges.entry(key).or_default();
        edge.push(StampedTransform {
            stamp,
            parent: parent.to_string(),
            child: child.to_string(),
            transform,
        });
        edge.sort_by_key(|sample| sample.stamp);
        if is_new_edge {
            self.adjacency
                .entry(parent.to_string())
                .or_default()
                .push((child.to_string(), true));
            self.adjacency
                .entry(child.to_string())
                .or_default()
                .push((parent.to_string(), false));
        }
    }

    pub fn insert_msg(&mut self, msg: &TransformStampedMsg) {
        self.insert(
            Time::from_msg(&msg.header.stamp),
            msg.header.frame_id.as_str(),
            msg.child_frame_id.as_str(),
            Transform::from_msg(&msg.transform),
        );
    }

    pub fn insert_tf_message(&mut self, msg: &TFMessage) {
        for transform in msg.transforms.as_slice() {
            self.insert_msg(transform);
        }
    }

    pub fn lookup(
        &self,
        target_frame: &str,
        source_frame: &str,
        stamp: Time,
    ) -> Result<Transform, LookupError> {
        self.lookup_impl(target_frame, source_frame, Some(stamp))
    }

    /// Like [`TfBuffer::lookup`], but samples each edge at its newest stamp —
    /// tf2's "time zero / latest available" semantics.
    pub fn lookup_latest(
        &self,
        target_frame: &str,
        source_frame: &str,
    ) -> Result<Transform, LookupError> {
        self.lookup_impl(target_frame, source_frame, None)
    }

    fn lookup_impl(
        &self,
        target_frame: &str,
        source_frame: &str,
        stamp: Option<Time>,
    ) -> Result<Transform, LookupError> {
        if target_frame.is_empty() || source_frame.is_empty() {
            return Err(LookupError::EmptyFrame);
        }
        if target_frame == source_frame {
            return Ok(Transform::IDENTITY);
        }

        let mut queue = VecDeque::from([(target_frame.to_string(), Transform::IDENTITY)]);
        let mut seen = std::collections::HashSet::from([target_frame.to_string()]);
        while let Some((frame, chain)) = queue.pop_front() {
            let Some(neighbours) = self.adjacency.get(&frame) else {
                continue;
            };
            for (neighbour, forward) in neighbours {
                if !seen.insert(neighbour.clone()) {
                    continue;
                }
                let key = if *forward {
                    (frame.clone(), neighbour.clone())
                } else {
                    (neighbour.clone(), frame.clone())
                };
                let samples = &self.edges[&key];
                let edge = sample_at(samples, stamp)?;
                let step = if *forward {
                    edge.transform
                } else {
                    edge.transform.inverse()
                };
                let next_chain = chain.then(step);
                if neighbour == source_frame {
                    return Ok(next_chain);
                }
                queue.push_back((neighbour.clone(), next_chain));
            }
        }
        Err(LookupError::NoPath {
            target: target_frame.to_string(),
            source: source_frame.to_string(),
        })
    }
}

fn sample_at(
    samples: &[StampedTransform],
    stamp: Option<Time>,
) -> Result<StampedTransform, LookupError> {
    let Some(stamp) = stamp else {
        // Latest-available semantics: the newest sample on the edge.
        return Ok(samples
            .last()
            .expect("empty edges are never inserted")
            .clone());
    };
    match samples {
        [] => unreachable!("empty edge vectors are never inserted"),
        [one] => Ok(one.clone()),
        _ if stamp < samples[0].stamp || stamp > samples[samples.len() - 1].stamp => {
            let first = &samples[0];
            Err(LookupError::Extrapolation {
                parent: first.parent.clone(),
                child: first.child.clone(),
            })
        }
        _ => {
            for pair in samples.windows(2) {
                let a = &pair[0];
                let b = &pair[1];
                if stamp == a.stamp {
                    return Ok(a.clone());
                }
                if stamp >= a.stamp && stamp <= b.stamp {
                    let span = (b.stamp - a.stamp).abs_std().as_secs_f64();
                    let offset = (stamp - a.stamp).abs_std().as_secs_f64();
                    return Ok(StampedTransform {
                        stamp,
                        parent: a.parent.clone(),
                        child: a.child.clone(),
                        transform: Transform::interpolate(a.transform, b.transform, offset / span),
                    });
                }
            }
            Ok(samples[samples.len() - 1].clone())
        }
    }
}

/// Publishes dynamic transforms on `/tf`.
pub struct TfBroadcaster<P> {
    publisher: P,
}

impl TfBroadcaster<()> {
    #[must_use]
    pub fn new<T: Transport>(transport: &T) -> TfBroadcaster<T::Pub<TFMessage>> {
        TfBroadcaster {
            publisher: transport.publisher::<TFMessage>("/tf", Qos::Default),
        }
    }
}

impl<P: MsgPublisher<TFMessage>> TfBroadcaster<P> {
    pub fn send(&self, stamp: Time, parent: &str, child: &str, transform: Transform) {
        let mut parent = nul_terminated(parent);
        let mut child = nul_terminated(child);
        // SAFETY: `parent`/`child` are NUL-terminated valid UTF-8 (from `nul_terminated`)
        // and outlive `msg`, which is published before they drop.
        let msg = unsafe { one_borrowed(stamp, &mut parent, &mut child, transform) };
        self.publisher.publish(msg);
    }
}

/// Publishes static transforms on `/tf_static` using transient-local QoS.
pub struct StaticTfBroadcaster<P> {
    publisher: P,
}

impl StaticTfBroadcaster<()> {
    #[must_use]
    pub fn new<T: Transport>(transport: &T) -> StaticTfBroadcaster<T::Pub<TFMessage>> {
        StaticTfBroadcaster {
            publisher: transport.publisher::<TFMessage>("/tf_static", Qos::Latched),
        }
    }
}

impl<P: MsgPublisher<TFMessage>> StaticTfBroadcaster<P> {
    pub fn send(&self, parent: &str, child: &str, transform: Transform) {
        let mut parent = nul_terminated(parent);
        let mut child = nul_terminated(child);
        // SAFETY: `parent`/`child` are NUL-terminated valid UTF-8 (from `nul_terminated`)
        // and outlive `msg`, which is published before they drop.
        let msg = unsafe { one_borrowed(Time::default(), &mut parent, &mut child, transform) };
        self.publisher.publish(msg);
    }
}

/// Subscribes to `/tf` and `/tf_static`, inserting pending samples into a buffer.
pub struct TfListener<D, S> {
    dynamic: D,
    statics: S,
}

impl TfListener<(), ()> {
    #[must_use]
    pub fn new<T: Transport>(transport: &T) -> TfListener<T::Sub<TFMessage>, T::Sub<TFMessage>> {
        TfListener {
            dynamic: transport.subscriber::<TFMessage>("/tf", Qos::Default),
            statics: transport.subscriber::<TFMessage>("/tf_static", Qos::Latched),
        }
    }
}

impl<D: MsgSubscriber<TFMessage>, S: MsgSubscriber<TFMessage>> TfListener<D, S> {
    pub fn poll_into(&mut self, buffer: &mut TfBuffer) -> usize {
        let mut count = 0;
        while let Some(msg) = self.dynamic.take() {
            count += unsafe { insert_and_fini(buffer, msg) };
        }
        while let Some(msg) = self.statics.take() {
            count += unsafe { insert_and_fini(buffer, msg) };
        }
        count
    }
}

unsafe fn insert_and_fini(buffer: &mut TfBuffer, mut msg: TFMessage) -> usize {
    let count = msg.transforms.len();
    buffer.insert_tf_message(&msg);
    // SAFETY: `msg` owns its sequence and is consumed exactly once here.
    unsafe { msg.fini() };
    count
}

fn lerp(a: f64, b: f64, t: f64) -> f64 {
    a + (b - a) * t
}

fn add(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

fn scale(v: [f64; 3], s: f64) -> [f64; 3] {
    [v[0] * s, v[1] * s, v[2] * s]
}

fn quat_mul(a: [f64; 4], b: [f64; 4]) -> [f64; 4] {
    [
        a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1],
        a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0],
        a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3],
        a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2],
    ]
}

fn quat_conjugate(q: [f64; 4]) -> [f64; 4] {
    [-q[0], -q[1], -q[2], q[3]]
}

fn quat_normalize(q: [f64; 4]) -> [f64; 4] {
    let n = (q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]).sqrt();
    if n <= f64::EPSILON {
        Transform::IDENTITY.rotation
    } else {
        [q[0] / n, q[1] / n, q[2] / n, q[3] / n]
    }
}

fn quat_nlerp(mut a: [f64; 4], b: [f64; 4], t: f64) -> [f64; 4] {
    if dot4(a, b) < 0.0 {
        a = [-a[0], -a[1], -a[2], -a[3]];
    }
    quat_normalize([
        lerp(a[0], b[0], t),
        lerp(a[1], b[1], t),
        lerp(a[2], b[2], t),
        lerp(a[3], b[3], t),
    ])
}

fn dot4(a: [f64; 4], b: [f64; 4]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3]
}

fn rotate(q: [f64; 4], v: [f64; 3]) -> [f64; 3] {
    let p = [v[0], v[1], v[2], 0.0];
    let r = quat_mul(
        quat_mul(quat_normalize(q), p),
        quat_conjugate(quat_normalize(q)),
    );
    [r[0], r[1], r[2]]
}

#[cfg(test)]
mod tests {
    use super::{TFMessage, TfBuffer, Transform, TransformStampedMsg};
    use crate::msgs::{std_msgs__Header, RosSequence, RosString};
    use crate::time::Time;

    #[test]
    fn tf_message_round_trips() {
        let mut msg = TFMessage {
            transforms: RosSequence::alloc(vec![TransformStampedMsg {
                header: std_msgs__Header {
                    stamp: Time::from_secs(2).to_msg(),
                    frame_id: RosString::alloc("map"),
                },
                child_frame_id: RosString::alloc("base"),
                transform: Transform {
                    translation: [1.0, 2.0, 3.0],
                    rotation: [0.0, 0.0, 0.0, 1.0],
                }
                .into_msg(),
            }]),
        };
        let mut back = TFMessage::from_cdr(&msg.to_cdr(crate::msgs::Endian::Little)).unwrap();
        unsafe {
            let transforms = back.transforms.as_slice();
            assert_eq!(transforms.len(), 1);
            assert_eq!(transforms[0].header.frame_id.as_str(), "map");
            assert_eq!(transforms[0].child_frame_id.as_str(), "base");
            back.fini();
            msg.fini();
        }
    }

    #[test]
    fn buffer_interpolates_and_composes_chain() {
        let mut buffer = TfBuffer::new();
        buffer.insert(
            Time::from_secs(0),
            "map",
            "odom",
            Transform {
                translation: [0.0, 0.0, 0.0],
                rotation: [0.0, 0.0, 0.0, 1.0],
            },
        );
        buffer.insert(
            Time::from_secs(10),
            "map",
            "odom",
            Transform {
                translation: [10.0, 0.0, 0.0],
                rotation: [0.0, 0.0, 0.0, 1.0],
            },
        );
        buffer.insert(
            Time::from_secs(0),
            "odom",
            "base",
            Transform {
                translation: [1.0, 2.0, 0.0],
                rotation: [0.0, 0.0, 0.0, 1.0],
            },
        );
        let got = buffer.lookup("map", "base", Time::from_secs(5)).unwrap();
        assert_eq!(got.translation, [6.0, 2.0, 0.0]);
    }

    #[test]
    fn reinserting_an_edge_keeps_a_single_adjacency_entry() {
        let mut buffer = TfBuffer::new();
        let shift = |x: f64| Transform {
            translation: [x, 0.0, 0.0],
            rotation: [0.0, 0.0, 0.0, 1.0],
        };
        // Same directed edge inserted twice: new sample, but no duplicate
        // neighbour entry.
        buffer.insert(Time::from_secs(0), "map", "base", shift(1.0));
        buffer.insert(Time::from_secs(10), "map", "base", shift(3.0));
        assert_eq!(buffer.adjacency["map"], vec![("base".to_string(), true)]);
        assert_eq!(buffer.adjacency["base"], vec![("map".to_string(), false)]);
        let got = buffer.lookup("map", "base", Time::from_secs(5)).unwrap();
        assert_eq!(got.translation, [2.0, 0.0, 0.0]);
        // Reverse direction still resolves via the inverse.
        let back = buffer.lookup("base", "map", Time::from_secs(5)).unwrap();
        assert_eq!(back.translation, [-2.0, 0.0, 0.0]);
    }
}
