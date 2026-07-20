//! ROS time primitives and `/clock` integration.
#![deny(unsafe_code)]

use std::ops::{Add, Sub};
use std::time::{Duration as StdDuration, Instant, SystemTime, UNIX_EPOCH};

use crate::msgs::builtin_interfaces__Time;
use crate::transport::{MsgSubscriber, Qos, Transport};

/// Signed ROS duration, stored as nanoseconds.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord)]
pub struct Duration {
    nanos: i64,
}

impl Duration {
    #[must_use]
    pub const fn from_nanos(nanos: i64) -> Self {
        Self { nanos }
    }

    #[must_use]
    pub const fn from_millis(millis: i64) -> Self {
        Self {
            nanos: millis.saturating_mul(1_000_000),
        }
    }

    #[must_use]
    pub const fn from_secs(secs: i64) -> Self {
        Self {
            nanos: secs.saturating_mul(1_000_000_000),
        }
    }

    #[must_use]
    pub const fn as_nanos(self) -> i64 {
        self.nanos
    }

    #[must_use]
    pub fn abs_std(self) -> StdDuration {
        StdDuration::from_nanos(self.nanos.unsigned_abs())
    }
}

/// ROS timestamp, normalized to nanoseconds since epoch or sim-time origin.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Time {
    nanos: i64,
}

impl Time {
    #[must_use]
    pub const fn from_nanos(nanos: i64) -> Self {
        Self { nanos }
    }

    #[must_use]
    pub const fn from_millis(millis: i64) -> Self {
        Self::from_nanos(millis.saturating_mul(1_000_000))
    }

    #[must_use]
    pub const fn from_secs(secs: i64) -> Self {
        Self::from_nanos(secs.saturating_mul(1_000_000_000))
    }

    #[must_use]
    pub const fn from_parts(sec: i32, nanosec: u32) -> Self {
        Self {
            nanos: (sec as i64)
                .saturating_mul(1_000_000_000)
                .saturating_add(nanosec as i64),
        }
    }

    #[must_use]
    pub fn now_system() -> Self {
        let d = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap_or_default();
        Self::from_nanos(d.as_nanos().min(i64::MAX as u128) as i64)
    }

    #[must_use]
    pub const fn as_nanos(self) -> i64 {
        self.nanos
    }

    #[must_use]
    pub fn to_msg(self) -> builtin_interfaces__Time {
        let sec = self.nanos.div_euclid(1_000_000_000);
        let nanosec = self.nanos.rem_euclid(1_000_000_000);
        builtin_interfaces__Time {
            sec: sec.clamp(i64::from(i32::MIN), i64::from(i32::MAX)) as i32,
            nanosec: nanosec as u32,
        }
    }

    #[must_use]
    pub const fn from_msg(msg: &builtin_interfaces__Time) -> Self {
        Self::from_parts(msg.sec, msg.nanosec)
    }
}

impl Add<Duration> for Time {
    type Output = Time;

    fn add(self, rhs: Duration) -> Self::Output {
        Time::from_nanos(self.nanos.saturating_add(rhs.nanos))
    }
}

impl Sub<Duration> for Time {
    type Output = Time;

    fn sub(self, rhs: Duration) -> Self::Output {
        Time::from_nanos(self.nanos.saturating_sub(rhs.nanos))
    }
}

impl Sub<Time> for Time {
    type Output = Duration;

    fn sub(self, rhs: Time) -> Self::Output {
        Duration::from_nanos(self.nanos.saturating_sub(rhs.nanos))
    }
}

/// `rosgraph_msgs/msg/Clock`.
#[repr(C)]
pub struct ClockMsg {
    pub clock: builtin_interfaces__Time,
}

impl ClockMsg {
    #[must_use]
    pub fn to_cdr(&self, endian: crate::msgs::Endian) -> Vec<u8> {
        let mut w = crate::msgs::Writer::new(endian);
        self.clock.serialize_into(&mut w);
        w.finish()
    }

    pub fn from_cdr(buf: &[u8]) -> Result<Self, crate::msgs::CdrError> {
        let mut r = crate::msgs::Reader::new(buf)?;
        Ok(Self {
            clock: builtin_interfaces__Time::deserialize_from(&mut r)?,
        })
    }
}

impl crate::codec::CdrMsg for ClockMsg {
    const TYPE_NAME: &'static str = "rosgraph_msgs::msg::dds_::Clock_";

    fn encode(&self) -> Vec<u8> {
        self.to_cdr(crate::msgs::Endian::Little)
    }

    fn decode(buf: &[u8]) -> Result<Self, crate::codec::CodecError> {
        Self::from_cdr(buf).map_err(|_| crate::codec::CodecError("clock cdr decode failed"))
    }
}

/// Source of node time: wall clock or the latest `/clock` sample.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ClockMode {
    Wall,
    Sim,
}

/// ROS clock state. Call [`Clock::poll`] from the node loop when sim time is on.
pub struct Clock<S = ()> {
    mode: ClockMode,
    sim_time: Option<Time>,
    subscriber: Option<S>,
}

impl Clock {
    #[must_use]
    pub const fn wall() -> Self {
        Self {
            mode: ClockMode::Wall,
            sim_time: None,
            subscriber: None,
        }
    }
}

impl<S: MsgSubscriber<ClockMsg>> Clock<S> {
    #[must_use]
    pub fn sim(subscriber: S) -> Self {
        Self {
            mode: ClockMode::Sim,
            sim_time: None,
            subscriber: Some(subscriber),
        }
    }

    pub fn poll(&mut self) -> Option<Time> {
        let sub = self.subscriber.as_mut()?;
        while let Some(msg) = sub.take() {
            self.sim_time = Some(Time::from_msg(&msg.clock));
        }
        self.sim_time
    }
}

impl<S> Clock<S> {
    /// Wall-clock instance for any subscriber type `S` (no `/clock`
    /// subscription). Lets a node hold a `Clock<DdsSub<ClockMsg>>` field even
    /// when running on wall time.
    #[must_use]
    pub const fn wall_typed() -> Self {
        Self {
            mode: ClockMode::Wall,
            sim_time: None,
            subscriber: None,
        }
    }

    #[must_use]
    pub const fn mode(&self) -> ClockMode {
        self.mode
    }

    /// The `/clock` subscriber, if any — lets the executor register its event
    /// source in the waitset so sim time advances on data, not by polling.
    pub fn subscriber_mut(&mut self) -> Option<&mut S> {
        self.subscriber.as_mut()
    }

    #[must_use]
    pub fn now(&self) -> Time {
        match self.mode {
            ClockMode::Wall => Time::now_system(),
            ClockMode::Sim => self.sim_time.unwrap_or_default(),
        }
    }
}

impl Clock {
    #[must_use]
    pub fn from_transport<T: Transport>(
        transport: &T,
        use_sim_time: bool,
    ) -> Clock<T::Sub<ClockMsg>> {
        if use_sim_time {
            // `/clock` is best-effort keep-last, matching rclcpp.
            Clock::sim(transport.subscriber::<ClockMsg>("/clock", Qos::SensorData))
        } else {
            Clock {
                mode: ClockMode::Wall,
                sim_time: None,
                subscriber: None,
            }
        }
    }
}

/// Loop-rate helper using monotonic wall time.
pub struct Rate {
    period: StdDuration,
    next: Instant,
}

impl Rate {
    #[must_use]
    pub fn hz(hz: f64) -> Self {
        let period = StdDuration::from_secs_f64(1.0 / hz.max(f64::EPSILON));
        Self {
            period,
            next: Instant::now() + period,
        }
    }

    pub fn sleep(&mut self) {
        let now = Instant::now();
        if self.next > now {
            std::thread::sleep(self.next - now);
        }
        self.next += self.period;
        if self.next < Instant::now() {
            self.next = Instant::now() + self.period;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{Duration, Time};

    #[test]
    fn time_duration_math_normalizes() {
        let t = Time::from_parts(10, 250);
        assert_eq!((t + Duration::from_nanos(750)).to_msg().sec, 10);
        assert_eq!((t + Duration::from_nanos(750)).to_msg().nanosec, 1_000);
        assert_eq!((t - Time::from_parts(9, 0)).as_nanos(), 1_000_000_250);
    }

    #[test]
    fn clock_msg_round_trips() {
        let msg = super::ClockMsg {
            clock: Time::from_parts(4, 5).to_msg(),
        };
        let back = super::ClockMsg::from_cdr(&msg.to_cdr(crate::msgs::Endian::Little)).unwrap();
        assert_eq!(Time::from_msg(&back.clock), Time::from_parts(4, 5));
    }
}
