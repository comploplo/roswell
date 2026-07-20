//! ROS logging and `/rosout` publishing.

use std::collections::HashMap;
use std::ffi::c_char;

use crate::msgs::{rcl_interfaces__Log as LogMsg, RosString};
use crate::time::{Duration, Time};
use crate::transport::{MsgPublisher, Qos, Transport};

/// ROS log severity values from `rcl_interfaces/msg/Log`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[repr(u8)]
pub enum Severity {
    Debug = 10,
    Info = 20,
    Warn = 30,
    Error = 40,
    Fatal = 50,
}

impl Severity {
    #[must_use]
    pub const fn as_u8(self) -> u8 {
        self as u8
    }
}

/// Static log call-site metadata.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct LogSite {
    pub file: &'static str,
    pub function: &'static str,
    pub line: u32,
}

impl LogSite {
    #[must_use]
    pub const fn new(file: &'static str, function: &'static str, line: u32) -> Self {
        Self {
            file,
            function,
            line,
        }
    }
}

/// # Safety
/// `bytes` must be valid UTF-8 followed by a trailing NUL and must outlive every
/// read of the returned borrowed (non-owning, `capacity == 0`) `RosString`.
unsafe fn borrowed_string(bytes: &mut [u8]) -> RosString {
    debug_assert_eq!(bytes.last(), Some(&0));
    // SAFETY: caller guarantees valid UTF-8 + NUL that outlives the borrowed view.
    unsafe {
        RosString::from_raw_parts(
            bytes.as_mut_ptr().cast::<c_char>(),
            bytes.len().saturating_sub(1),
            0,
        )
    }
}

/// Publishes node logs to `/rosout`.
pub struct Rosout<P> {
    node_name: String,
    publisher: P,
    last: HashMap<(LogSite, Severity), Time>,
}

impl Rosout<()> {
    #[must_use]
    pub fn new<T: Transport>(
        transport: &T,
        node_name: impl Into<String>,
    ) -> Rosout<T::Pub<LogMsg>> {
        Rosout {
            node_name: node_name.into(),
            publisher: transport.publisher::<LogMsg>("/rosout", Qos::Default),
            last: HashMap::new(),
        }
    }
}

impl<P: MsgPublisher<LogMsg>> Rosout<P> {
    pub fn log(&mut self, stamp: Time, level: Severity, message: &str, site: LogSite) {
        self.publish(stamp, level, message, site);
    }

    pub fn throttled(
        &mut self,
        stamp: Time,
        every: Duration,
        level: Severity,
        message: &str,
        site: LogSite,
    ) -> bool {
        let key = (site, level);
        if self
            .last
            .get(&key)
            .is_some_and(|prev| stamp - *prev < every)
        {
            return false;
        }
        self.last.insert(key, stamp);
        self.publish(stamp, level, message, site);
        true
    }

    fn publish(&self, stamp: Time, level: Severity, message: &str, site: LogSite) {
        let mut name = nul_terminated(self.node_name.as_str());
        let mut message = nul_terminated(message);
        let mut file = nul_terminated(site.file);
        let mut function = nul_terminated(site.function);
        // SAFETY: all four buffers are NUL-terminated valid UTF-8 (from `nul_terminated`)
        // and outlive `msg`, which is published before they drop. The strings are
        // borrowed (capacity == 0), so the generated value leaks nothing on drop.
        let msg = LogMsg {
            stamp: stamp.to_msg(),
            level: level.as_u8(),
            name: unsafe { borrowed_string(&mut name) },
            msg: unsafe { borrowed_string(&mut message) },
            file: unsafe { borrowed_string(&mut file) },
            function: unsafe { borrowed_string(&mut function) },
            line: site.line,
        };
        self.publisher.publish(msg);
    }
}

fn nul_terminated(s: &str) -> Vec<u8> {
    let mut bytes = s.as_bytes().to_vec();
    bytes.push(0);
    bytes
}

#[cfg(test)]
mod tests {
    use crate::codec::CdrMsg;
    use crate::time::{Duration, Time};
    use crate::transport::MsgPublisher;

    use super::{LogMsg, LogSite, RosString, Rosout, Severity};

    struct Sink(std::cell::RefCell<Vec<Vec<u8>>>);

    impl MsgPublisher<LogMsg> for Sink {
        fn publish(&self, msg: LogMsg) {
            self.0.borrow_mut().push(msg.encode());
        }
    }

    #[test]
    fn log_msg_round_trips() {
        let mut msg = LogMsg {
            stamp: Time::from_parts(1, 2).to_msg(),
            level: Severity::Warn.as_u8(),
            name: RosString::alloc("node"),
            msg: RosString::alloc("careful"),
            file: RosString::alloc("file.rs"),
            function: RosString::alloc("f"),
            line: 42,
        };
        let mut back = LogMsg::from_cdr(&msg.to_cdr(crate::msgs::Endian::Little)).unwrap();
        unsafe {
            assert_eq!(back.name.as_str(), "node");
            assert_eq!(back.msg.as_str(), "careful");
            assert_eq!(back.file.as_str(), "file.rs");
            assert_eq!(back.function.as_str(), "f");
            assert_eq!(back.level, Severity::Warn.as_u8());
            msg.fini();
            back.fini();
        }
    }

    #[test]
    fn throttling_suppresses_until_period_passes() {
        let sink = Sink(std::cell::RefCell::new(Vec::new()));
        let mut rosout = Rosout {
            node_name: "n".to_string(),
            publisher: sink,
            last: std::collections::HashMap::new(),
        };
        assert!(rosout.throttled(
            Time::from_nanos(0),
            Duration::from_secs(1),
            Severity::Info,
            "x",
            LogSite::new("f", "g", 1)
        ));
        assert!(!rosout.throttled(
            Time::from_millis(500),
            Duration::from_secs(1),
            Severity::Info,
            "x",
            LogSite::new("f", "g", 1)
        ));
        assert!(rosout.throttled(
            Time::from_secs(2),
            Duration::from_secs(1),
            Severity::Info,
            "x",
            LogSite::new("f", "g", 1)
        ));
    }
}
