//! `diagnostic_msgs` status messages and `/diagnostics` publisher helper.
#![deny(unsafe_code)]

use crate::codec::{CdrMsg, CodecError};
use crate::msgs::{
    diagnostic_msgs__DiagnosticArray as ArrayGen, diagnostic_msgs__DiagnosticStatus as StatusGen,
    diagnostic_msgs__KeyValue as KeyValueGen, std_msgs__Header, RosSequence, RosString,
};
use crate::time::Time;
use crate::transport::{Dds, MsgPublisher, Qos, Transport};

#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct KeyValue {
    pub key: String,
    pub value: String,
}

impl KeyValue {
    #[must_use]
    pub fn new(key: impl Into<String>, value: impl Into<String>) -> Self {
        Self {
            key: key.into(),
            value: value.into(),
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[repr(u8)]
pub enum DiagnosticLevel {
    #[default]
    Ok = 0,
    Warn = 1,
    Error = 2,
    Stale = 3,
}

impl DiagnosticLevel {
    #[must_use]
    pub const fn from_u8(level: u8) -> Self {
        match level {
            1 => Self::Warn,
            2 => Self::Error,
            3 => Self::Stale,
            _ => Self::Ok,
        }
    }
}

#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct DiagnosticStatus {
    pub level: DiagnosticLevel,
    pub name: String,
    pub message: String,
    pub hardware_id: String,
    pub values: Vec<KeyValue>,
}

impl DiagnosticStatus {
    #[must_use]
    pub fn new(
        level: DiagnosticLevel,
        name: impl Into<String>,
        message: impl Into<String>,
    ) -> Self {
        Self {
            level,
            name: name.into(),
            message: message.into(),
            hardware_id: String::new(),
            values: Vec::new(),
        }
    }
}

#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Header {
    pub stamp: Time,
    pub frame_id: String,
}

#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct DiagnosticArray {
    pub header: Header,
    pub status: Vec<DiagnosticStatus>,
}

impl DiagnosticArray {
    #[must_use]
    pub fn new(stamp: Time, status: Vec<DiagnosticStatus>) -> Self {
        Self {
            header: Header {
                stamp,
                frame_id: String::new(),
            },
            status,
        }
    }

    fn to_gen(&self) -> ArrayGen {
        ArrayGen {
            header: std_msgs__Header {
                stamp: self.header.stamp.to_msg(),
                frame_id: RosString::alloc(&self.header.frame_id),
            },
            status: RosSequence::alloc(self.status.iter().map(status_to_gen).collect()),
        }
    }

    fn from_gen(g: &ArrayGen) -> Self {
        Self {
            header: Header {
                stamp: Time::from_msg(&g.header.stamp),
                frame_id: g.header.frame_id.as_str().to_string(),
            },
            status: g.status.as_slice().iter().map(status_from_gen).collect(),
        }
    }
}

fn status_to_gen(s: &DiagnosticStatus) -> StatusGen {
    StatusGen {
        level: s.level as u8,
        name: RosString::alloc(&s.name),
        message: RosString::alloc(&s.message),
        hardware_id: RosString::alloc(&s.hardware_id),
        values: RosSequence::alloc(
            s.values
                .iter()
                .map(|kv| KeyValueGen {
                    key: RosString::alloc(&kv.key),
                    value: RosString::alloc(&kv.value),
                })
                .collect(),
        ),
    }
}

fn status_from_gen(s: &StatusGen) -> DiagnosticStatus {
    DiagnosticStatus {
        level: DiagnosticLevel::from_u8(s.level),
        name: s.name.as_str().to_string(),
        message: s.message.as_str().to_string(),
        hardware_id: s.hardware_id.as_str().to_string(),
        values: s
            .values
            .as_slice()
            .iter()
            .map(|kv| KeyValue::new(kv.key.as_str(), kv.value.as_str()))
            .collect(),
    }
}

// The only `unsafe` here frees the C-ABI generated codec buffers (`fini`) after
// they cross the wire; the ergonomic `String`/`Vec` types above stay pure-safe.
#[allow(unsafe_code)]
impl CdrMsg for DiagnosticArray {
    const TYPE_NAME: &'static str = "diagnostic_msgs::msg::dds_::DiagnosticArray_";

    fn encode(&self) -> Vec<u8> {
        let mut g = self.to_gen();
        let bytes = g.encode();
        // SAFETY: `g` is a freshly-built owned value, finalized exactly once.
        unsafe { g.fini() };
        bytes
    }

    fn decode(buf: &[u8]) -> Result<Self, CodecError> {
        let mut g =
            ArrayGen::decode(buf).map_err(|_| CodecError("diagnostic-array decode failed"))?;
        let out = Self::from_gen(&g);
        // SAFETY: `g` was decoded (owned) and is finalized exactly once.
        unsafe { g.fini() };
        Ok(out)
    }
}

pub struct Diagnostics<P: MsgPublisher<DiagnosticArray>> {
    publisher: P,
}

impl Diagnostics<crate::transport::DdsPub<DiagnosticArray>> {
    #[must_use]
    pub fn new(dds: &Dds) -> Self {
        Self {
            publisher: dds.publisher::<DiagnosticArray>("/diagnostics", Qos::Default),
        }
    }
}

impl<P: MsgPublisher<DiagnosticArray>> Diagnostics<P> {
    pub fn publish(&self, stamp: Time, statuses: Vec<DiagnosticStatus>) {
        self.publisher
            .publish(DiagnosticArray::new(stamp, statuses));
    }
}

#[cfg(test)]
mod tests {
    use super::{CdrMsg, DiagnosticArray, DiagnosticLevel, DiagnosticStatus, KeyValue};
    use crate::time::Time;

    #[test]
    fn diagnostic_array_round_trips() {
        let mut status = DiagnosticStatus::new(DiagnosticLevel::Warn, "battery", "low");
        status.hardware_id = "pack0".into();
        status.values.push(KeyValue::new("voltage", "11.8"));
        let msg = DiagnosticArray::new(Time::from_parts(7, 8), vec![status]);
        let back = DiagnosticArray::decode(&msg.encode()).unwrap();
        assert_eq!(back, msg);
    }
}
