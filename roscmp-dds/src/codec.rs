//! Glue between roscmp's generated CDR (de)serializers and RustDDS's adapter
//! traits, plus the ROS2 naming conventions a vanilla node expects.
//!
//! RustDDS owns the RTPS protocol; the *payload* is encoded/decoded entirely by
//! our generated `to_cdr`/`from_cdr` (M2). A message participates by
//! implementing [`CdrMsg`] (one line via [`impl_cdr!`]).

use std::marker::PhantomData;

use bytes::Bytes;
use rustdds::{
    no_key::{Decode, DefaultDecoder, DeserializerAdapter, SerializerAdapter},
    RepresentationIdentifier,
};

/// Error from the roscmp CDR codec at the DDS boundary.
#[derive(Debug)]
pub struct CodecError(pub &'static str);
impl std::fmt::Display for CodecError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "roscmp codec: {}", self.0)
    }
}
impl std::error::Error for CodecError {}

/// A message that can ride RTPS using roscmp's CDR and ROS2 naming.
pub trait CdrMsg: 'static + Sized {
    /// DDS type name, e.g. `std_msgs::msg::dds_::String_`.
    const TYPE_NAME: &'static str;
    /// Full CDR message (4-byte encapsulation header + body), little-endian.
    fn encode(&self) -> Vec<u8>;
    /// Decode a full CDR message (header + body).
    fn decode(buf: &[u8]) -> Result<Self, CodecError>;
}

/// ROS2 topic-name mangling: `/chatter` -> DDS `rt/chatter`.
pub fn topic(ros_name: &str) -> String {
    format!("rt{ros_name}")
}

/// ROS2 service request/reply DDS topic names for `/<service>`.
pub fn service_topics(service: &str) -> (String, String) {
    let bare = service.trim_start_matches('/');
    (format!("rq/{bare}Request"), format!("rr/{bare}Reply"))
}

const SUPPORTED: [RepresentationIdentifier; 2] = [
    RepresentationIdentifier::CDR_LE,
    RepresentationIdentifier::CDR_BE,
];

/// Serializer adapter: hand RustDDS our CDR body (it prepends the header).
pub struct Ser<T>(PhantomData<T>);
impl<T: CdrMsg> SerializerAdapter<T> for Ser<T> {
    type Error = CodecError;
    fn output_encoding() -> RepresentationIdentifier {
        RepresentationIdentifier::CDR_LE
    }
    fn to_bytes(value: &T) -> Result<Bytes, CodecError> {
        Ok(Bytes::copy_from_slice(&value.encode()[4..]))
    }
}

/// Decoder: rebuild the 4-byte encapsulation header from `encoding`, then defer
/// to our `from_cdr`.
pub struct Dec<T>(PhantomData<T>);
impl<T> Copy for Dec<T> {}
impl<T> Clone for Dec<T> {
    fn clone(&self) -> Self {
        *self
    }
}
impl<T: CdrMsg> Decode<T> for Dec<T> {
    type Error = CodecError;
    fn decode_bytes(
        self,
        input: &[u8],
        encoding: RepresentationIdentifier,
    ) -> Result<T, CodecError> {
        let header: [u8; 4] = match encoding {
            RepresentationIdentifier::CDR_BE | RepresentationIdentifier::PL_CDR_BE => {
                [0x00, 0x00, 0x00, 0x00]
            }
            _ => [0x00, 0x01, 0x00, 0x00],
        };
        let mut buf = Vec::with_capacity(input.len() + 4);
        buf.extend_from_slice(&header);
        buf.extend_from_slice(input);
        T::decode(&buf)
    }
}

/// Deserializer adapter pairing [`Dec`] with [`CdrMsg`].
pub struct De<T>(PhantomData<T>);
impl<T: CdrMsg> DeserializerAdapter<T> for De<T> {
    type Error = CodecError;
    type Decoded = T;
    fn supported_encodings() -> &'static [RepresentationIdentifier] {
        &SUPPORTED
    }
    fn transform_decoded(decoded: T) -> T {
        decoded
    }
}
impl<T: CdrMsg> DefaultDecoder<T> for De<T> {
    type Decoder = Dec<T>;
    const DECODER: Dec<T> = Dec(PhantomData);
}

/// Implement [`CdrMsg`] for a generated type with its ROS2 DDS type name.
#[macro_export]
macro_rules! impl_cdr {
    ($t:ident, $name:literal) => {
        impl $crate::codec::CdrMsg for $crate::msgs::$t {
            const TYPE_NAME: &'static str = $name;
            fn encode(&self) -> Vec<u8> {
                self.to_cdr($crate::msgs::Endian::Little)
            }
            fn decode(buf: &[u8]) -> Result<Self, $crate::codec::CodecError> {
                Self::from_cdr(buf).map_err(|_| $crate::codec::CodecError("cdr decode failed"))
            }
        }
    };
}

impl_cdr!(std_msgs__String, "std_msgs::msg::dds_::String_");
impl_cdr!(geometry_msgs__Twist, "geometry_msgs::msg::dds_::Twist_");
impl_cdr!(
    example_interfaces__AddTwoInts_Request,
    "example_interfaces::srv::dds_::AddTwoInts_Request_"
);
impl_cdr!(
    example_interfaces__AddTwoInts_Response,
    "example_interfaces::srv::dds_::AddTwoInts_Response_"
);
impl_cdr!(turtlesim__Pose, "turtlesim::msg::dds_::Pose_");
