//! roswell-ros2-compat: bridge roswell's generated bindings to real RTPS via RustDDS,
//! keeping our CDR serializer as the single source of truth.

pub mod action;
#[cfg(feature = "tokio")]
pub mod async_rt;
pub mod codec;
pub mod diagnostics;
pub mod discovery;
pub mod graph;
pub mod lifecycle;
pub mod log;
pub mod msgs;
pub mod node;
pub mod parameters;
pub mod qos;
pub mod raw;
pub mod service;
pub mod tf;
pub mod time;
pub mod transport;
pub mod tunnel;
pub mod type_description;
