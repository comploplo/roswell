//! roscmp-dds: bridge roscmp's generated bindings to real RTPS via RustDDS,
//! keeping our CDR serializer as the single source of truth.

pub mod codec;
pub mod msgs;
pub mod transport;
