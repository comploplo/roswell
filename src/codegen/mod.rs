//! Code generation backends.
//!
//! All three backends emit types over the *same* C ABI: scalars use the
//! fixed-width integer/float types, nested messages embed by value, fixed
//! arrays are inline, and dynamic sequences / strings use a
//! `{ data, size, capacity }` triple. Because every backend agrees on this
//! layout, a struct generated for Rust, C, and Python has identical size and
//! field offsets — the property the layout tests verify.

pub mod c;
pub mod python;
pub mod rust;

use crate::ir::MsgId;

/// The shared symbol name for a message, e.g. `geometry_msgs__Point`.
///
/// Uses the rosidl-style `package__Name` convention (minus the `msg`
/// subnamespace) so the same identifier names the type in every language.
pub fn symbol(id: &MsgId) -> String {
    format!("{}__{}", id.package.replace('/', "_"), id.name)
}
