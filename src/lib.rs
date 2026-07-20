//! `roswell` — a from-scratch compiler for ROS message definitions.
//!
//! Milestone 1: parse ROS2 `.msg` files into an AST and generate FFI-compatible
//! type bindings (Rust / C / Python) that share one memory layout.

pub mod ast;
pub mod cdr;
pub mod codegen;
pub mod dynamic;
pub mod idl;
pub mod ir;
pub mod parser;
pub mod resolve;
pub mod typehash;
pub mod workspace;

pub use idl::parse_idl;
pub use parser::{ParseError, parse_action, parse_message, parse_service};
pub use resolve::{ResolveError, action_messages, resolve, service_messages};
