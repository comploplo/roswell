//! Tiny argument cursor shared by the roswell-ros2-compat bins.
//!
//! Bundles the `std::env::args()` iterator with a `usage:` message so bins stop
//! repeating the `iter.next().ok_or_else(usage)?` value fetch and the
//! `.parse().map_err(|_| usage())?` typed parse (and the `fn usage()` wrapper).

use std::io;
use std::str::FromStr;

pub struct ArgCursor {
    iter: std::iter::Skip<std::env::Args>,
    usage: &'static str,
}

impl ArgCursor {
    /// Cursor over the process arguments (past argv[0]); `usage` is the message
    /// for the `InvalidInput` error returned on any missing/malformed argument.
    pub fn new(usage: &'static str) -> Self {
        Self {
            iter: std::env::args().skip(1),
            usage,
        }
    }

    /// The usage error surfaced on a missing or malformed argument.
    pub fn usage(&self) -> io::Error {
        io::Error::new(io::ErrorKind::InvalidInput, self.usage)
    }

    /// The next raw argument, if any.
    pub fn next_arg(&mut self) -> Option<String> {
        self.iter.next()
    }

    /// The next argument, or the usage error when absent.
    pub fn value(&mut self) -> io::Result<String> {
        self.iter.next().ok_or_else(|| self.usage())
    }

    /// The next argument parsed as `T`, or the usage error when absent/malformed.
    pub fn parse<T: FromStr>(&mut self) -> io::Result<T> {
        self.value()?.parse().map_err(|_| self.usage())
    }
}
