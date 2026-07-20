//! ROS managed-node lifecycle state machine and service messages.

use std::ffi::c_char;

use crate::msgs::{
    lifecycle_msgs__ChangeState_Request as ChangeStateRequest,
    lifecycle_msgs__ChangeState_Response as ChangeStateResponse,
    lifecycle_msgs__GetState_Request as GetStateRequest,
    lifecycle_msgs__GetState_Response as GetStateResponse, lifecycle_msgs__State as StateMsg,
    lifecycle_msgs__Transition as TransitionMsg, RosString,
};
use crate::service::Service;
use crate::transport::Dds;

/// A `State` wire message borrowing its `'static` label (capacity 0 ⇒ never
/// freed), so writing it to DDS leaks nothing.
fn state_msg(state: State) -> StateMsg {
    let label = state.label();
    StateMsg {
        id: state as u8,
        // SAFETY: `label` is a `'static` str; the borrowed RosString (capacity 0)
        // is only read (never freed) and its backing outlives every use.
        label: unsafe {
            RosString::from_raw_parts(label.as_ptr().cast::<c_char>().cast_mut(), label.len(), 0)
        },
    }
}

/// A `Transition` wire message borrowing its `'static` label (see [`state_msg`]).
fn transition_msg(transition: Transition) -> TransitionMsg {
    let label = transition.label();
    TransitionMsg {
        id: transition as u8,
        // SAFETY: as in `state_msg` — `label` is `'static` and borrowed.
        label: unsafe {
            RosString::from_raw_parts(label.as_ptr().cast::<c_char>().cast_mut(), label.len(), 0)
        },
    }
}

/// A `ChangeState` request carrying `transition` (borrowed label; see [`state_msg`]).
#[must_use]
pub fn change_state_request(transition: Transition) -> ChangeStateRequest {
    ChangeStateRequest {
        transition: transition_msg(transition),
    }
}

/// A `GetState` response reporting `state` (borrowed label; see [`state_msg`]).
#[must_use]
pub fn get_state_response(state: State) -> GetStateResponse {
    GetStateResponse {
        current_state: state_msg(state),
    }
}

/// Primary lifecycle states from `lifecycle_msgs/msg/State`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum State {
    Unknown = 0,
    Unconfigured = 1,
    Inactive = 2,
    Active = 3,
    Finalized = 4,
}

impl State {
    #[must_use]
    pub const fn label(self) -> &'static str {
        match self {
            State::Unknown => "unknown",
            State::Unconfigured => "unconfigured",
            State::Inactive => "inactive",
            State::Active => "active",
            State::Finalized => "finalized",
        }
    }

    #[must_use]
    pub const fn from_id(id: u8) -> Self {
        match id {
            1 => State::Unconfigured,
            2 => State::Inactive,
            3 => State::Active,
            4 => State::Finalized,
            _ => State::Unknown,
        }
    }
}

/// Transition ids from `lifecycle_msgs/msg/Transition`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum Transition {
    Configure = 1,
    Cleanup = 2,
    Activate = 3,
    Deactivate = 4,
    Shutdown = 5,
}

impl Transition {
    #[must_use]
    pub const fn label(self) -> &'static str {
        match self {
            Transition::Configure => "configure",
            Transition::Cleanup => "cleanup",
            Transition::Activate => "activate",
            Transition::Deactivate => "deactivate",
            Transition::Shutdown => "shutdown",
        }
    }

    #[must_use]
    pub const fn from_id(id: u8) -> Option<Self> {
        match id {
            1 => Some(Transition::Configure),
            2 => Some(Transition::Cleanup),
            3 => Some(Transition::Activate),
            4 => Some(Transition::Deactivate),
            5 => Some(Transition::Shutdown),
            _ => None,
        }
    }
}

/// A small lifecycle state machine. ROS service wrappers can delegate policy to
/// this type and publish state changes around it.
#[derive(Clone, Debug)]
pub struct Lifecycle {
    state: State,
}

impl Default for Lifecycle {
    fn default() -> Self {
        Self {
            state: State::Unconfigured,
        }
    }
}

/// DDS service endpoints for a managed node's lifecycle surface.
pub struct LifecycleServices {
    lifecycle: Lifecycle,
    change_state: Service<ChangeStateRequest, ChangeStateResponse>,
    get_state: Service<GetStateRequest, GetStateResponse>,
}

impl LifecycleServices {
    #[must_use]
    pub fn new(dds: &Dds, node_name: &str) -> Self {
        let base = format!("/{}", node_name.trim_matches('/'));
        Self {
            lifecycle: Lifecycle::new(),
            change_state: Service::new(dds, &format!("{base}/change_state")),
            get_state: Service::new(dds, &format!("{base}/get_state")),
        }
    }

    #[must_use]
    pub const fn state(&self) -> State {
        self.lifecycle.state()
    }

    pub fn serve_pending(&mut self) -> usize {
        let mut served = 0;
        served += self.change_state.serve_pending(|req| {
            let transition = Transition::from_id(req.transition.id);
            let success = transition
                .and_then(|transition| self.lifecycle.transition(transition).ok())
                .is_some();
            ChangeStateResponse { success }
        });
        served += self
            .get_state
            .serve_pending(|_req| get_state_response(self.lifecycle.state()));
        served
    }
}

impl Lifecycle {
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    #[must_use]
    pub const fn state(&self) -> State {
        self.state
    }

    pub fn transition(&mut self, transition: Transition) -> Result<State, TransitionError> {
        let next = match (self.state, transition) {
            (State::Unconfigured, Transition::Configure)
            | (State::Active, Transition::Deactivate) => State::Inactive,
            (State::Inactive, Transition::Cleanup) => State::Unconfigured,
            (State::Inactive, Transition::Activate) => State::Active,
            (State::Unconfigured | State::Inactive | State::Active, Transition::Shutdown) => {
                State::Finalized
            }
            _ => {
                return Err(TransitionError {
                    from: self.state,
                    transition,
                });
            }
        };
        self.state = next;
        Ok(next)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TransitionError {
    pub from: State,
    pub transition: Transition,
}

#[cfg(test)]
mod tests {
    use super::{
        change_state_request, get_state_response, ChangeStateRequest, ChangeStateResponse,
        GetStateRequest, GetStateResponse, Lifecycle, State, Transition,
    };

    #[test]
    fn lifecycle_accepts_standard_path() {
        let mut lifecycle = Lifecycle::new();
        assert_eq!(
            lifecycle.transition(Transition::Configure),
            Ok(State::Inactive)
        );
        assert_eq!(
            lifecycle.transition(Transition::Activate),
            Ok(State::Active)
        );
        assert_eq!(
            lifecycle.transition(Transition::Deactivate),
            Ok(State::Inactive)
        );
        assert_eq!(
            lifecycle.transition(Transition::Cleanup),
            Ok(State::Unconfigured)
        );
        assert_eq!(
            lifecycle.transition(Transition::Shutdown),
            Ok(State::Finalized)
        );
    }

    #[test]
    fn lifecycle_service_messages_round_trip() {
        let req = change_state_request(Transition::Activate);
        let mut req_back =
            ChangeStateRequest::from_cdr(&req.to_cdr(crate::msgs::Endian::Little)).unwrap();
        assert_eq!(
            Transition::from_id(req_back.transition.id),
            Some(Transition::Activate)
        );

        let resp = ChangeStateResponse { success: true };
        let resp_back =
            ChangeStateResponse::from_cdr(&resp.to_cdr(crate::msgs::Endian::Little)).unwrap();
        assert!(resp_back.success);

        let get = GetStateRequest {};
        let _get_back =
            GetStateRequest::from_cdr(&get.to_cdr(crate::msgs::Endian::Little)).unwrap();

        let state = get_state_response(State::Inactive);
        let mut state_back =
            GetStateResponse::from_cdr(&state.to_cdr(crate::msgs::Endian::Little)).unwrap();
        assert_eq!(State::from_id(state_back.current_state.id), State::Inactive);
        unsafe {
            // `req`/`state` borrow `'static` labels (nothing to free); only the
            // decoded (owned) values need finalizing.
            req_back.fini();
            state_back.fini();
        }
    }
}
