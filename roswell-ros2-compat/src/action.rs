//! Transport-independent ROS action goal state.

use std::collections::HashMap;

use crate::codec::{impl_cdr_msg, CdrMsg, CodecError};
use crate::msgs::{
    builtin_interfaces__Time, example_interfaces__Fibonacci_Feedback,
    example_interfaces__Fibonacci_FeedbackMessage, example_interfaces__Fibonacci_GetResult_Request,
    example_interfaces__Fibonacci_GetResult_Response, example_interfaces__Fibonacci_Goal,
    example_interfaces__Fibonacci_Result, example_interfaces__Fibonacci_SendGoal_Request,
    example_interfaces__Fibonacci_SendGoal_Response, unique_identifier_msgs__UUID, CdrError,
    Endian, Reader, RosSequence, Writer,
};
use crate::service::{Client, Service};
use crate::time::{Duration, Time};
use crate::transport::{Dds, DdsPub, DdsSub, MsgPublisher, MsgSubscriber, Qos, Transport};

/// ROS action service/topic names for one action.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ActionNames {
    pub send_goal: String,
    pub get_result: String,
    pub cancel_goal: String,
    pub feedback: String,
    pub status: String,
}

impl ActionNames {
    #[must_use]
    pub fn new(action_name: &str) -> Self {
        let base = action_name.trim_end_matches('/');
        Self {
            send_goal: format!("{base}/_action/send_goal"),
            get_result: format!("{base}/_action/get_result"),
            cancel_goal: format!("{base}/_action/cancel_goal"),
            feedback: format!("{base}/_action/feedback"),
            status: format!("{base}/_action/status"),
        }
    }
}

/// Goal identifier used by ROS actions.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct GoalId(pub [u8; 16]);

impl GoalId {
    #[must_use]
    pub const fn nil() -> Self {
        Self([0; 16])
    }

    /// Generate a fresh, non-nil goal id. ROS only requires uniqueness, not
    /// RFC-4122 formatting, so we mix the wall clock with a process-wide counter
    /// rather than pull in a UUID/RNG dependency.
    #[must_use]
    pub fn generate() -> Self {
        use std::sync::atomic::{AtomicU64, Ordering};
        static COUNTER: AtomicU64 = AtomicU64::new(0);
        let nanos = Time::now_system().as_nanos() as u64;
        let seq = COUNTER.fetch_add(1, Ordering::Relaxed).wrapping_add(1);
        let mut bytes = [0u8; 16];
        bytes[..8].copy_from_slice(&nanos.to_le_bytes());
        bytes[8..].copy_from_slice(&seq.to_le_bytes());
        Self(bytes)
    }
}

/// `action_msgs/msg/GoalStatus` status values.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(i8)]
pub enum GoalStatus {
    Unknown = 0,
    Accepted = 1,
    Executing = 2,
    Canceling = 3,
    Succeeded = 4,
    Canceled = 5,
    Aborted = 6,
}

impl GoalStatus {
    /// Decode a wire status byte, mapping unknown values to [`GoalStatus::Unknown`].
    #[must_use]
    pub const fn from_i8(value: i8) -> Self {
        match value {
            1 => GoalStatus::Accepted,
            2 => GoalStatus::Executing,
            3 => GoalStatus::Canceling,
            4 => GoalStatus::Succeeded,
            5 => GoalStatus::Canceled,
            6 => GoalStatus::Aborted,
            _ => GoalStatus::Unknown,
        }
    }

    #[must_use]
    pub const fn terminal(self) -> bool {
        matches!(
            self,
            GoalStatus::Succeeded | GoalStatus::Canceled | GoalStatus::Aborted
        )
    }
}

/// Generated `<Action>_SendGoal_Request` messages implement this.
pub trait SendGoalRequest {
    type Goal;

    fn goal_id(&self) -> GoalId;
    fn goal(&self) -> &Self::Goal;
}

/// Generated `<Action>_SendGoal_Response` messages implement this.
pub trait SendGoalResponse {
    fn new(accepted: bool, stamp: Time) -> Self;
}

/// Generated `<Action>_GetResult_Request` messages implement this.
pub trait GetResultRequest {
    fn goal_id(&self) -> GoalId;
}

/// Generated `<Action>_GetResult_Response` messages implement this.
pub trait GetResultResponse {
    type Result;

    fn new(status: GoalStatus, result: Self::Result) -> Self;
}

/// Generated `<Action>_FeedbackMessage` messages implement this.
pub trait FeedbackMessage {
    type Feedback;

    fn new(goal_id: GoalId, feedback: Self::Feedback) -> Self;
}

/// `action_msgs/msg/GoalInfo`.
#[repr(C)]
pub struct GoalInfoMsg {
    pub goal_id: GoalId,
    pub stamp: builtin_interfaces__Time,
}

impl GoalInfoMsg {
    #[must_use]
    pub fn new(goal_id: GoalId, stamp: Time) -> Self {
        Self {
            goal_id,
            stamp: stamp.to_msg(),
        }
    }

    fn serialize_into(&self, w: &mut Writer) {
        for byte in self.goal_id.0 {
            w.write_u8(byte);
        }
        self.stamp.serialize_into(w);
    }

    fn deserialize_from(r: &mut Reader<'_>) -> Result<Self, CdrError> {
        let mut uuid = [0; 16];
        for byte in &mut uuid {
            *byte = r.read_u8()?;
        }
        Ok(Self {
            goal_id: GoalId(uuid),
            stamp: builtin_interfaces__Time::deserialize_from(r)?,
        })
    }
}

/// `action_msgs/msg/GoalStatus`.
#[repr(C)]
pub struct GoalStatusMsg {
    pub goal_info: GoalInfoMsg,
    pub status: i8,
}

impl GoalStatusMsg {
    #[must_use]
    pub fn new(goal_id: GoalId, stamp: Time, status: GoalStatus) -> Self {
        Self {
            goal_info: GoalInfoMsg::new(goal_id, stamp),
            status: status as i8,
        }
    }

    fn serialize_into(&self, w: &mut Writer) {
        self.goal_info.serialize_into(w);
        w.write_i8(self.status);
    }

    fn deserialize_from(r: &mut Reader<'_>) -> Result<Self, CdrError> {
        Ok(Self {
            goal_info: GoalInfoMsg::deserialize_from(r)?,
            status: r.read_i8()?,
        })
    }
}

/// `action_msgs/msg/GoalStatusArray`.
#[repr(C)]
pub struct GoalStatusArrayMsg {
    pub status_list: RosSequence<GoalStatusMsg>,
}

impl GoalStatusArrayMsg {
    #[must_use]
    pub fn new(statuses: Vec<GoalStatusMsg>) -> Self {
        Self {
            status_list: RosSequence::alloc(statuses),
        }
    }

    fn serialize_into(&self, w: &mut Writer) {
        let statuses = self.status_list.as_slice();
        w.write_seq_len(statuses.len());
        for status in statuses {
            status.serialize_into(w);
        }
    }

    fn deserialize_from(r: &mut Reader<'_>) -> Result<Self, CdrError> {
        let len = r.read_seq_len()?;
        let statuses = (0..len)
            .map(|_| GoalStatusMsg::deserialize_from(r))
            .collect::<Result<Vec<_>, _>>()?;
        Ok(Self {
            status_list: RosSequence::alloc(statuses),
        })
    }

    /// # Safety
    /// Frees the owned status sequence; call at most once.
    pub unsafe fn fini(self) {
        drop(self.status_list.into_vec());
    }
}

impl_cdr_msg!(
    GoalStatusArrayMsg,
    "action_msgs::msg::dds_::GoalStatusArray_",
    "goal-status-array decode failed"
);

/// `action_msgs/srv/CancelGoal` request.
#[repr(C)]
pub struct CancelGoalRequest {
    pub goal_info: GoalInfoMsg,
}

impl CancelGoalRequest {
    fn serialize_into(&self, w: &mut Writer) {
        self.goal_info.serialize_into(w);
    }

    fn deserialize_from(r: &mut Reader<'_>) -> Result<Self, CdrError> {
        Ok(Self {
            goal_info: GoalInfoMsg::deserialize_from(r)?,
        })
    }
}

impl_cdr_msg!(
    CancelGoalRequest,
    "action_msgs::srv::dds_::CancelGoal_Request_",
    "cancel-goal request decode failed"
);

/// `action_msgs/srv/CancelGoal` response.
#[repr(C)]
pub struct CancelGoalResponse {
    pub return_code: i8,
    pub goals_canceling: RosSequence<GoalInfoMsg>,
}

impl CancelGoalResponse {
    pub const ERROR_NONE: i8 = 0;
    pub const ERROR_REJECTED: i8 = 1;
    pub const ERROR_UNKNOWN_GOAL_ID: i8 = 2;
    pub const ERROR_GOAL_TERMINATED: i8 = 3;

    #[must_use]
    pub fn empty(return_code: i8) -> Self {
        Self {
            return_code,
            goals_canceling: RosSequence::alloc(Vec::new()),
        }
    }

    #[must_use]
    pub fn with_goals(return_code: i8, goals_canceling: Vec<GoalInfoMsg>) -> Self {
        Self {
            return_code,
            goals_canceling: RosSequence::alloc(goals_canceling),
        }
    }

    fn serialize_into(&self, w: &mut Writer) {
        w.write_i8(self.return_code);
        let goals = self.goals_canceling.as_slice();
        w.write_seq_len(goals.len());
        for goal in goals {
            goal.serialize_into(w);
        }
    }

    fn deserialize_from(r: &mut Reader<'_>) -> Result<Self, CdrError> {
        let return_code = r.read_i8()?;
        let len = r.read_seq_len()?;
        let goals_canceling = (0..len)
            .map(|_| GoalInfoMsg::deserialize_from(r))
            .collect::<Result<Vec<_>, _>>()?;
        Ok(Self {
            return_code,
            goals_canceling: RosSequence::alloc(goals_canceling),
        })
    }

    /// # Safety
    /// Frees the owned goal sequence; call at most once.
    pub unsafe fn fini(self) {
        drop(self.goals_canceling.into_vec());
    }
}

impl_cdr_msg!(
    CancelGoalResponse,
    "action_msgs::srv::dds_::CancelGoal_Response_",
    "cancel-goal response decode failed"
);

#[derive(Clone, Debug)]
pub struct Goal<G, R = ()> {
    pub id: GoalId,
    pub accepted_at: Time,
    pub finished_at: Option<Time>,
    pub status: GoalStatus,
    pub goal: G,
    pub result: Option<R>,
}

/// Server-side action goal registry and transition policy.
#[derive(Clone, Debug, Default)]
pub struct ActionServerState<G, R = ()> {
    goals: HashMap<GoalId, Goal<G, R>>,
}

impl<G, R> ActionServerState<G, R> {
    #[must_use]
    pub fn new() -> Self {
        Self {
            goals: HashMap::new(),
        }
    }

    pub fn accept(&mut self, id: GoalId, accepted_at: Time, goal: G) -> Result<(), ActionError> {
        if self.goals.contains_key(&id) {
            return Err(ActionError::DuplicateGoal(id));
        }
        self.goals.insert(
            id,
            Goal {
                id,
                accepted_at,
                finished_at: None,
                status: GoalStatus::Accepted,
                goal,
                result: None,
            },
        );
        Ok(())
    }

    pub fn execute(&mut self, id: GoalId) -> Result<(), ActionError> {
        self.transition(id, &[GoalStatus::Accepted], GoalStatus::Executing)
    }

    pub fn request_cancel(&mut self, id: GoalId) -> Result<(), ActionError> {
        self.transition(
            id,
            &[GoalStatus::Accepted, GoalStatus::Executing],
            GoalStatus::Canceling,
        )
    }

    pub fn succeed(&mut self, id: GoalId, result: R) -> Result<(), ActionError> {
        self.finish(id, &[GoalStatus::Executing], GoalStatus::Succeeded, result)
    }

    pub fn abort(&mut self, id: GoalId, result: R) -> Result<(), ActionError> {
        self.finish(
            id,
            &[GoalStatus::Accepted, GoalStatus::Executing],
            GoalStatus::Aborted,
            result,
        )
    }

    pub fn cancel(&mut self, id: GoalId, result: R) -> Result<(), ActionError> {
        self.finish(id, &[GoalStatus::Canceling], GoalStatus::Canceled, result)
    }

    #[must_use]
    pub fn get(&self, id: GoalId) -> Option<&Goal<G, R>> {
        self.goals.get(&id)
    }

    #[must_use]
    pub fn result_available(&self, id: GoalId) -> bool {
        self.goals
            .get(&id)
            .is_some_and(|goal| goal.status.terminal() && goal.result.is_some())
    }

    pub fn prune_results(&mut self, now: Time, retention: Duration) -> usize {
        let before = self.goals.len();
        self.goals.retain(|_, goal| {
            !goal.status.terminal()
                || goal
                    .finished_at
                    .is_none_or(|finished_at| (now - finished_at) <= retention)
        });
        before - self.goals.len()
    }

    pub fn handle_cancel_request(
        &mut self,
        req: &CancelGoalRequest,
        now: Time,
    ) -> CancelGoalResponse {
        let requested = req.goal_info.goal_id;
        if requested == GoalId::nil() {
            let mut canceling = Vec::new();
            for (id, goal) in &mut self.goals {
                if matches!(goal.status, GoalStatus::Accepted | GoalStatus::Executing) {
                    goal.status = GoalStatus::Canceling;
                    canceling.push(GoalInfoMsg::new(*id, now));
                }
            }
            return CancelGoalResponse::with_goals(CancelGoalResponse::ERROR_NONE, canceling);
        }
        let Some(goal) = self.goals.get_mut(&requested) else {
            return CancelGoalResponse::empty(CancelGoalResponse::ERROR_UNKNOWN_GOAL_ID);
        };
        if goal.status.terminal() {
            return CancelGoalResponse::empty(CancelGoalResponse::ERROR_GOAL_TERMINATED);
        }
        if matches!(goal.status, GoalStatus::Accepted | GoalStatus::Executing) {
            goal.status = GoalStatus::Canceling;
        }
        CancelGoalResponse::with_goals(
            CancelGoalResponse::ERROR_NONE,
            vec![GoalInfoMsg::new(requested, now)],
        )
    }

    pub fn statuses(&self) -> impl Iterator<Item = (GoalId, GoalStatus)> + '_ {
        self.goals.iter().map(|(id, goal)| (*id, goal.status))
    }

    fn transition(
        &mut self,
        id: GoalId,
        allowed: &[GoalStatus],
        next: GoalStatus,
    ) -> Result<(), ActionError> {
        let goal = self
            .goals
            .get_mut(&id)
            .ok_or(ActionError::UnknownGoal(id))?;
        if !allowed.contains(&goal.status) {
            return Err(ActionError::InvalidTransition {
                id,
                from: goal.status,
                to: next,
            });
        }
        goal.status = next;
        Ok(())
    }

    fn finish(
        &mut self,
        id: GoalId,
        allowed: &[GoalStatus],
        next: GoalStatus,
        result: R,
    ) -> Result<(), ActionError> {
        self.transition(id, allowed, next)?;
        let goal = self
            .goals
            .get_mut(&id)
            .ok_or(ActionError::UnknownGoal(id))?;
        goal.result = Some(result);
        goal.finished_at = Some(Time::now_system());
        Ok(())
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ActionError {
    DuplicateGoal(GoalId),
    UnknownGoal(GoalId),
    InvalidTransition {
        id: GoalId,
        from: GoalStatus,
        to: GoalStatus,
    },
}

// ---------------------------------------------------------------------------
// Client side
// ---------------------------------------------------------------------------

/// A single feedback sample: which goal it belongs to and its payload.
pub struct FeedbackSample {
    pub goal_id: GoalId,
    pub feedback: example_interfaces__Fibonacci_Feedback,
}

/// Typed client for the demo `example_interfaces/action/Fibonacci` action.
///
/// Concrete on purpose: codegen emits only the server-direction traits, so a
/// generic typed client is unreachable for any other action — real clients go
/// through the dynamic FFI `RcmActionClient`. This one exists for the demo bin
/// and loopback tests.
///
/// Mirrors the server: the three services (`send_goal`, `get_result`,
/// `cancel_goal`) ride [`Client`] (request/reply correlated by RTPS sample
/// identity), and feedback/status are plain subscriptions. QoS matches what the
/// server offers — feedback is [`Qos::Default`] (reliable/volatile) and status
/// is [`Qos::Latched`] (transient-local), so a late-joining client still sees
/// the last status array.
pub struct FibonacciActionClient {
    send_goal: Client<
        example_interfaces__Fibonacci_SendGoal_Request,
        example_interfaces__Fibonacci_SendGoal_Response,
    >,
    get_result: Client<
        example_interfaces__Fibonacci_GetResult_Request,
        example_interfaces__Fibonacci_GetResult_Response,
    >,
    cancel_goal: Client<CancelGoalRequest, CancelGoalResponse>,
    feedback: DdsSub<example_interfaces__Fibonacci_FeedbackMessage>,
    status: DdsSub<GoalStatusArrayMsg>,
}

impl FibonacciActionClient {
    /// Bind a client to `action_name` on `dds`.
    #[must_use]
    pub fn new(dds: &Dds, action_name: &str) -> Self {
        let names = ActionNames::new(action_name);
        Self {
            send_goal: Client::new(dds, &names.send_goal),
            get_result: Client::new(dds, &names.get_result),
            cancel_goal: Client::new(dds, &names.cancel_goal),
            feedback: dds.subscriber(&names.feedback, Qos::Default),
            status: dds.subscriber::<GoalStatusArrayMsg>(&names.status, Qos::Latched),
        }
    }

    /// True once all three service clients (`send_goal`, `get_result`,
    /// `cancel_goal`) have discovered their server endpoints. Poll before the
    /// first `send_goal` instead of sleeping for discovery.
    pub fn server_is_ready(&mut self) -> bool {
        // No short-circuit: each call also drains that client's status queue.
        let sg = self.send_goal.server_is_ready();
        let gr = self.get_result.server_is_ready();
        let cg = self.cancel_goal.server_is_ready();
        sg && gr && cg
    }

    /// Send `goal` under a freshly generated id, blocking up to `timeout` for the
    /// server's accept/reject reply. Returns the goal id and whether it was
    /// accepted, or `None` on timeout.
    pub fn send_goal(
        &mut self,
        goal: example_interfaces__Fibonacci_Goal,
        timeout: std::time::Duration,
    ) -> Option<(GoalId, bool)> {
        let goal_id = GoalId::generate();
        let req = example_interfaces__Fibonacci_SendGoal_Request {
            goal_id: unique_identifier_msgs__UUID { uuid: goal_id.0 },
            goal,
        };
        let resp = self.send_goal.call(req, timeout)?;
        Some((goal_id, resp.accepted))
    }

    /// Drain all feedback samples received since the last poll. Non-blocking.
    pub fn poll_feedback(&mut self) -> Vec<FeedbackSample> {
        let mut out = Vec::new();
        while let Some(msg) = self.feedback.take() {
            out.push(FeedbackSample {
                goal_id: GoalId(msg.goal_id.uuid),
                feedback: msg.feedback,
            });
        }
        out
    }

    /// Take the most recent goal-status array, if any has arrived since the last
    /// poll. Non-blocking; returns the newest sample (older ones are discarded).
    pub fn poll_status(&mut self) -> Option<Vec<(GoalId, GoalStatus)>> {
        let mut latest = None;
        while let Some(msg) = self.status.take() {
            latest = Some(msg);
        }
        let msg = latest?;
        let statuses = msg
            .status_list
            .as_slice()
            .iter()
            .map(|s| (s.goal_info.goal_id, GoalStatus::from_i8(s.status)))
            .collect();
        // Safety: `msg` owns the sequence and is consumed exactly once here.
        unsafe { msg.fini() };
        Some(statuses)
    }

    /// Request the result for `goal_id`, blocking up to `timeout`. Returns the
    /// final status and result, or `None` on timeout.
    pub fn get_result(
        &mut self,
        goal_id: GoalId,
        timeout: std::time::Duration,
    ) -> Option<(GoalStatus, example_interfaces__Fibonacci_Result)> {
        let req = example_interfaces__Fibonacci_GetResult_Request {
            goal_id: unique_identifier_msgs__UUID { uuid: goal_id.0 },
        };
        let resp = self.get_result.call(req, timeout)?;
        Some((GoalStatus::from_i8(resp.status), resp.result))
    }

    /// Request cancellation of `goal_id` (or all goals when `goal_id` is
    /// [`GoalId::nil`]), blocking up to `timeout` for the server's reply.
    pub fn cancel_goal(
        &mut self,
        goal_id: GoalId,
        timeout: std::time::Duration,
    ) -> Option<CancelGoalResponse> {
        let req = CancelGoalRequest {
            goal_info: GoalInfoMsg::new(goal_id, Time::now_system()),
        };
        self.cancel_goal.call(req, timeout)
    }
}

// ---------------------------------------------------------------------------
// Server side (transport-bound)
// ---------------------------------------------------------------------------

/// Server for one ROS2 action, generic over its five wire types — the mirror
/// image of [`ActionClient`], using the server-direction traits
/// ([`SendGoalRequest`], [`SendGoalResponse`], [`GetResultRequest`],
/// [`GetResultResponse`], [`FeedbackMessage`]) the compiler emits per action.
///
/// The three services ride [`Service`]; feedback is a [`Qos::Default`]
/// publisher and status a [`Qos::Latched`] one, matching what a vanilla client
/// (`ros2 action send_goal`) expects. Goal bookkeeping and the cancel policy
/// live in an internal [`ActionServerState`]; the caller owns the payloads
/// (goal parameters, result computation) via the serve-handler closures, so no
/// `Clone` bounds are forced onto generated types.
pub struct ActionServer<SgReq, SgResp, GrReq, GrResp, Fb>
where
    SgReq: CdrMsg + SendGoalRequest,
    SgResp: CdrMsg + SendGoalResponse,
    GrReq: CdrMsg + GetResultRequest,
    GrResp: CdrMsg + GetResultResponse,
    Fb: CdrMsg + FeedbackMessage,
{
    send_goal: Service<SgReq, SgResp>,
    get_result: Service<GrReq, GrResp>,
    cancel_goal: Service<CancelGoalRequest, CancelGoalResponse>,
    feedback: DdsPub<Fb>,
    status: DdsPub<GoalStatusArrayMsg>,
    state: ActionServerState<(), ()>,
}

impl<SgReq, SgResp, GrReq, GrResp, Fb> ActionServer<SgReq, SgResp, GrReq, GrResp, Fb>
where
    SgReq: CdrMsg + SendGoalRequest,
    SgResp: CdrMsg + SendGoalResponse,
    GrReq: CdrMsg + GetResultRequest,
    GrResp: CdrMsg + GetResultResponse,
    Fb: CdrMsg + FeedbackMessage,
{
    /// Bind a server to `action_name` on `dds`.
    #[must_use]
    pub fn new(dds: &Dds, action_name: &str) -> Self {
        let names = ActionNames::new(action_name);
        Self {
            send_goal: Service::new(dds, &names.send_goal),
            get_result: Service::new(dds, &names.get_result),
            cancel_goal: Service::new(dds, &names.cancel_goal),
            feedback: dds.publisher::<Fb>(&names.feedback, Qos::Default),
            status: dds.publisher::<GoalStatusArrayMsg>(&names.status, Qos::Latched),
            state: ActionServerState::new(),
        }
    }

    /// Serve every pending `send_goal` request. `on_goal(goal_id, goal)`
    /// decides acceptance; accepted goals are registered and moved straight to
    /// `Executing`. Returns the number of requests served.
    pub fn serve_goals(&mut self, mut on_goal: impl FnMut(GoalId, &SgReq::Goal) -> bool) -> usize {
        let state = &mut self.state;
        self.send_goal.serve_pending(|req| {
            let id = req.goal_id();
            let now = Time::now_system();
            let accepted = on_goal(id, req.goal());
            if accepted && state.accept(id, now, ()).is_ok() {
                let _ = state.execute(id);
            }
            SgResp::new(accepted, now)
        })
    }

    /// Serve every pending `cancel_goal` request with the standard policy:
    /// live goals move to `Canceling`, terminal goals report
    /// `ERROR_GOAL_TERMINATED`, unknown ids `ERROR_UNKNOWN_GOAL_ID` (see
    /// [`ActionServerState::handle_cancel_request`]). Returns requests served.
    pub fn serve_cancels(&mut self) -> usize {
        let state = &mut self.state;
        self.cancel_goal
            .serve_pending(|req| state.handle_cancel_request(req, Time::now_system()))
    }

    /// Serve every pending `get_result` request. `on_result(goal_id)` supplies
    /// the terminal status and result payload; the goal's registered status is
    /// advanced to match (invalid transitions — e.g. an unknown id — are left
    /// as-is, exactly like replying about a goal the server never accepted).
    /// Returns the number of requests served.
    pub fn serve_results(
        &mut self,
        mut on_result: impl FnMut(GoalId) -> (GoalStatus, GrResp::Result),
    ) -> usize {
        let state = &mut self.state;
        self.get_result.serve_pending(|req| {
            let id = req.goal_id();
            let (status, result) = on_result(id);
            let _ = match status {
                GoalStatus::Succeeded => state.succeed(id, ()),
                GoalStatus::Aborted => state.abort(id, ()),
                GoalStatus::Canceled => state.cancel(id, ()),
                _ => Ok(()),
            };
            GrResp::new(status, result)
        })
    }

    /// Publish one feedback message for `goal_id`.
    pub fn publish_feedback(&self, goal_id: GoalId, feedback: Fb::Feedback) {
        self.feedback.publish(Fb::new(goal_id, feedback));
    }

    /// Publish the latched status array for every registered goal.
    pub fn publish_status(&self) {
        let now = Time::now_system();
        let statuses = self
            .state
            .statuses()
            .map(|(id, status)| GoalStatusMsg::new(id, now, status))
            .collect();
        self.status.publish(GoalStatusArrayMsg::new(statuses));
    }

    /// Goal ids currently in `Canceling`, for handlers that honor cancellation.
    pub fn canceling(&self) -> Vec<GoalId> {
        self.state
            .statuses()
            .filter(|(_, status)| *status == GoalStatus::Canceling)
            .map(|(id, _)| id)
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::{
        ActionError, ActionServerState, CancelGoalRequest, CancelGoalResponse, GoalId, GoalInfoMsg,
        GoalStatus,
    };
    use crate::time::{Duration, Time};

    #[test]
    fn action_goal_standard_success_path() {
        let id = GoalId([1; 16]);
        let mut state = ActionServerState::new();
        state.accept(id, Time::from_secs(1), "goal").unwrap();
        assert_eq!(state.get(id).unwrap().status, GoalStatus::Accepted);
        state.execute(id).unwrap();
        state.succeed(id, "done").unwrap();
        let goal = state.get(id).unwrap();
        assert_eq!(goal.status, GoalStatus::Succeeded);
        assert_eq!(goal.result, Some("done"));
    }

    #[test]
    fn action_rejects_invalid_transition() {
        let id = GoalId([2; 16]);
        let mut state = ActionServerState::new();
        state.accept(id, Time::from_secs(1), ()).unwrap();
        assert!(matches!(
            state.succeed(id, ()),
            Err(ActionError::InvalidTransition { .. })
        ));
    }

    #[test]
    fn action_names_match_ros_action_contract() {
        let names = super::ActionNames::new("/navigate_to_pose");
        assert_eq!(names.send_goal, "/navigate_to_pose/_action/send_goal");
        assert_eq!(names.get_result, "/navigate_to_pose/_action/get_result");
        assert_eq!(names.cancel_goal, "/navigate_to_pose/_action/cancel_goal");
        assert_eq!(names.feedback, "/navigate_to_pose/_action/feedback");
        assert_eq!(names.status, "/navigate_to_pose/_action/status");
    }

    #[test]
    fn action_cancel_request_moves_goal_to_canceling() {
        let id = GoalId([3; 16]);
        let mut state = ActionServerState::<_, ()>::new();
        state.accept(id, Time::from_secs(1), "goal").unwrap();
        state.execute(id).unwrap();
        let resp = state.handle_cancel_request(
            &CancelGoalRequest {
                goal_info: GoalInfoMsg::new(id, Time::from_secs(1)),
            },
            Time::from_secs(2),
        );
        assert_eq!(resp.return_code, CancelGoalResponse::ERROR_NONE);
        assert_eq!(state.get(id).unwrap().status, GoalStatus::Canceling);
    }

    #[test]
    fn action_prunes_expired_terminal_results() {
        let id = GoalId([4; 16]);
        let mut state = ActionServerState::new();
        state.accept(id, Time::from_secs(1), ()).unwrap();
        state.execute(id).unwrap();
        state.succeed(id, ()).unwrap();
        state.goals.get_mut(&id).unwrap().finished_at = Some(Time::from_secs(10));
        assert_eq!(
            state.prune_results(Time::from_secs(20), Duration::from_secs(5)),
            1
        );
        assert!(state.get(id).is_none());
    }
}
