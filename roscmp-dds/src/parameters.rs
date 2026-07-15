//! ROS2 parameter services and `/parameter_events`.
#![deny(unsafe_code)]

use std::collections::{BTreeMap, BTreeSet};
use std::time::Duration;

use crate::codec::{CdrMsg, CodecError};
use crate::msgs::{
    builtin_interfaces__Time,
    rcl_interfaces__DescribeParameters_Request as DescribeParametersRequestGen,
    rcl_interfaces__DescribeParameters_Response as DescribeParametersResponseGen,
    rcl_interfaces__FloatingPointRange as FloatingPointRangeGen,
    rcl_interfaces__GetParameterTypes_Request as GetParameterTypesRequestGen,
    rcl_interfaces__GetParameterTypes_Response as GetParameterTypesResponseGen,
    rcl_interfaces__GetParameters_Request as GetParametersRequestGen,
    rcl_interfaces__GetParameters_Response as GetParametersResponseGen,
    rcl_interfaces__IntegerRange as IntegerRangeGen,
    rcl_interfaces__ListParametersResult as ListParametersResultGen,
    rcl_interfaces__ListParameters_Request as ListParametersRequestGen,
    rcl_interfaces__ListParameters_Response as ListParametersResponseGen,
    rcl_interfaces__Parameter as ParameterGen,
    rcl_interfaces__ParameterDescriptor as ParameterDescriptorGen,
    rcl_interfaces__ParameterEvent as ParameterEventGen,
    rcl_interfaces__ParameterValue as ParameterValueGen,
    rcl_interfaces__SetParametersAtomically_Request as SetParametersAtomicallyRequestGen,
    rcl_interfaces__SetParametersAtomically_Response as SetParametersAtomicallyResponseGen,
    rcl_interfaces__SetParametersResult as SetParametersResultGen,
    rcl_interfaces__SetParameters_Request as SetParametersRequestGen,
    rcl_interfaces__SetParameters_Response as SetParametersResponseGen, RosSequence, RosString,
};
use crate::service::{Client, Service};
use crate::time::Time;
use crate::transport::{Dds, MsgPublisher, Qos, Transport};

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[repr(u8)]
pub enum ParameterType {
    #[default]
    NotSet = 0,
    Bool = 1,
    Integer = 2,
    Double = 3,
    String = 4,
    ByteArray = 5,
    BoolArray = 6,
    IntegerArray = 7,
    DoubleArray = 8,
    StringArray = 9,
}

impl ParameterType {
    #[must_use]
    pub const fn from_u8(value: u8) -> Self {
        match value {
            1 => Self::Bool,
            2 => Self::Integer,
            3 => Self::Double,
            4 => Self::String,
            5 => Self::ByteArray,
            6 => Self::BoolArray,
            7 => Self::IntegerArray,
            8 => Self::DoubleArray,
            9 => Self::StringArray,
            _ => Self::NotSet,
        }
    }
}

#[derive(Clone, Debug, Default, PartialEq)]
pub enum ParameterValue {
    #[default]
    NotSet,
    Bool(bool),
    Integer(i64),
    Double(f64),
    String(String),
    ByteArray(Vec<u8>),
    BoolArray(Vec<bool>),
    IntegerArray(Vec<i64>),
    DoubleArray(Vec<f64>),
    StringArray(Vec<String>),
}

impl ParameterValue {
    #[must_use]
    pub const fn parameter_type(&self) -> ParameterType {
        match self {
            Self::NotSet => ParameterType::NotSet,
            Self::Bool(_) => ParameterType::Bool,
            Self::Integer(_) => ParameterType::Integer,
            Self::Double(_) => ParameterType::Double,
            Self::String(_) => ParameterType::String,
            Self::ByteArray(_) => ParameterType::ByteArray,
            Self::BoolArray(_) => ParameterType::BoolArray,
            Self::IntegerArray(_) => ParameterType::IntegerArray,
            Self::DoubleArray(_) => ParameterType::DoubleArray,
            Self::StringArray(_) => ParameterType::StringArray,
        }
    }

    fn to_gen(&self) -> ParameterValueGen {
        // Every field is present on the wire (a tagged union); non-active fields
        // carry their defaults, exactly as the hand-rolled encoder did.
        ParameterValueGen {
            r#type: self.parameter_type() as u8,
            bool_value: matches!(self, Self::Bool(true)),
            integer_value: match self {
                Self::Integer(v) => *v,
                _ => 0,
            },
            double_value: match self {
                Self::Double(v) => *v,
                _ => 0.0,
            },
            string_value: RosString::alloc(match self {
                Self::String(v) => v,
                _ => "",
            }),
            byte_array_value: RosSequence::alloc(match self {
                Self::ByteArray(v) => v.clone(),
                _ => Vec::new(),
            }),
            bool_array_value: RosSequence::alloc(match self {
                Self::BoolArray(v) => v.clone(),
                _ => Vec::new(),
            }),
            integer_array_value: RosSequence::alloc(match self {
                Self::IntegerArray(v) => v.clone(),
                _ => Vec::new(),
            }),
            double_array_value: RosSequence::alloc(match self {
                Self::DoubleArray(v) => v.clone(),
                _ => Vec::new(),
            }),
            string_array_value: string_seq(match self {
                Self::StringArray(v) => v,
                _ => &[],
            }),
        }
    }

    fn from_gen(g: &ParameterValueGen) -> Self {
        match ParameterType::from_u8(g.r#type) {
            ParameterType::NotSet => Self::NotSet,
            ParameterType::Bool => Self::Bool(g.bool_value),
            ParameterType::Integer => Self::Integer(g.integer_value),
            ParameterType::Double => Self::Double(g.double_value),
            ParameterType::String => Self::String(g.string_value.as_str().to_string()),
            ParameterType::ByteArray => Self::ByteArray(g.byte_array_value.as_slice().to_vec()),
            ParameterType::BoolArray => Self::BoolArray(g.bool_array_value.as_slice().to_vec()),
            ParameterType::IntegerArray => {
                Self::IntegerArray(g.integer_array_value.as_slice().to_vec())
            }
            ParameterType::DoubleArray => {
                Self::DoubleArray(g.double_array_value.as_slice().to_vec())
            }
            ParameterType::StringArray => Self::StringArray(string_seq_from(&g.string_array_value)),
        }
    }
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct Parameter {
    pub name: String,
    pub value: ParameterValue,
}

impl Parameter {
    #[must_use]
    pub fn new(name: impl Into<String>, value: ParameterValue) -> Self {
        Self {
            name: name.into(),
            value,
        }
    }

    fn to_gen(&self) -> ParameterGen {
        ParameterGen {
            name: RosString::alloc(&self.name),
            value: self.value.to_gen(),
        }
    }

    fn from_gen(g: &ParameterGen) -> Self {
        Self {
            name: g.name.as_str().to_string(),
            value: ParameterValue::from_gen(&g.value),
        }
    }
}

#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct SetParametersResult {
    pub successful: bool,
    pub reason: String,
}

impl SetParametersResult {
    #[must_use]
    pub fn ok() -> Self {
        Self {
            successful: true,
            reason: String::new(),
        }
    }

    #[must_use]
    pub fn failed(reason: impl Into<String>) -> Self {
        Self {
            successful: false,
            reason: reason.into(),
        }
    }

    fn to_gen(&self) -> SetParametersResultGen {
        SetParametersResultGen {
            successful: self.successful,
            reason: RosString::alloc(&self.reason),
        }
    }

    fn from_gen(g: &SetParametersResultGen) -> Self {
        Self {
            successful: g.successful,
            reason: g.reason.as_str().to_string(),
        }
    }
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct IntegerRange {
    pub from_value: i64,
    pub to_value: i64,
    pub step: u64,
}

impl IntegerRange {
    fn to_gen(&self) -> IntegerRangeGen {
        IntegerRangeGen {
            from_value: self.from_value,
            to_value: self.to_value,
            step: self.step,
        }
    }

    fn from_gen(g: &IntegerRangeGen) -> Self {
        Self {
            from_value: g.from_value,
            to_value: g.to_value,
            step: g.step,
        }
    }
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct FloatingPointRange {
    pub from_value: f64,
    pub to_value: f64,
    pub step: f64,
}

impl FloatingPointRange {
    fn to_gen(&self) -> FloatingPointRangeGen {
        FloatingPointRangeGen {
            from_value: self.from_value,
            to_value: self.to_value,
            step: self.step,
        }
    }

    fn from_gen(g: &FloatingPointRangeGen) -> Self {
        Self {
            from_value: g.from_value,
            to_value: g.to_value,
            step: g.step,
        }
    }
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct ParameterDescriptor {
    pub name: String,
    pub parameter_type: ParameterType,
    pub description: String,
    pub additional_constraints: String,
    pub read_only: bool,
    pub dynamic_typing: bool,
    pub floating_point_range: Vec<FloatingPointRange>,
    pub integer_range: Vec<IntegerRange>,
}

impl ParameterDescriptor {
    #[must_use]
    pub fn new(name: impl Into<String>, parameter_type: ParameterType) -> Self {
        Self {
            name: name.into(),
            parameter_type,
            ..Self::default()
        }
    }

    fn to_gen(&self) -> ParameterDescriptorGen {
        ParameterDescriptorGen {
            name: RosString::alloc(&self.name),
            r#type: self.parameter_type as u8,
            description: RosString::alloc(&self.description),
            additional_constraints: RosString::alloc(&self.additional_constraints),
            read_only: self.read_only,
            dynamic_typing: self.dynamic_typing,
            floating_point_range: RosSequence::alloc(
                self.floating_point_range
                    .iter()
                    .map(FloatingPointRange::to_gen)
                    .collect(),
            ),
            integer_range: RosSequence::alloc(
                self.integer_range
                    .iter()
                    .map(IntegerRange::to_gen)
                    .collect(),
            ),
        }
    }

    fn from_gen(g: &ParameterDescriptorGen) -> Self {
        Self {
            name: g.name.as_str().to_string(),
            parameter_type: ParameterType::from_u8(g.r#type),
            description: g.description.as_str().to_string(),
            additional_constraints: g.additional_constraints.as_str().to_string(),
            read_only: g.read_only,
            dynamic_typing: g.dynamic_typing,
            floating_point_range: g
                .floating_point_range
                .as_slice()
                .iter()
                .map(FloatingPointRange::from_gen)
                .collect(),
            integer_range: g
                .integer_range
                .as_slice()
                .iter()
                .map(IntegerRange::from_gen)
                .collect(),
        }
    }
}

pub struct ParameterEvent {
    pub stamp: builtin_interfaces__Time,
    pub node: String,
    pub new_parameters: Vec<Parameter>,
    pub changed_parameters: Vec<Parameter>,
    pub deleted_parameters: Vec<Parameter>,
}

impl ParameterEvent {
    fn to_gen(&self) -> ParameterEventGen {
        ParameterEventGen {
            stamp: builtin_interfaces__Time {
                sec: self.stamp.sec,
                nanosec: self.stamp.nanosec,
            },
            node: RosString::alloc(&self.node),
            new_parameters: param_seq(&self.new_parameters),
            changed_parameters: param_seq(&self.changed_parameters),
            deleted_parameters: param_seq(&self.deleted_parameters),
        }
    }

    fn from_gen(g: &ParameterEventGen) -> Self {
        Self {
            stamp: builtin_interfaces__Time {
                sec: g.stamp.sec,
                nanosec: g.stamp.nanosec,
            },
            node: g.node.as_str().to_string(),
            new_parameters: param_seq_from(&g.new_parameters),
            changed_parameters: param_seq_from(&g.changed_parameters),
            deleted_parameters: param_seq_from(&g.deleted_parameters),
        }
    }
}

impl Clone for ParameterEvent {
    fn clone(&self) -> Self {
        Self {
            stamp: builtin_interfaces__Time {
                sec: self.stamp.sec,
                nanosec: self.stamp.nanosec,
            },
            node: self.node.clone(),
            new_parameters: self.new_parameters.clone(),
            changed_parameters: self.changed_parameters.clone(),
            deleted_parameters: self.deleted_parameters.clone(),
        }
    }
}

impl Default for ParameterEvent {
    fn default() -> Self {
        Self {
            stamp: builtin_interfaces__Time { sec: 0, nanosec: 0 },
            node: String::new(),
            new_parameters: Vec::new(),
            changed_parameters: Vec::new(),
            deleted_parameters: Vec::new(),
        }
    }
}

impl PartialEq for ParameterEvent {
    fn eq(&self, other: &Self) -> bool {
        self.stamp.sec == other.stamp.sec
            && self.stamp.nanosec == other.stamp.nanosec
            && self.node == other.node
            && self.new_parameters == other.new_parameters
            && self.changed_parameters == other.changed_parameters
            && self.deleted_parameters == other.deleted_parameters
    }
}

impl std::fmt::Debug for ParameterEvent {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("ParameterEvent")
            .field("stamp_sec", &self.stamp.sec)
            .field("stamp_nanosec", &self.stamp.nanosec)
            .field("node", &self.node)
            .field("new_parameters", &self.new_parameters)
            .field("changed_parameters", &self.changed_parameters)
            .field("deleted_parameters", &self.deleted_parameters)
            .finish()
    }
}

#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct GetParametersRequest {
    pub names: Vec<String>,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct GetParametersResponse {
    pub values: Vec<ParameterValue>,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct SetParametersRequest {
    pub parameters: Vec<Parameter>,
}

#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct SetParametersResponse {
    pub results: Vec<SetParametersResult>,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct SetParametersAtomicallyRequest {
    pub parameters: Vec<Parameter>,
}

#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct SetParametersAtomicallyResponse {
    pub result: SetParametersResult,
}

#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct GetParameterTypesRequest {
    pub names: Vec<String>,
}

#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct GetParameterTypesResponse {
    pub types: Vec<u8>,
}

#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct DescribeParametersRequest {
    pub names: Vec<String>,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct DescribeParametersResponse {
    pub descriptors: Vec<ParameterDescriptor>,
}

#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct ListParametersRequest {
    pub prefixes: Vec<String>,
    pub depth: u64,
}

#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct ListParametersResult {
    pub names: Vec<String>,
    pub prefixes: Vec<String>,
}

#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct ListParametersResponse {
    pub result: ListParametersResult,
}

// ---- generated-codec boundary -------------------------------------------
//
// The wire (de)serialization is the generated `rcl_interfaces` codec. Each
// ergonomic type provides `to_gen`/`from_gen` converting to/from the C-ABI
// generated struct; `cdr_via_gen!` derives the `CdrMsg` impl (build owned
// generated ⇒ encode ⇒ `fini`, so nothing leaks) shared by every DDS-visible
// request/response and `ParameterEvent`.

fn string_seq(v: &[String]) -> RosSequence<RosString> {
    RosSequence::alloc(v.iter().map(|s| RosString::alloc(s)).collect())
}

fn string_seq_from(s: &RosSequence<RosString>) -> Vec<String> {
    s.as_slice()
        .iter()
        .map(|x| x.as_str().to_string())
        .collect()
}

fn param_seq(v: &[Parameter]) -> RosSequence<ParameterGen> {
    RosSequence::alloc(v.iter().map(Parameter::to_gen).collect())
}

fn param_seq_from(s: &RosSequence<ParameterGen>) -> Vec<Parameter> {
    s.as_slice().iter().map(Parameter::from_gen).collect()
}

impl GetParametersRequest {
    fn to_gen(&self) -> GetParametersRequestGen {
        GetParametersRequestGen {
            names: string_seq(&self.names),
        }
    }
    fn from_gen(g: &GetParametersRequestGen) -> Self {
        Self {
            names: string_seq_from(&g.names),
        }
    }
}

impl GetParametersResponse {
    fn to_gen(&self) -> GetParametersResponseGen {
        GetParametersResponseGen {
            values: RosSequence::alloc(self.values.iter().map(ParameterValue::to_gen).collect()),
        }
    }
    fn from_gen(g: &GetParametersResponseGen) -> Self {
        Self {
            values: g
                .values
                .as_slice()
                .iter()
                .map(ParameterValue::from_gen)
                .collect(),
        }
    }
}

impl SetParametersRequest {
    fn to_gen(&self) -> SetParametersRequestGen {
        SetParametersRequestGen {
            parameters: param_seq(&self.parameters),
        }
    }
    fn from_gen(g: &SetParametersRequestGen) -> Self {
        Self {
            parameters: param_seq_from(&g.parameters),
        }
    }
}

impl SetParametersResponse {
    fn to_gen(&self) -> SetParametersResponseGen {
        SetParametersResponseGen {
            results: RosSequence::alloc(
                self.results
                    .iter()
                    .map(SetParametersResult::to_gen)
                    .collect(),
            ),
        }
    }
    fn from_gen(g: &SetParametersResponseGen) -> Self {
        Self {
            results: g
                .results
                .as_slice()
                .iter()
                .map(SetParametersResult::from_gen)
                .collect(),
        }
    }
}

impl SetParametersAtomicallyRequest {
    fn to_gen(&self) -> SetParametersAtomicallyRequestGen {
        SetParametersAtomicallyRequestGen {
            parameters: param_seq(&self.parameters),
        }
    }
    fn from_gen(g: &SetParametersAtomicallyRequestGen) -> Self {
        Self {
            parameters: param_seq_from(&g.parameters),
        }
    }
}

impl SetParametersAtomicallyResponse {
    fn to_gen(&self) -> SetParametersAtomicallyResponseGen {
        SetParametersAtomicallyResponseGen {
            result: self.result.to_gen(),
        }
    }
    fn from_gen(g: &SetParametersAtomicallyResponseGen) -> Self {
        Self {
            result: SetParametersResult::from_gen(&g.result),
        }
    }
}

impl GetParameterTypesRequest {
    fn to_gen(&self) -> GetParameterTypesRequestGen {
        GetParameterTypesRequestGen {
            names: string_seq(&self.names),
        }
    }
    fn from_gen(g: &GetParameterTypesRequestGen) -> Self {
        Self {
            names: string_seq_from(&g.names),
        }
    }
}

impl GetParameterTypesResponse {
    fn to_gen(&self) -> GetParameterTypesResponseGen {
        GetParameterTypesResponseGen {
            types: RosSequence::alloc(self.types.clone()),
        }
    }
    fn from_gen(g: &GetParameterTypesResponseGen) -> Self {
        Self {
            types: g.types.as_slice().to_vec(),
        }
    }
}

impl DescribeParametersRequest {
    fn to_gen(&self) -> DescribeParametersRequestGen {
        DescribeParametersRequestGen {
            names: string_seq(&self.names),
        }
    }
    fn from_gen(g: &DescribeParametersRequestGen) -> Self {
        Self {
            names: string_seq_from(&g.names),
        }
    }
}

impl DescribeParametersResponse {
    fn to_gen(&self) -> DescribeParametersResponseGen {
        DescribeParametersResponseGen {
            descriptors: RosSequence::alloc(
                self.descriptors
                    .iter()
                    .map(ParameterDescriptor::to_gen)
                    .collect(),
            ),
        }
    }
    fn from_gen(g: &DescribeParametersResponseGen) -> Self {
        Self {
            descriptors: g
                .descriptors
                .as_slice()
                .iter()
                .map(ParameterDescriptor::from_gen)
                .collect(),
        }
    }
}

impl ListParametersRequest {
    fn to_gen(&self) -> ListParametersRequestGen {
        ListParametersRequestGen {
            prefixes: string_seq(&self.prefixes),
            depth: self.depth,
        }
    }
    fn from_gen(g: &ListParametersRequestGen) -> Self {
        Self {
            prefixes: string_seq_from(&g.prefixes),
            depth: g.depth,
        }
    }
}

impl ListParametersResult {
    fn to_gen(&self) -> ListParametersResultGen {
        ListParametersResultGen {
            names: string_seq(&self.names),
            prefixes: string_seq(&self.prefixes),
        }
    }
    fn from_gen(g: &ListParametersResultGen) -> Self {
        Self {
            names: string_seq_from(&g.names),
            prefixes: string_seq_from(&g.prefixes),
        }
    }
}

impl ListParametersResponse {
    fn to_gen(&self) -> ListParametersResponseGen {
        ListParametersResponseGen {
            result: self.result.to_gen(),
        }
    }
    fn from_gen(g: &ListParametersResponseGen) -> Self {
        Self {
            result: ListParametersResult::from_gen(&g.result),
        }
    }
}

macro_rules! cdr_via_gen {
    ($erg:ty, $gen:ty, $type_name:expr, $err:expr) => {
        #[allow(unsafe_code)]
        impl CdrMsg for $erg {
            const TYPE_NAME: &'static str = $type_name;

            fn encode(&self) -> Vec<u8> {
                let mut g = self.to_gen();
                let bytes = g.encode();
                // SAFETY: `g` is a freshly-built owned value, finalized once.
                unsafe { g.fini() };
                bytes
            }

            fn decode(buf: &[u8]) -> Result<Self, CodecError> {
                let mut g = <$gen>::decode(buf).map_err(|_| CodecError($err))?;
                let out = Self::from_gen(&g);
                // SAFETY: `g` was decoded (owned) and is finalized once.
                unsafe { g.fini() };
                Ok(out)
            }
        }
    };
}

cdr_via_gen!(
    ParameterEvent,
    ParameterEventGen,
    "rcl_interfaces::msg::dds_::ParameterEvent_",
    "parameter event decode failed"
);
cdr_via_gen!(
    GetParametersRequest,
    GetParametersRequestGen,
    "rcl_interfaces::srv::dds_::GetParameters_Request_",
    "get-parameters request decode failed"
);
cdr_via_gen!(
    GetParametersResponse,
    GetParametersResponseGen,
    "rcl_interfaces::srv::dds_::GetParameters_Response_",
    "get-parameters response decode failed"
);
cdr_via_gen!(
    SetParametersRequest,
    SetParametersRequestGen,
    "rcl_interfaces::srv::dds_::SetParameters_Request_",
    "set-parameters request decode failed"
);
cdr_via_gen!(
    SetParametersResponse,
    SetParametersResponseGen,
    "rcl_interfaces::srv::dds_::SetParameters_Response_",
    "set-parameters response decode failed"
);
cdr_via_gen!(
    SetParametersAtomicallyRequest,
    SetParametersAtomicallyRequestGen,
    "rcl_interfaces::srv::dds_::SetParametersAtomically_Request_",
    "set-parameters-atomically request decode failed"
);
cdr_via_gen!(
    SetParametersAtomicallyResponse,
    SetParametersAtomicallyResponseGen,
    "rcl_interfaces::srv::dds_::SetParametersAtomically_Response_",
    "set-parameters-atomically response decode failed"
);
cdr_via_gen!(
    GetParameterTypesRequest,
    GetParameterTypesRequestGen,
    "rcl_interfaces::srv::dds_::GetParameterTypes_Request_",
    "get-parameter-types request decode failed"
);
cdr_via_gen!(
    GetParameterTypesResponse,
    GetParameterTypesResponseGen,
    "rcl_interfaces::srv::dds_::GetParameterTypes_Response_",
    "get-parameter-types response decode failed"
);
cdr_via_gen!(
    DescribeParametersRequest,
    DescribeParametersRequestGen,
    "rcl_interfaces::srv::dds_::DescribeParameters_Request_",
    "describe-parameters request decode failed"
);
cdr_via_gen!(
    DescribeParametersResponse,
    DescribeParametersResponseGen,
    "rcl_interfaces::srv::dds_::DescribeParameters_Response_",
    "describe-parameters response decode failed"
);
cdr_via_gen!(
    ListParametersRequest,
    ListParametersRequestGen,
    "rcl_interfaces::srv::dds_::ListParameters_Request_",
    "list-parameters request decode failed"
);
cdr_via_gen!(
    ListParametersResponse,
    ListParametersResponseGen,
    "rcl_interfaces::srv::dds_::ListParameters_Response_",
    "list-parameters response decode failed"
);

pub struct ParameterServer<P: MsgPublisher<ParameterEvent>> {
    node: String,
    values: BTreeMap<String, ParameterValue>,
    descriptors: BTreeMap<String, ParameterDescriptor>,
    events: P,
    get_parameters: Service<GetParametersRequest, GetParametersResponse>,
    set_parameters: Service<SetParametersRequest, SetParametersResponse>,
    set_parameters_atomically:
        Service<SetParametersAtomicallyRequest, SetParametersAtomicallyResponse>,
    get_parameter_types: Service<GetParameterTypesRequest, GetParameterTypesResponse>,
    describe_parameters: Service<DescribeParametersRequest, DescribeParametersResponse>,
    list_parameters: Service<ListParametersRequest, ListParametersResponse>,
}

impl ParameterServer<crate::transport::DdsPub<ParameterEvent>> {
    #[must_use]
    pub fn new(dds: &Dds, node: &str) -> Self {
        let service_prefix = format!("/{node}");
        Self {
            node: service_prefix.clone(),
            values: BTreeMap::new(),
            descriptors: BTreeMap::new(),
            events: dds.publisher::<ParameterEvent>("/parameter_events", Qos::Default),
            get_parameters: Service::new(dds, &format!("{service_prefix}/get_parameters")),
            set_parameters: Service::new(dds, &format!("{service_prefix}/set_parameters")),
            set_parameters_atomically: Service::new(
                dds,
                &format!("{service_prefix}/set_parameters_atomically"),
            ),
            get_parameter_types: Service::new(
                dds,
                &format!("{service_prefix}/get_parameter_types"),
            ),
            describe_parameters: Service::new(
                dds,
                &format!("{service_prefix}/describe_parameters"),
            ),
            list_parameters: Service::new(dds, &format!("{service_prefix}/list_parameters")),
        }
    }
}

impl<P: MsgPublisher<ParameterEvent>> ParameterServer<P> {
    pub fn declare(&mut self, descriptor: ParameterDescriptor, value: ParameterValue) {
        let name = descriptor.name.clone();
        self.values.insert(name.clone(), value.clone());
        self.descriptors.insert(name.clone(), descriptor);
        self.publish_event(vec![Parameter::new(name, value)], vec![], vec![]);
    }

    pub fn set_local(&mut self, name: impl Into<String>, value: ParameterValue) {
        let name = name.into();
        let is_new = !self.values.contains_key(&name);
        self.descriptors
            .entry(name.clone())
            .or_insert_with(|| ParameterDescriptor::new(&name, value.parameter_type()));
        self.values.insert(name.clone(), value.clone());
        let parameter = Parameter::new(name, value);
        if is_new {
            self.publish_event(vec![parameter], vec![], vec![]);
        } else {
            self.publish_event(vec![], vec![parameter], vec![]);
        }
    }

    pub fn serve_pending(&mut self) -> usize {
        let mut served = 0;
        let values = &self.values;
        served += self
            .get_parameters
            .serve_pending(|req| GetParametersResponse {
                values: req
                    .names
                    .iter()
                    .map(|name| values.get(name).cloned().unwrap_or_default())
                    .collect(),
            });
        served += self
            .get_parameter_types
            .serve_pending(|req| GetParameterTypesResponse {
                types: req
                    .names
                    .iter()
                    .map(|name| {
                        values
                            .get(name)
                            .map_or(ParameterType::NotSet, ParameterValue::parameter_type)
                            as u8
                    })
                    .collect(),
            });
        let descriptors = &self.descriptors;
        served += self
            .describe_parameters
            .serve_pending(|req| DescribeParametersResponse {
                descriptors: req
                    .names
                    .iter()
                    .map(|name| {
                        descriptors.get(name).cloned().unwrap_or_else(|| {
                            ParameterDescriptor::new(name, ParameterType::NotSet)
                        })
                    })
                    .collect(),
            });
        served += self
            .list_parameters
            .serve_pending(|req| ListParametersResponse {
                result: list_parameters(
                    &values.keys().cloned().collect::<Vec<_>>(),
                    &req.prefixes,
                    req.depth,
                ),
            });

        let mut set_requests = Vec::new();
        served += self.set_parameters.serve_pending(|req| {
            set_requests.push(req.parameters.clone());
            SetParametersResponse {
                results: vec![SetParametersResult::ok(); req.parameters.len()],
            }
        });
        for parameters in set_requests {
            self.apply_parameters(parameters);
        }

        let mut atomic_requests = Vec::new();
        served += self.set_parameters_atomically.serve_pending(|req| {
            atomic_requests.push(req.parameters.clone());
            SetParametersAtomicallyResponse {
                result: SetParametersResult::ok(),
            }
        });
        for parameters in atomic_requests {
            self.apply_parameters(parameters);
        }

        served
    }

    fn apply_parameters(&mut self, parameters: Vec<Parameter>) {
        let mut new_parameters = Vec::new();
        let mut changed_parameters = Vec::new();
        let mut deleted_parameters = Vec::new();
        for parameter in parameters {
            if parameter.value == ParameterValue::NotSet {
                self.values.remove(&parameter.name);
                self.descriptors.remove(&parameter.name);
                deleted_parameters.push(parameter);
                continue;
            }
            let is_new = !self.values.contains_key(&parameter.name);
            self.descriptors
                .entry(parameter.name.clone())
                .or_insert_with(|| {
                    ParameterDescriptor::new(&parameter.name, parameter.value.parameter_type())
                })
                .parameter_type = parameter.value.parameter_type();
            self.values
                .insert(parameter.name.clone(), parameter.value.clone());
            if is_new {
                new_parameters.push(parameter);
            } else {
                changed_parameters.push(parameter);
            }
        }
        self.publish_event(new_parameters, changed_parameters, deleted_parameters);
    }

    fn publish_event(
        &self,
        new_parameters: Vec<Parameter>,
        changed_parameters: Vec<Parameter>,
        deleted_parameters: Vec<Parameter>,
    ) {
        if new_parameters.is_empty()
            && changed_parameters.is_empty()
            && deleted_parameters.is_empty()
        {
            return;
        }
        self.events.publish(ParameterEvent {
            stamp: Time::now_system().to_msg(),
            node: self.node.clone(),
            new_parameters,
            changed_parameters,
            deleted_parameters,
        });
    }
}

/// Client for the parameter services a remote [`ParameterServer`] exposes,
/// mirroring the `ros2 param` CLI verbs. Each method issues one correlated
/// request via [`Client::call`] and blocks up to `timeout` for the reply,
/// returning `None` on timeout.
pub struct ParameterClient {
    get_parameters: Client<GetParametersRequest, GetParametersResponse>,
    set_parameters: Client<SetParametersRequest, SetParametersResponse>,
    set_parameters_atomically:
        Client<SetParametersAtomicallyRequest, SetParametersAtomicallyResponse>,
    get_parameter_types: Client<GetParameterTypesRequest, GetParameterTypesResponse>,
    describe_parameters: Client<DescribeParametersRequest, DescribeParametersResponse>,
    list_parameters: Client<ListParametersRequest, ListParametersResponse>,
}

impl ParameterClient {
    /// Bind a client to the parameter services of `node` on `dds`. The service
    /// names mirror [`ParameterServer::new`] (`/<node>/get_parameters`, …), so a
    /// leading slash on `node` is normalized to match the server's mangling.
    #[must_use]
    pub fn new(dds: &Dds, node: &str) -> Self {
        let service_prefix = format!("/{}", node.trim_start_matches('/'));
        Self {
            get_parameters: Client::new(dds, &format!("{service_prefix}/get_parameters")),
            set_parameters: Client::new(dds, &format!("{service_prefix}/set_parameters")),
            set_parameters_atomically: Client::new(
                dds,
                &format!("{service_prefix}/set_parameters_atomically"),
            ),
            get_parameter_types: Client::new(dds, &format!("{service_prefix}/get_parameter_types")),
            describe_parameters: Client::new(dds, &format!("{service_prefix}/describe_parameters")),
            list_parameters: Client::new(dds, &format!("{service_prefix}/list_parameters")),
        }
    }

    /// Fetch the current values of `names`, pairing each requested name with the
    /// value the server returned (a `NotSet` value marks an undeclared name).
    pub fn get(&mut self, names: &[String], timeout: Duration) -> Option<Vec<Parameter>> {
        let resp = self.get_parameters.call(
            GetParametersRequest {
                names: names.to_vec(),
            },
            timeout,
        )?;
        Some(
            names
                .iter()
                .cloned()
                .zip(resp.values)
                .map(|(name, value)| Parameter { name, value })
                .collect(),
        )
    }

    /// Fetch the [`ParameterType`] of each requested name (`NotSet` if undeclared).
    pub fn get_types(&mut self, names: &[String], timeout: Duration) -> Option<Vec<ParameterType>> {
        let resp = self.get_parameter_types.call(
            GetParameterTypesRequest {
                names: names.to_vec(),
            },
            timeout,
        )?;
        Some(resp.types.into_iter().map(ParameterType::from_u8).collect())
    }

    /// Set `parameters`, returning one [`SetParametersResult`] per parameter.
    pub fn set(
        &mut self,
        parameters: Vec<Parameter>,
        timeout: Duration,
    ) -> Option<Vec<SetParametersResult>> {
        let resp = self
            .set_parameters
            .call(SetParametersRequest { parameters }, timeout)?;
        Some(resp.results)
    }

    /// Set `parameters` atomically, returning the single combined result.
    pub fn set_atomically(
        &mut self,
        parameters: Vec<Parameter>,
        timeout: Duration,
    ) -> Option<SetParametersResult> {
        let resp = self
            .set_parameters_atomically
            .call(SetParametersAtomicallyRequest { parameters }, timeout)?;
        Some(resp.result)
    }

    /// List parameter names (and intermediate prefixes) under `prefixes`, limited
    /// to `depth` dot-separated segments (`0` means unlimited).
    pub fn list(
        &mut self,
        prefixes: &[String],
        depth: u64,
        timeout: Duration,
    ) -> Option<ListParametersResult> {
        let resp = self.list_parameters.call(
            ListParametersRequest {
                prefixes: prefixes.to_vec(),
                depth,
            },
            timeout,
        )?;
        Some(resp.result)
    }

    /// Fetch the [`ParameterDescriptor`] for each requested name.
    pub fn describe(
        &mut self,
        names: &[String],
        timeout: Duration,
    ) -> Option<Vec<ParameterDescriptor>> {
        let resp = self.describe_parameters.call(
            DescribeParametersRequest {
                names: names.to_vec(),
            },
            timeout,
        )?;
        Some(resp.descriptors)
    }
}

fn list_parameters(names: &[String], prefixes: &[String], depth: u64) -> ListParametersResult {
    let mut result_names = Vec::new();
    let mut result_prefixes = BTreeSet::new();
    for name in names {
        if !prefixes.is_empty()
            && !prefixes
                .iter()
                .any(|prefix| name == prefix || name.starts_with(&format!("{prefix}.")))
        {
            continue;
        }
        if depth == 0 || name.split('.').count() <= depth as usize {
            result_names.push(name.clone());
        }
        if let Some((prefix, _)) = name.rsplit_once('.') {
            result_prefixes.insert(prefix.to_string());
        }
    }
    ListParametersResult {
        names: result_names,
        prefixes: result_prefixes.into_iter().collect(),
    }
}

#[cfg(test)]
mod tests {
    use super::{
        list_parameters, CdrMsg, GetParametersRequest, Parameter, ParameterDescriptor,
        ParameterEvent, ParameterType, ParameterValue, SetParametersRequest,
    };

    #[test]
    fn parameter_value_round_trips_arrays() {
        let req = SetParametersRequest {
            parameters: vec![
                Parameter::new("gain", ParameterValue::Double(1.5)),
                Parameter::new(
                    "names",
                    ParameterValue::StringArray(vec!["left".into(), "right".into()]),
                ),
            ],
        };
        let back = SetParametersRequest::decode(&req.encode()).unwrap();
        assert_eq!(back, req);
    }

    #[test]
    fn parameter_event_round_trips() {
        let event = ParameterEvent {
            node: "/node".into(),
            new_parameters: vec![Parameter::new("enabled", ParameterValue::Bool(true))],
            ..ParameterEvent::default()
        };
        let back = ParameterEvent::decode(&event.encode()).unwrap();
        assert_eq!(back, event);
    }

    #[test]
    fn get_parameters_request_round_trips() {
        let req = GetParametersRequest {
            names: vec!["foo".into(), "bar".into()],
        };
        assert_eq!(GetParametersRequest::decode(&req.encode()).unwrap(), req);
    }

    #[test]
    fn descriptors_carry_types() {
        let descriptor = ParameterDescriptor::new("speed", ParameterType::Double);
        assert_eq!(descriptor.parameter_type, ParameterType::Double);
    }

    #[test]
    fn list_parameters_filters_prefixes_and_depth() {
        let names = vec![
            "camera.exposure".into(),
            "camera.gain".into(),
            "nav.speed.max".into(),
        ];
        let result = list_parameters(&names, &["camera".into()], 2);
        assert_eq!(result.names, vec!["camera.exposure", "camera.gain"]);
        assert!(result.prefixes.contains(&"camera".to_string()));
    }
}
