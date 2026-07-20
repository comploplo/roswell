//! `type_description_interfaces/srv/GetTypeDescription` support.

use std::collections::HashMap;

use crate::msgs::{
    type_description_interfaces__Field as FieldGen,
    type_description_interfaces__FieldType as FieldTypeGen,
    type_description_interfaces__GetTypeDescription_Request as GetTypeDescriptionRequest,
    type_description_interfaces__GetTypeDescription_Response as GetTypeDescriptionResponse,
    type_description_interfaces__IndividualTypeDescription as IndividualGen,
    type_description_interfaces__KeyValue as KeyValueGen,
    type_description_interfaces__TypeDescription as TypeDescriptionGen,
    type_description_interfaces__TypeSource as TypeSourceGen, RosSequence, RosString,
};
use crate::service::Service;
use crate::transport::Dds;

#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct FieldData {
    pub name: String,
    pub type_id: u8,
    pub capacity: u64,
    pub string_capacity: u64,
    pub nested_type_name: String,
    pub default_value: String,
}

#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct IndividualTypeDescriptionData {
    pub type_name: String,
    pub fields: Vec<FieldData>,
}

#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct TypeSourceData {
    pub type_name: String,
    pub encoding: String,
    pub raw_file_contents: String,
}

#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct KeyValueData {
    pub key: String,
    pub value: String,
}

#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct TypeDescriptionData {
    pub type_hash: String,
    pub type_description: IndividualTypeDescriptionData,
    pub referenced_type_descriptions: Vec<IndividualTypeDescriptionData>,
    pub type_sources: Vec<TypeSourceData>,
    pub extra_information: Vec<KeyValueData>,
}

// The wire (de)serialization is the generated `type_description_interfaces`
// codec; these builders assemble the C-ABI generated request/response structs
// from the ergonomic `*Data` types. Values written to DDS own their buffers, so
// callers `fini` them after use (see the tests / `Service` boundary).

fn field_gen(d: &FieldData) -> FieldGen {
    FieldGen {
        name: RosString::alloc(&d.name),
        r#type: FieldTypeGen {
            type_id: d.type_id,
            capacity: d.capacity,
            string_capacity: d.string_capacity,
            nested_type_name: RosString::alloc(&d.nested_type_name),
        },
        default_value: RosString::alloc(&d.default_value),
    }
}

fn individual_gen(d: &IndividualTypeDescriptionData) -> IndividualGen {
    IndividualGen {
        type_name: RosString::alloc(&d.type_name),
        fields: RosSequence::alloc(d.fields.iter().map(field_gen).collect()),
    }
}

fn type_source_gen(d: &TypeSourceData) -> TypeSourceGen {
    TypeSourceGen {
        type_name: RosString::alloc(&d.type_name),
        encoding: RosString::alloc(&d.encoding),
        raw_file_contents: RosString::alloc(&d.raw_file_contents),
    }
}

fn key_value_gen(d: &KeyValueData) -> KeyValueGen {
    KeyValueGen {
        key: RosString::alloc(&d.key),
        value: RosString::alloc(&d.value),
    }
}

/// A `GetTypeDescription` request for `type_name`/`type_hash`.
#[must_use]
pub fn get_type_description_request(
    type_name: &str,
    type_hash: &str,
    include_type_sources: bool,
) -> GetTypeDescriptionRequest {
    GetTypeDescriptionRequest {
        type_name: RosString::alloc(type_name),
        type_hash: RosString::alloc(type_hash),
        include_type_sources,
    }
}

/// A successful `GetTypeDescription` response describing `data`.
#[must_use]
pub fn describe_success(
    data: &TypeDescriptionData,
    include_type_sources: bool,
) -> GetTypeDescriptionResponse {
    let sources = if include_type_sources {
        data.type_sources.iter().map(type_source_gen).collect()
    } else {
        Vec::new()
    };
    GetTypeDescriptionResponse {
        successful: true,
        failure_reason: RosString::alloc(""),
        type_description: TypeDescriptionGen {
            type_description: individual_gen(&data.type_description),
            referenced_type_descriptions: RosSequence::alloc(
                data.referenced_type_descriptions
                    .iter()
                    .map(individual_gen)
                    .collect(),
            ),
        },
        type_sources: RosSequence::alloc(sources),
        extra_information: RosSequence::alloc(
            data.extra_information.iter().map(key_value_gen).collect(),
        ),
    }
}

/// A failed `GetTypeDescription` response carrying `reason`.
#[must_use]
pub fn describe_failure(reason: &str) -> GetTypeDescriptionResponse {
    GetTypeDescriptionResponse {
        successful: false,
        failure_reason: RosString::alloc(reason),
        type_description: TypeDescriptionGen {
            type_description: individual_gen(&IndividualTypeDescriptionData::default()),
            referenced_type_descriptions: RosSequence::alloc(Vec::new()),
        },
        type_sources: RosSequence::alloc(Vec::new()),
        extra_information: RosSequence::alloc(Vec::new()),
    }
}

#[derive(Default)]
pub struct TypeDescriptionRegistry {
    entries: HashMap<String, TypeDescriptionData>,
}

impl TypeDescriptionRegistry {
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    pub fn insert(&mut self, data: TypeDescriptionData) {
        self.entries
            .insert(data.type_description.type_name.clone(), data);
    }

    #[must_use]
    pub fn respond(&self, req: &GetTypeDescriptionRequest) -> GetTypeDescriptionResponse {
        let (type_name, type_hash) = (req.type_name.as_str(), req.type_hash.as_str());
        let Some(entry) = self.entries.get(type_name) else {
            return describe_failure("type description not found");
        };
        if !type_hash.is_empty() && type_hash != entry.type_hash {
            return describe_failure("type hash mismatch");
        }
        describe_success(entry, req.include_type_sources)
    }
}

pub struct TypeDescriptionService {
    registry: TypeDescriptionRegistry,
    service: Service<GetTypeDescriptionRequest, GetTypeDescriptionResponse>,
}

impl TypeDescriptionService {
    #[must_use]
    pub fn new(dds: &Dds, node_name: &str, registry: TypeDescriptionRegistry) -> Self {
        let base = format!("/{}", node_name.trim_matches('/'));
        Self {
            registry,
            service: Service::new(dds, &format!("{base}/get_type_description")),
        }
    }

    pub fn serve_pending(&mut self) -> usize {
        self.service.serve_pending(|req| self.registry.respond(req))
    }
}

#[cfg(test)]
mod tests {
    use super::{
        describe_success, get_type_description_request, FieldData, GetTypeDescriptionResponse,
        IndividualTypeDescriptionData, TypeDescriptionData, TypeDescriptionRegistry,
        TypeSourceData,
    };

    fn sample_data() -> TypeDescriptionData {
        TypeDescriptionData {
            type_hash: "RIHS01_deadbeef".to_string(),
            type_description: IndividualTypeDescriptionData {
                type_name: "demo/msg/Thing".to_string(),
                fields: vec![FieldData {
                    name: "value".to_string(),
                    type_id: 7,
                    capacity: 0,
                    string_capacity: 0,
                    nested_type_name: String::new(),
                    default_value: String::new(),
                }],
            },
            referenced_type_descriptions: Vec::new(),
            type_sources: vec![TypeSourceData {
                type_name: "demo/msg/Thing".to_string(),
                encoding: "msg".to_string(),
                raw_file_contents: "uint32 value\n".to_string(),
            }],
            extra_information: Vec::new(),
        }
    }

    #[test]
    fn get_type_description_response_round_trips() {
        let mut msg = describe_success(&sample_data(), true);
        let mut back =
            GetTypeDescriptionResponse::from_cdr(&msg.to_cdr(crate::msgs::Endian::Little)).unwrap();
        unsafe {
            assert!(back.successful);
            assert_eq!(
                back.type_description.type_description.type_name.as_str(),
                "demo/msg/Thing"
            );
            assert_eq!(back.type_sources.as_slice().len(), 1);
            msg.fini();
            back.fini();
        }
    }

    #[test]
    fn registry_checks_type_hash_and_sources_flag() {
        let mut registry = TypeDescriptionRegistry::new();
        registry.insert(sample_data());
        let mut req = get_type_description_request("demo/msg/Thing", "RIHS01_deadbeef", false);
        let mut resp = registry.respond(&req);
        unsafe {
            assert!(resp.successful);
            assert_eq!(resp.type_sources.as_slice().len(), 0);
            req.fini();
            resp.fini();
        }

        let mut bad = get_type_description_request("demo/msg/Thing", "RIHS01_bad", true);
        let mut resp = registry.respond(&bad);
        unsafe {
            assert!(!resp.successful);
            assert_eq!(resp.failure_reason.as_str(), "type hash mismatch");
            bad.fini();
            resp.fini();
        }
    }
}
