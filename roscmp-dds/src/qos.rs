//! QoS profiles and compatibility diagnostics for ROS endpoints.
//!
//! [`QosProfile`] is the full description of an endpoint's QoS: the three ROS
//! presets ([`Qos`]) are the common starting points, and the optional
//! deadline/lifespan/liveliness policies layer on top via the `with_*`
//! builders. [`QosProfile::policies`] lowers it to the concrete RustDDS
//! [`QosPolicies`].

use std::time::Duration;

use rustdds::{
    policy::{Deadline, Durability, History, Lifespan, Liveliness as DdsLiveliness, Reliability},
    QosPolicies, QosPolicyBuilder,
};

use crate::transport::Qos;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ReliabilityKind {
    BestEffort,
    Reliable,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DurabilityKind {
    Volatile,
    TransientLocal,
}

/// Liveliness policy: how a writer asserts it is still alive within `lease`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Liveliness {
    /// Middleware asserts liveliness automatically each `lease`.
    Automatic { lease: Duration },
    /// The participant must assert liveliness for all its writers each `lease`.
    ManualByParticipant { lease: Duration },
    /// This writer must assert liveliness (e.g. by writing) each `lease`.
    ManualByTopic { lease: Duration },
}

/// Full QoS description for one endpoint. Start from a [`Qos`] preset via
/// [`QosProfile::from_preset`], then layer optional policies with the `with_*`
/// builders.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct QosProfile {
    pub reliability: ReliabilityKind,
    pub durability: DurabilityKind,
    pub depth: usize,
    /// Keep every sample (RustDDS `History::KeepAll`), ignoring `depth`.
    pub keep_all: bool,
    /// Max period between successive samples (`None` = no deadline).
    pub deadline: Option<Duration>,
    /// How long a written sample stays valid (`None` = infinite).
    pub lifespan: Option<Duration>,
    /// Liveliness assertion policy (`None` = RustDDS default = automatic).
    pub liveliness: Option<Liveliness>,
}

impl QosProfile {
    #[must_use]
    pub const fn from_preset(qos: Qos) -> Self {
        let (reliability, durability, depth) = match qos {
            Qos::Default => (ReliabilityKind::Reliable, DurabilityKind::Volatile, 10),
            Qos::SensorData => (ReliabilityKind::BestEffort, DurabilityKind::Volatile, 5),
            Qos::Latched => (ReliabilityKind::Reliable, DurabilityKind::TransientLocal, 1),
        };
        Self {
            reliability,
            durability,
            depth,
            keep_all: false,
            deadline: None,
            lifespan: None,
            liveliness: None,
        }
    }

    /// Keep every sample (`History::KeepAll`) rather than the last `depth`.
    #[must_use]
    pub const fn with_keep_all(mut self) -> Self {
        self.keep_all = true;
        self
    }

    /// Require a sample at least every `period`.
    #[must_use]
    pub const fn with_deadline(mut self, period: Duration) -> Self {
        self.deadline = Some(period);
        self
    }

    /// Expire samples older than `ttl`.
    #[must_use]
    pub const fn with_lifespan(mut self, ttl: Duration) -> Self {
        self.lifespan = Some(ttl);
        self
    }

    /// Set the liveliness assertion policy.
    #[must_use]
    pub const fn with_liveliness(mut self, liveliness: Liveliness) -> Self {
        self.liveliness = Some(liveliness);
        self
    }

    /// Lower this profile to concrete RustDDS policies.
    #[must_use]
    pub fn policies(&self) -> QosPolicies {
        let reliability = match self.reliability {
            ReliabilityKind::Reliable => Reliability::Reliable {
                max_blocking_time: rustdds::Duration::from_millis(100),
            },
            ReliabilityKind::BestEffort => Reliability::BestEffort,
        };
        let durability = match self.durability {
            DurabilityKind::Volatile => Durability::Volatile,
            DurabilityKind::TransientLocal => Durability::TransientLocal,
        };
        let history = if self.keep_all {
            History::KeepAll
        } else {
            History::KeepLast {
                depth: i32::try_from(self.depth).unwrap_or(i32::MAX),
            }
        };
        let mut b = QosPolicyBuilder::new()
            .reliability(reliability)
            .durability(durability)
            .history(history);
        if let Some(period) = self.deadline {
            b = b.deadline(Deadline(rustdds::Duration::from_std(period)));
        }
        if let Some(ttl) = self.lifespan {
            b = b.lifespan(Lifespan {
                duration: rustdds::Duration::from_std(ttl),
            });
        }
        if let Some(liveliness) = self.liveliness {
            b = b.liveliness(match liveliness {
                Liveliness::Automatic { lease } => DdsLiveliness::Automatic {
                    lease_duration: rustdds::Duration::from_std(lease),
                },
                Liveliness::ManualByParticipant { lease } => DdsLiveliness::ManualByParticipant {
                    lease_duration: rustdds::Duration::from_std(lease),
                },
                Liveliness::ManualByTopic { lease } => DdsLiveliness::ManualByTopic {
                    lease_duration: rustdds::Duration::from_std(lease),
                },
            });
        }
        b.build()
    }
}

#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct QosCompatibility {
    pub compatible: bool,
    pub reasons: Vec<QosIncompatibility>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum QosIncompatibility {
    Reliability,
    Durability,
}

#[must_use]
pub fn check_compatibility(offered: QosProfile, requested: QosProfile) -> QosCompatibility {
    let mut reasons = Vec::new();
    if !reliability_compatible(offered.reliability, requested.reliability) {
        reasons.push(QosIncompatibility::Reliability);
    }
    if !durability_compatible(offered.durability, requested.durability) {
        reasons.push(QosIncompatibility::Durability);
    }
    QosCompatibility {
        compatible: reasons.is_empty(),
        reasons,
    }
}

#[must_use]
pub fn check_presets(offered: Qos, requested: Qos) -> QosCompatibility {
    check_compatibility(
        QosProfile::from_preset(offered),
        QosProfile::from_preset(requested),
    )
}

fn reliability_compatible(offered: ReliabilityKind, requested: ReliabilityKind) -> bool {
    matches!(
        (offered, requested),
        (
            ReliabilityKind::Reliable,
            ReliabilityKind::Reliable | ReliabilityKind::BestEffort
        ) | (ReliabilityKind::BestEffort, ReliabilityKind::BestEffort)
    )
}

fn durability_compatible(offered: DurabilityKind, requested: DurabilityKind) -> bool {
    matches!(
        (offered, requested),
        (
            DurabilityKind::TransientLocal,
            DurabilityKind::TransientLocal | DurabilityKind::Volatile
        ) | (DurabilityKind::Volatile, DurabilityKind::Volatile)
    )
}

/// A QoS-related status event surfaced from an endpoint via
/// `poll_events()`. Each variant maps from one RustDDS status notification;
/// `count` fields report the cumulative occurrence count.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum QosEvent {
    /// A reader missed its requested deadline / a writer missed its offered one.
    DeadlineMissed { count: i32 },
    /// Offered (writer) or requested (reader) QoS is incompatible with a peer.
    IncompatibleQos {
        policy: IncompatiblePolicy,
        count: i32,
    },
    /// A remote writer became active or inactive (reader side).
    LivelinessChanged { alive: i32, not_alive: i32 },
    /// This writer failed to assert liveliness within its lease.
    LivelinessLost { count: i32 },
    /// A sample was never received (reader side).
    SampleLost { count: i32 },
    /// A sample was dropped because resource limits were exceeded (reader side).
    SampleRejected { count: i32 },
    /// The reader matched/unmatched a compatible writer (`current` = live peers).
    SubscriptionMatched { current: i32 },
    /// The writer matched/unmatched a compatible reader (`current` = live peers).
    PublicationMatched { current: i32 },
}

/// The QoS policy that caused an [`QosEvent::IncompatibleQos`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum IncompatiblePolicy {
    Reliability,
    Durability,
    Deadline,
    Liveliness,
    History,
    /// Any other policy id RustDDS reported.
    Other,
}

impl From<rustdds::qos::QosPolicyId> for IncompatiblePolicy {
    fn from(id: rustdds::qos::QosPolicyId) -> Self {
        use rustdds::qos::QosPolicyId;
        match id {
            QosPolicyId::Reliability => Self::Reliability,
            QosPolicyId::Durability => Self::Durability,
            QosPolicyId::Deadline => Self::Deadline,
            QosPolicyId::Liveliness => Self::Liveliness,
            QosPolicyId::History => Self::History,
            _ => Self::Other,
        }
    }
}

impl From<rustdds::DataReaderStatus> for QosEvent {
    fn from(status: rustdds::DataReaderStatus) -> Self {
        use rustdds::DataReaderStatus as S;
        match status {
            S::RequestedDeadlineMissed { count } => Self::DeadlineMissed {
                count: count.count(),
            },
            S::RequestedIncompatibleQos {
                count,
                last_policy_id,
                ..
            } => Self::IncompatibleQos {
                policy: last_policy_id.into(),
                count: count.count(),
            },
            S::LivelinessChanged {
                alive_total,
                not_alive_total,
            } => Self::LivelinessChanged {
                alive: alive_total.count(),
                not_alive: not_alive_total.count(),
            },
            S::SampleLost { count } => Self::SampleLost {
                count: count.count(),
            },
            S::SampleRejected { count, .. } => Self::SampleRejected {
                count: count.count(),
            },
            S::SubscriptionMatched { current, .. } => Self::SubscriptionMatched {
                current: current.count(),
            },
        }
    }
}

impl From<rustdds::DataWriterStatus> for QosEvent {
    fn from(status: rustdds::DataWriterStatus) -> Self {
        use rustdds::DataWriterStatus as S;
        match status {
            S::LivelinessLost { count } => Self::LivelinessLost {
                count: count.count(),
            },
            S::OfferedDeadlineMissed { count } => Self::DeadlineMissed {
                count: count.count(),
            },
            S::OfferedIncompatibleQos {
                count,
                last_policy_id,
                ..
            } => Self::IncompatibleQos {
                policy: last_policy_id.into(),
                count: count.count(),
            },
            S::PublicationMatched { current, .. } => Self::PublicationMatched {
                current: current.count(),
            },
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{
        check_presets, DurabilityKind, IncompatiblePolicy, Liveliness, QosEvent,
        QosIncompatibility, QosProfile, ReliabilityKind,
    };
    use crate::transport::Qos;
    use std::time::Duration;

    #[test]
    fn reliable_writer_satisfies_best_effort_reader() {
        assert!(check_presets(Qos::Default, Qos::SensorData).compatible);
    }

    #[test]
    fn best_effort_writer_does_not_satisfy_reliable_reader() {
        let result = check_presets(Qos::SensorData, Qos::Default);
        assert!(!result.compatible);
        assert_eq!(result.reasons, vec![QosIncompatibility::Reliability]);
    }

    #[test]
    fn volatile_writer_does_not_satisfy_transient_local_reader() {
        let result = check_presets(Qos::Default, Qos::Latched);
        assert!(!result.compatible);
        assert_eq!(result.reasons, vec![QosIncompatibility::Durability]);
    }

    #[test]
    fn default_preset_maps_to_reliable_volatile_keeplast10() {
        let p = QosProfile::from_preset(Qos::Default).policies();
        assert!(p.is_reliable());
        assert_eq!(
            p.reliable_max_blocking_time(),
            Some(rustdds::Duration::from_millis(100))
        );
        assert_eq!(p.durability(), Some(rustdds::policy::Durability::Volatile));
        assert_eq!(
            p.history(),
            Some(rustdds::policy::History::KeepLast { depth: 10 })
        );
        assert_eq!(p.deadline(), None);
        assert_eq!(p.lifespan(), None);
        assert_eq!(p.liveliness(), None);
    }

    #[test]
    fn sensor_data_preset_maps_to_best_effort_keeplast5() {
        let p = QosProfile::from_preset(Qos::SensorData).policies();
        assert!(!p.is_reliable());
        assert_eq!(
            p.history(),
            Some(rustdds::policy::History::KeepLast { depth: 5 })
        );
    }

    #[test]
    fn latched_preset_maps_to_transient_local_keeplast1() {
        let p = QosProfile::from_preset(Qos::Latched).policies();
        assert!(p.is_reliable());
        assert_eq!(
            p.durability(),
            Some(rustdds::policy::Durability::TransientLocal)
        );
        assert_eq!(
            p.history(),
            Some(rustdds::policy::History::KeepLast { depth: 1 })
        );
    }

    #[test]
    fn keep_all_lowers_to_history_keep_all() {
        let p = QosProfile::from_preset(Qos::Default)
            .with_keep_all()
            .policies();
        assert_eq!(p.history(), Some(rustdds::policy::History::KeepAll));
    }

    #[test]
    fn presets_default_to_keep_last() {
        // KeepAll is opt-in only; presets keep their KeepLast depth.
        for qos in [Qos::Default, Qos::SensorData, Qos::Latched] {
            assert!(!QosProfile::from_preset(qos).keep_all);
        }
    }

    #[test]
    fn optional_policies_lower_to_rustdds() {
        let p = QosProfile::from_preset(Qos::Default)
            .with_deadline(Duration::from_millis(250))
            .with_lifespan(Duration::from_secs(2))
            .with_liveliness(Liveliness::ManualByTopic {
                lease: Duration::from_millis(500),
            })
            .policies();
        assert_eq!(
            p.deadline(),
            Some(rustdds::policy::Deadline(rustdds::Duration::from_millis(
                250
            )))
        );
        assert_eq!(
            p.lifespan(),
            Some(rustdds::policy::Lifespan {
                duration: rustdds::Duration::from_secs(2)
            })
        );
        assert_eq!(
            p.liveliness(),
            Some(rustdds::policy::Liveliness::ManualByTopic {
                lease_duration: rustdds::Duration::from_millis(500)
            })
        );
    }

    #[test]
    fn builders_are_independent_of_preset_base() {
        let base = QosProfile::from_preset(Qos::SensorData);
        assert_eq!(base.reliability, ReliabilityKind::BestEffort);
        assert_eq!(base.durability, DurabilityKind::Volatile);
        let tuned = base.with_deadline(Duration::from_millis(100));
        assert_eq!(tuned.deadline, Some(Duration::from_millis(100)));
        assert_eq!(base.deadline, None, "builder must not mutate the base");
    }

    #[test]
    fn reader_status_maps_to_qos_events() {
        use rustdds::dds::statusevents::CountWithChange;
        let ev: QosEvent = rustdds::DataReaderStatus::RequestedDeadlineMissed {
            count: CountWithChange::start_from(3, 1),
        }
        .into();
        assert_eq!(ev, QosEvent::DeadlineMissed { count: 3 });

        let ev: QosEvent = rustdds::DataReaderStatus::RequestedIncompatibleQos {
            count: CountWithChange::start_from(1, 1),
            last_policy_id: rustdds::qos::QosPolicyId::Reliability,
            writer: rustdds::GUID::GUID_UNKNOWN,
            requested_qos: Box::default(),
            offered_qos: Box::default(),
        }
        .into();
        assert_eq!(
            ev,
            QosEvent::IncompatibleQos {
                policy: IncompatiblePolicy::Reliability,
                count: 1,
            }
        );
    }

    #[test]
    fn writer_status_maps_to_qos_events() {
        use rustdds::dds::statusevents::CountWithChange;
        let ev: QosEvent = rustdds::DataWriterStatus::OfferedIncompatibleQos {
            count: CountWithChange::start_from(2, 1),
            last_policy_id: rustdds::qos::QosPolicyId::Durability,
            reader: rustdds::GUID::GUID_UNKNOWN,
            requested_qos: Box::default(),
            offered_qos: Box::default(),
        }
        .into();
        assert_eq!(
            ev,
            QosEvent::IncompatibleQos {
                policy: IncompatiblePolicy::Durability,
                count: 2,
            }
        );

        let ev: QosEvent = rustdds::DataWriterStatus::LivelinessLost {
            count: CountWithChange::start_from(5, 1),
        }
        .into();
        assert_eq!(ev, QosEvent::LivelinessLost { count: 5 });
    }
}
