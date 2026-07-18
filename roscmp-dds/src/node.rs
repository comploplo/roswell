//! Minimal ROS-like node runtime and single-threaded executor.
//!
//! Also implements standard `--ros-args` handling (`-r`/`-p`/`--params-file`)
//! and `use_sim_time`/`/clock` integration for node timers.

use std::collections::BTreeMap;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use crate::codec::CdrMsg;
use crate::discovery::DiscoveryInfo;
use crate::parameters::{ParameterServer, ParameterValue};
use crate::service::{Client, Service};
use crate::time::{Clock, ClockMode, ClockMsg, Time};
use crate::transport::{Dds, DdsPub, DdsSub, MsgSubscriber, Qos, Transport};

pub struct Node {
    name: String,
    namespace: String,
    dds: Option<Dds>,
    remaps: Vec<(String, String)>,
    initial_parameters: BTreeMap<String, ParameterValue>,
    clock: Clock<DdsSub<ClockMsg>>,
    epoch: Instant,
    timers: Vec<Timer>,
    subscriptions: Vec<Box<dyn Subscription>>,
    services: Vec<Box<dyn ServiceEndpoint>>,
    // Kept alive so the latched ros_discovery_info sample stays advertised (the
    // node vanishes from `ros2 node list` when the participant does), and
    // updated as endpoints are added so `ros2 node info` lists them.
    discovery: Option<DiscoveryInfo>,
    shutdown: Arc<AtomicBool>,
    // Waitset: every subscription/service/clock reader registers its data
    // event source here so the executor blocks on `poll` until one is ready
    // instead of busy-polling on a fixed sleep.
    poll: mio::Poll,
    events: mio::Events,
    next_token: usize,
}

impl Node {
    #[must_use]
    pub fn new(name: impl Into<String>) -> Self {
        Self::with_namespace(name, "", 0)
    }

    #[must_use]
    pub fn with_namespace(
        name: impl Into<String>,
        namespace: impl Into<String>,
        domain: u16,
    ) -> Self {
        let dds = Dds::new(domain);
        let clock = Clock::from_transport(&dds, false);
        let name = name.into();
        let namespace = normalize_namespace(&namespace.into());
        let mut discovery = DiscoveryInfo::new(&dds);
        discovery.add_node(&namespace, &name);
        Self {
            name,
            namespace,
            dds: Some(dds),
            remaps: Vec::new(),
            initial_parameters: BTreeMap::new(),
            clock,
            epoch: Instant::now(),
            timers: Vec::new(),
            subscriptions: Vec::new(),
            services: Vec::new(),
            discovery: Some(discovery),
            shutdown: Arc::new(AtomicBool::new(false)),
            poll: mio::Poll::new().expect("create mio poll"),
            events: mio::Events::with_capacity(8),
            next_token: 0,
        }
    }

    /// Build a node from process arguments, applying standard `--ros-args`
    /// (`-r`/`--remap`, `-p`/`--param`, `--params-file`, `--` terminator).
    /// `__node:=`/`__ns:=` remaps override `name`/namespace; `use_sim_time`
    /// (from `-p` or a params file) switches node timers onto `/clock`.
    #[must_use]
    pub fn from_args(name: impl Into<String>, args: &[String]) -> Self {
        Self::from_args_on_domain(name, args, 0)
    }

    #[must_use]
    pub fn from_args_on_domain(name: impl Into<String>, args: &[String], domain: u16) -> Self {
        let parsed = parse_ros_args(args);
        let name = parsed.node_name.clone().unwrap_or_else(|| name.into());
        let namespace = parsed.namespace.clone().unwrap_or_default();
        let mut node = Self::with_namespace(name, namespace, domain);
        node.remaps = parsed.remaps;

        // Parameter overrides: params-file sections first, then `-p` on top.
        let sections = node.param_section_names();
        let mut params: BTreeMap<String, ParameterValue> = BTreeMap::new();
        for file in &parsed.params_files {
            match std::fs::read_to_string(file) {
                Ok(text) => {
                    for (key, value) in params_from_file(&text, &sections) {
                        params.insert(key, value);
                    }
                }
                Err(err) => eprintln!("roscmp: ignoring --params-file '{file}': {err}"),
            }
        }
        for (key, value) in parsed.params {
            params.insert(key, value);
        }

        let use_sim_time = matches!(params.get("use_sim_time"), Some(ParameterValue::Bool(true)));
        node.initial_parameters = params;
        if use_sim_time {
            node.clock = Clock::from_transport(node.dds(), true);
            // /clock is subscription data: registering its reader lets the
            // waitset wake on clock arrival (which is the only thing that
            // advances sim time), so no busy poll is needed to service it.
            if let Some(sub) = node.clock.subscriber_mut() {
                Self::register_reader(&node.poll, &mut node.next_token, sub.event_source());
            }
        }
        node
    }

    #[cfg(test)]
    fn without_dds(name: impl Into<String>, namespace: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            namespace: normalize_namespace(&namespace.into()),
            dds: None,
            remaps: Vec::new(),
            initial_parameters: BTreeMap::new(),
            clock: Clock::wall_typed(),
            epoch: Instant::now(),
            timers: Vec::new(),
            subscriptions: Vec::new(),
            services: Vec::new(),
            discovery: None,
            shutdown: Arc::new(AtomicBool::new(false)),
            poll: mio::Poll::new().expect("create mio poll"),
            events: mio::Events::with_capacity(8),
            next_token: 0,
        }
    }

    #[must_use]
    pub fn name(&self) -> &str {
        &self.name
    }

    #[must_use]
    pub fn namespace(&self) -> &str {
        &self.namespace
    }

    #[must_use]
    pub fn dds(&self) -> &Dds {
        self.dds.as_ref().expect("node has no DDS participant")
    }

    /// Fully-qualified node name (`/ns/name`, or `name` with no namespace),
    /// used as the parameter service prefix and params-file section key.
    #[must_use]
    fn fully_qualified_name(&self) -> String {
        if self.namespace == "/" {
            self.name.clone()
        } else {
            format!("{}/{}", self.namespace.trim_start_matches('/'), self.name)
        }
    }

    /// Section keys a params file may use to target this node.
    fn param_section_names(&self) -> Vec<String> {
        let fq = self.fully_qualified_name();
        let mut names = vec![
            "/**".to_string(),
            "**".to_string(),
            self.name.clone(),
            format!("/{}", self.name),
            fq.clone(),
            format!("/{fq}"),
        ];
        names.sort();
        names.dedup();
        names
    }

    /// Expand a name to fully-qualified form (namespace + leading slash),
    /// ignoring remap rules.
    fn expand_name(&self, name: &str) -> String {
        if name.starts_with('/') {
            name.to_string()
        } else if self.namespace == "/" {
            format!("/{name}")
        } else {
            format!("{}/{name}", self.namespace)
        }
    }

    /// Resolve a topic/service name to its fully-qualified form, applying any
    /// `-r from:=to` remap rules (matched against the raw or expanded name).
    #[must_use]
    pub fn resolve_name(&self, name: &str) -> String {
        let resolved = self.expand_name(name);
        for (from, to) in &self.remaps {
            if from == name || *from == resolved || self.expand_name(from) == resolved {
                return self.expand_name(to);
            }
        }
        resolved
    }

    /// The value of a parameter supplied via `--ros-args`/params file, if any.
    #[must_use]
    pub fn parameter(&self, name: &str) -> Option<&ParameterValue> {
        self.initial_parameters.get(name)
    }

    /// Whether the node is running on sim time (`use_sim_time:=true`).
    #[must_use]
    pub fn use_sim_time(&self) -> bool {
        self.clock.mode() == ClockMode::Sim
    }

    /// Current ROS time: the latest `/clock` sample under sim time, otherwise
    /// the system wall clock. Returns the sim origin (0) before the first
    /// `/clock` sample arrives.
    pub fn now(&mut self) -> Time {
        match self.clock.mode() {
            ClockMode::Wall => Time::now_system(),
            ClockMode::Sim => {
                let _ = self.clock.poll();
                self.clock.now()
            }
        }
    }

    pub fn publisher<M: CdrMsg>(&self, topic: &str, qos: Qos) -> DdsPub<M> {
        self.dds().publisher(&self.resolve_name(topic), qos)
    }

    pub fn subscriber<M: CdrMsg>(&self, topic: &str, qos: Qos) -> DdsSub<M> {
        self.dds().subscriber(&self.resolve_name(topic), qos)
    }

    /// Register a callback subscription: `callback` runs once per received `M`
    /// when the topic is drained during [`Node::spin_once`] / [`Node::spin`].
    pub fn subscribe<M: CdrMsg + 'static>(
        &mut self,
        topic: &str,
        qos: Qos,
        callback: impl FnMut(M) + 'static,
    ) {
        let mut sub = self.subscriber::<M>(topic, qos);
        // Advertise the subscriber on ros_discovery_info so `ros2 node info`
        // lists it (disjoint field borrows: discovery vs. name/namespace).
        if let Some(discovery) = self.discovery.as_mut() {
            discovery.add_reader_gid(&self.namespace, &self.name, sub.gid());
        }
        Self::register_reader(&self.poll, &mut self.next_token, sub.event_source());
        self.subscriptions.push(Box::new(CallbackSub {
            sub,
            callback: Box::new(callback),
        }));
    }

    /// Register a service server serviced from the spin loop: `handler` maps a
    /// `Req` to its `Resp`, replies are correlated by sample identity.
    pub fn create_service<Req: CdrMsg + 'static, Resp: CdrMsg + 'static>(
        &mut self,
        service: &str,
        handler: impl FnMut(&Req) -> Resp + 'static,
    ) {
        let mut service = self.service::<Req, Resp>(service);
        Self::register_reader(&self.poll, &mut self.next_token, service.event_source());
        self.services.push(Box::new(CallbackService {
            service,
            handler: Box::new(handler),
        }));
    }

    #[must_use]
    pub fn service<Req: CdrMsg, Resp: CdrMsg>(&self, service: &str) -> Service<Req, Resp> {
        Service::new(self.dds(), &self.resolve_name(service))
    }

    #[must_use]
    pub fn client<Req: CdrMsg, Resp: CdrMsg>(&self, service: &str) -> Client<Req, Resp> {
        Client::new(self.dds(), &self.resolve_name(service))
    }

    /// A parameter server seeded with the overrides supplied via `--ros-args`
    /// and any `--params-file` sections matching this node.
    #[must_use]
    pub fn parameter_server(&self) -> ParameterServer<DdsPub<crate::parameters::ParameterEvent>> {
        let mut server = ParameterServer::new(self.dds(), &self.fully_qualified_name());
        for (name, value) in &self.initial_parameters {
            server.set_local(name.clone(), value.clone());
        }
        server
    }

    pub fn create_timer(&mut self, period: Duration, mut callback: impl FnMut(&Dds) + 'static) {
        let now = self.tick_nanos().unwrap_or(0);
        self.timers.push(Timer::new(period, now, move |dds| {
            callback(dds.expect("DDS timer fired without DDS participant"));
        }));
    }

    #[cfg(test)]
    fn create_test_timer(&mut self, period: Duration, callback: impl FnMut() + 'static) {
        let mut callback = callback;
        let now = self.tick_nanos().unwrap_or(0);
        self.timers
            .push(Timer::new(period, now, move |_| callback()));
    }

    /// Current time in the node's clock, in nanoseconds. `None` under sim time
    /// before the first `/clock` sample — timers must not fire until then.
    fn tick_nanos(&mut self) -> Option<i64> {
        match self.clock.mode() {
            ClockMode::Wall => {
                // Saturate below i64::MAX: `Timer::fire`'s proven catch-up
                // (`roscmp_verify::next_fire_after`) requires `now < i64::MAX`.
                Some(i64::try_from(self.epoch.elapsed().as_nanos()).unwrap_or(i64::MAX - 1))
            }
            ClockMode::Sim => self.clock.poll().map(Time::as_nanos),
        }
    }

    /// Drain everything ready right now — fire due timers, dispatch pending
    /// subscription messages, and serve pending requests — in one pass.
    /// Returns the number of items handled.
    fn drain(&mut self) -> usize {
        let mut handled = 0;
        if let Some(now) = self.tick_nanos() {
            for timer in &mut self.timers {
                if timer.ready(now) {
                    timer.fire(self.dds.as_ref(), now);
                    handled += 1;
                }
            }
        }
        for sub in &mut self.subscriptions {
            handled += sub.poll();
        }
        for service in &mut self.services {
            handled += service.serve();
        }
        handled
    }

    /// Process ready work in one pass, blocking up to `timeout` for the first
    /// item if nothing is ready yet. Returns the number of items handled.
    /// Pass [`Duration::ZERO`] for a pure non-blocking drain.
    pub fn spin_once(&mut self, timeout: Duration) -> usize {
        let deadline = Instant::now() + timeout;
        loop {
            let handled = self.drain();
            if handled > 0 || self.is_shutdown() || Instant::now() >= deadline {
                return handled;
            }
            let remaining = deadline.saturating_duration_since(Instant::now());
            self.wait(remaining);
        }
    }

    /// Block until a registered reader is ready or `cap` (bounded by the next
    /// timer deadline / sim poll floor) elapses. Replaces the busy sleep: with
    /// no readers registered it degrades to a plain timed wait.
    fn wait(&mut self, cap: Duration) {
        let timeout = self.sleep_hint().min(cap);
        let _ = self.poll.poll(&mut self.events, Some(timeout));
    }

    /// Register a reader's data event source in the waitset under a fresh token.
    fn register_reader(
        poll: &mio::Poll,
        next_token: &mut usize,
        source: &mut dyn mio::event::Source,
    ) {
        let token = mio::Token(*next_token);
        *next_token += 1;
        poll.registry()
            .register(source, token, mio::Interest::READABLE)
            .expect("register reader with waitset");
    }

    /// Spin until [`Node::shutdown`] is signalled (e.g. from a ctrl-c handler
    /// wired to [`Node::shutdown_handle`]).
    pub fn spin(&mut self) {
        while !self.is_shutdown() {
            self.spin_once(Duration::from_millis(10));
        }
    }

    pub fn spin_for(&mut self, duration: Duration) {
        let deadline = Instant::now() + duration;
        while !self.is_shutdown() && Instant::now() < deadline {
            self.drain();
            let remaining = deadline.saturating_duration_since(Instant::now());
            self.wait(remaining);
        }
    }

    pub fn shutdown(&mut self) {
        self.shutdown.store(true, Ordering::SeqCst);
    }

    #[must_use]
    pub fn is_shutdown(&self) -> bool {
        self.shutdown.load(Ordering::SeqCst)
    }

    /// A shared shutdown flag. Install a ctrl-c handler that flips this to
    /// `true` (e.g. `ctrlc`-style) to stop [`Node::spin`] cleanly; no
    /// signal-handling dependency is pulled in here.
    #[must_use]
    pub fn shutdown_handle(&self) -> Arc<AtomicBool> {
        Arc::clone(&self.shutdown)
    }

    fn sleep_hint(&self) -> Duration {
        // Under sim time, wall sleeping never advances `/clock`; poll briefly
        // so fresh clock samples are picked up promptly.
        if self.clock.mode() == ClockMode::Sim {
            return Duration::from_millis(1);
        }
        let now = i64::try_from(self.epoch.elapsed().as_nanos()).unwrap_or(i64::MAX - 1);
        self.timers
            .iter()
            .map(|timer| Duration::from_nanos((timer.next_fire_nanos - now).max(0) as u64))
            .min()
            .unwrap_or_else(|| Duration::from_millis(10))
            .min(Duration::from_millis(10))
    }
}

/// Parsed `--ros-args` block(s).
#[derive(Debug, Default, Clone, PartialEq)]
struct RosArgs {
    node_name: Option<String>,
    namespace: Option<String>,
    remaps: Vec<(String, String)>,
    params: Vec<(String, ParameterValue)>,
    params_files: Vec<String>,
}

/// Parse standard ROS command-line arguments. Only tokens inside a `--ros-args`
/// block (terminated by `--` or end of input) are interpreted; unknown ROS
/// flags are ignored with a warning.
fn parse_ros_args(args: &[String]) -> RosArgs {
    let mut out = RosArgs::default();
    let mut in_block = false;
    let mut i = 0;
    while i < args.len() {
        let arg = args[i].as_str();
        if !in_block {
            if arg == "--ros-args" {
                in_block = true;
            }
            i += 1;
            continue;
        }
        match arg {
            "--" => in_block = false,
            "--ros-args" => {}
            "-r" | "--remap" => {
                if let Some(rule) = args.get(i + 1) {
                    i += 1;
                    apply_remap(&mut out, rule);
                }
            }
            "-p" | "--param" => {
                if let Some(rule) = args.get(i + 1) {
                    i += 1;
                    if let Some((name, value)) = split_assign(rule) {
                        out.params.push((name.to_string(), scalar_value(value)));
                    }
                }
            }
            "--params-file" => {
                if let Some(path) = args.get(i + 1) {
                    i += 1;
                    out.params_files.push(path.clone());
                }
            }
            other => eprintln!("roscmp: ignoring unknown --ros-args option '{other}'"),
        }
        i += 1;
    }
    out
}

fn apply_remap(out: &mut RosArgs, rule: &str) {
    let Some((from, to)) = split_assign(rule) else {
        eprintln!("roscmp: ignoring malformed remap '{rule}'");
        return;
    };
    match from {
        "__node" | "__name" => out.node_name = Some(to.to_string()),
        "__ns" => out.namespace = Some(to.to_string()),
        _ => out.remaps.push((from.to_string(), to.to_string())),
    }
}

/// Split a `lhs:=rhs` rule.
fn split_assign(rule: &str) -> Option<(&str, &str)> {
    rule.split_once(":=")
        .map(|(lhs, rhs)| (lhs.trim(), rhs.trim()))
}

/// A minimal YAML value: the subset ROS params files use.
enum Yaml {
    Scalar(String),
    List(Vec<String>),
    Map(BTreeMap<String, Yaml>),
}

/// Parse the ROS params-file subset and return the flattened `key -> value`
/// overrides that apply to a node with any of `sections` as its section key.
/// Wildcard (`/**`) sections are applied first so exact sections override them.
fn params_from_file(text: &str, sections: &[String]) -> BTreeMap<String, ParameterValue> {
    let root = parse_yaml(text);
    let mut out = BTreeMap::new();
    let mut collect = |key: &str| {
        if let Some(Yaml::Map(section)) = root.get(key) {
            if let Some(Yaml::Map(params)) = section.get("ros__parameters") {
                flatten_params("", params, &mut out);
            }
        }
    };
    collect("/**");
    collect("**");
    for key in sections.iter().filter(|k| !k.ends_with("**")) {
        collect(key);
    }
    out
}

/// Flatten nested maps into dotted parameter keys.
fn flatten_params(
    prefix: &str,
    map: &BTreeMap<String, Yaml>,
    out: &mut BTreeMap<String, ParameterValue>,
) {
    for (key, value) in map {
        let path = if prefix.is_empty() {
            key.clone()
        } else {
            format!("{prefix}.{key}")
        };
        match value {
            Yaml::Map(inner) => flatten_params(&path, inner, out),
            Yaml::Scalar(s) => {
                out.insert(path, scalar_value(s));
            }
            Yaml::List(items) => {
                out.insert(path, list_value(items));
            }
        }
    }
}

/// Indentation-based parser for the params-file YAML subset: nested maps,
/// scalars, and flat inline lists (`[a, b, c]`).
fn parse_yaml(text: &str) -> BTreeMap<String, Yaml> {
    let lines: Vec<(usize, &str)> = text
        .lines()
        .map(|line| match line.find('#') {
            Some(idx) => &line[..idx],
            None => line,
        })
        .filter(|line| !line.trim().is_empty())
        .map(|line| (indent_of(line), line.trim()))
        .collect();
    let mut pos = 0;
    parse_yaml_block(&lines, &mut pos, lines.first().map_or(0, |l| l.0))
}

fn parse_yaml_block(
    lines: &[(usize, &str)],
    pos: &mut usize,
    indent: usize,
) -> BTreeMap<String, Yaml> {
    let mut map = BTreeMap::new();
    while *pos < lines.len() {
        let (line_indent, content) = lines[*pos];
        if line_indent < indent {
            break;
        }
        // Deeper-than-expected indentation without a parent key: skip defensively.
        if line_indent > indent {
            *pos += 1;
            continue;
        }
        let Some((key, rest)) = content.split_once(':') else {
            *pos += 1;
            continue;
        };
        let key = key.trim().trim_matches('"').trim_matches('\'').to_string();
        let rest = rest.trim();
        *pos += 1;
        if rest.is_empty() {
            if *pos < lines.len() && lines[*pos].0 > indent {
                let child = parse_yaml_block(lines, pos, lines[*pos].0);
                map.insert(key, Yaml::Map(child));
            } else {
                map.insert(key, Yaml::Scalar(String::new()));
            }
        } else if rest.starts_with('[') {
            map.insert(key, Yaml::List(parse_flow_list(rest)));
        } else {
            map.insert(key, Yaml::Scalar(rest.to_string()));
        }
    }
    map
}

fn indent_of(line: &str) -> usize {
    line.len() - line.trim_start().len()
}

fn parse_flow_list(raw: &str) -> Vec<String> {
    let inner = raw.trim().trim_start_matches('[').trim_end_matches(']');
    if inner.trim().is_empty() {
        return Vec::new();
    }
    inner
        .split(',')
        .map(|item| item.trim().to_string())
        .collect()
}

/// Interpret a scalar token as bool, integer, double, or (quoted/other) string.
fn scalar_value(raw: &str) -> ParameterValue {
    let s = raw.trim();
    if s.len() >= 2
        && ((s.starts_with('"') && s.ends_with('"')) || (s.starts_with('\'') && s.ends_with('\'')))
    {
        return ParameterValue::String(s[1..s.len() - 1].to_string());
    }
    match s {
        "true" | "True" => return ParameterValue::Bool(true),
        "false" | "False" => return ParameterValue::Bool(false),
        _ => {}
    }
    if let Ok(i) = s.parse::<i64>() {
        return ParameterValue::Integer(i);
    }
    if let Ok(f) = s.parse::<f64>() {
        return ParameterValue::Double(f);
    }
    ParameterValue::String(s.to_string())
}

/// Classify a flat list into the narrowest ROS array parameter type.
#[allow(clippy::cast_precision_loss)]
fn list_value(items: &[String]) -> ParameterValue {
    let values: Vec<ParameterValue> = items.iter().map(|item| scalar_value(item)).collect();
    if !values.is_empty() && values.iter().all(|v| matches!(v, ParameterValue::Bool(_))) {
        return ParameterValue::BoolArray(
            values
                .iter()
                .map(|v| matches!(v, ParameterValue::Bool(true)))
                .collect(),
        );
    }
    if !values.is_empty()
        && values
            .iter()
            .all(|v| matches!(v, ParameterValue::Integer(_)))
    {
        return ParameterValue::IntegerArray(
            values
                .iter()
                .map(|v| match v {
                    ParameterValue::Integer(i) => *i,
                    _ => 0,
                })
                .collect(),
        );
    }
    if !values.is_empty()
        && values
            .iter()
            .all(|v| matches!(v, ParameterValue::Integer(_) | ParameterValue::Double(_)))
    {
        return ParameterValue::DoubleArray(
            values
                .iter()
                .map(|v| match v {
                    ParameterValue::Integer(i) => *i as f64,
                    ParameterValue::Double(d) => *d,
                    _ => 0.0,
                })
                .collect(),
        );
    }
    ParameterValue::StringArray(
        values
            .into_iter()
            .map(|v| match v {
                ParameterValue::String(s) => s,
                ParameterValue::Bool(b) => b.to_string(),
                ParameterValue::Integer(i) => i.to_string(),
                ParameterValue::Double(d) => d.to_string(),
                _ => String::new(),
            })
            .collect(),
    )
}

/// Type-erased callback subscription serviced by the executor.
trait Subscription {
    /// Dispatch every pending message to the callback; return how many.
    fn poll(&mut self) -> usize;
}

struct CallbackSub<M: CdrMsg> {
    sub: DdsSub<M>,
    callback: Box<dyn FnMut(M)>,
}

impl<M: CdrMsg> Subscription for CallbackSub<M> {
    fn poll(&mut self) -> usize {
        let mut n = 0;
        while let Some(msg) = self.sub.take() {
            (self.callback)(msg);
            n += 1;
        }
        n
    }
}

/// Type-erased service server serviced by the executor.
trait ServiceEndpoint {
    /// Serve every pending request with the handler; return how many.
    fn serve(&mut self) -> usize;
}

struct CallbackService<Req: CdrMsg, Resp: CdrMsg> {
    service: Service<Req, Resp>,
    handler: Box<dyn FnMut(&Req) -> Resp>,
}

impl<Req: CdrMsg, Resp: CdrMsg> ServiceEndpoint for CallbackService<Req, Resp> {
    fn serve(&mut self) -> usize {
        self.service.serve_pending(&mut self.handler)
    }
}

pub struct Timer {
    period_nanos: i64,
    next_fire_nanos: i64,
    callback: TimerCallback,
}

type TimerCallback = Box<dyn FnMut(Option<&Dds>)>;

impl Timer {
    fn new(period: Duration, now_nanos: i64, callback: impl FnMut(Option<&Dds>) + 'static) -> Self {
        let period_nanos = i64::try_from(period.as_nanos()).unwrap_or(i64::MAX).max(1);
        Self {
            period_nanos,
            next_fire_nanos: now_nanos.saturating_add(period_nanos),
            callback: Box::new(callback),
        }
    }

    fn ready(&self, now_nanos: i64) -> bool {
        now_nanos >= self.next_fire_nanos
    }

    fn fire(&mut self, dds: Option<&Dds>, now_nanos: i64) {
        (self.callback)(dds);
        // Machine-checked (Creusot): the catch-up terminates and never rewinds
        // the deadline, given `period_nanos >= 1` (guaranteed by `new`) and
        // `now_nanos < i64::MAX`. See `roscmp_verify::next_fire_after`.
        self.next_fire_nanos =
            roscmp_verify::next_fire_after(self.next_fire_nanos, self.period_nanos, now_nanos);
    }
}

fn normalize_namespace(namespace: &str) -> String {
    let trimmed = namespace.trim_matches('/');
    if trimmed.is_empty() {
        "/".to_string()
    } else {
        format!("/{trimmed}")
    }
}

#[cfg(test)]
mod tests {
    use std::cell::Cell;
    use std::rc::Rc;
    use std::time::Duration;

    use super::{
        list_value, normalize_namespace, params_from_file, parse_ros_args, scalar_value, Node,
    };
    use crate::parameters::ParameterValue;

    fn args(list: &[&str]) -> Vec<String> {
        list.iter().copied().map(String::from).collect()
    }

    #[test]
    fn node_resolves_relative_names_under_namespace() {
        let node = Node::without_dds("talker", "robot1");
        assert_eq!(node.resolve_name("cmd_vel"), "/robot1/cmd_vel");
        assert_eq!(node.resolve_name("/tf"), "/tf");
    }

    #[test]
    fn node_timer_fires_from_spin_once() {
        let mut node = Node::without_dds("timer_test", "");
        let count = Rc::new(Cell::new(0));
        let seen = Rc::clone(&count);
        node.create_test_timer(Duration::from_millis(0), move || {
            seen.set(seen.get() + 1);
        });
        assert_eq!(node.spin_once(Duration::ZERO), 1);
        assert_eq!(count.get(), 1);
    }

    #[test]
    fn namespaces_are_normalized() {
        assert_eq!(normalize_namespace(""), "/");
        assert_eq!(normalize_namespace("/robot/"), "/robot");
    }

    #[test]
    fn parses_remaps_node_and_namespace() {
        let parsed = parse_ros_args(&args(&[
            "prog",
            "--ros-args",
            "-r",
            "__node:=renamed",
            "-r",
            "__ns:=/robot",
            "-r",
            "/chatter:=/robot/chatter",
        ]));
        assert_eq!(parsed.node_name.as_deref(), Some("renamed"));
        assert_eq!(parsed.namespace.as_deref(), Some("/robot"));
        assert_eq!(
            parsed.remaps,
            vec![("/chatter".to_string(), "/robot/chatter".to_string())]
        );
    }

    #[test]
    fn parses_params_and_params_file() {
        let parsed = parse_ros_args(&args(&[
            "--ros-args",
            "-p",
            "use_sim_time:=true",
            "--param",
            "rate:=5",
            "--params-file",
            "cfg.yaml",
        ]));
        assert_eq!(
            parsed.params,
            vec![
                ("use_sim_time".to_string(), ParameterValue::Bool(true)),
                ("rate".to_string(), ParameterValue::Integer(5)),
            ]
        );
        assert_eq!(parsed.params_files, vec!["cfg.yaml".to_string()]);
    }

    #[test]
    fn terminator_stops_parsing_and_unknown_is_ignored() {
        let parsed = parse_ros_args(&args(&[
            "--ros-args",
            "--enclave",
            "/foo",
            "-p",
            "kept:=1",
            "--",
            "-p",
            "dropped:=2",
        ]));
        assert_eq!(
            parsed.params,
            vec![("kept".to_string(), ParameterValue::Integer(1))]
        );
    }

    #[test]
    fn scalar_typing() {
        assert_eq!(scalar_value("true"), ParameterValue::Bool(true));
        assert_eq!(scalar_value("-7"), ParameterValue::Integer(-7));
        assert_eq!(scalar_value("2.5"), ParameterValue::Double(2.5));
        assert_eq!(
            scalar_value("\"12\""),
            ParameterValue::String("12".to_string())
        );
        assert_eq!(
            scalar_value("hello"),
            ParameterValue::String("hello".to_string())
        );
    }

    #[test]
    fn list_typing() {
        assert_eq!(
            list_value(&["1".into(), "2".into(), "3".into()]),
            ParameterValue::IntegerArray(vec![1, 2, 3])
        );
        assert_eq!(
            list_value(&["1".into(), "2.5".into()]),
            ParameterValue::DoubleArray(vec![1.0, 2.5])
        );
        assert_eq!(
            list_value(&["a".into(), "b".into()]),
            ParameterValue::StringArray(vec!["a".into(), "b".into()])
        );
    }

    #[test]
    fn params_file_wildcard_section_and_nested_keys() {
        let yaml = "\
/**:
  ros__parameters:
    use_sim_time: true
talker:
  ros__parameters:
    rate: 10
    gains:
      p: 1.5
    ids: [1, 2, 3]
";
        let sections = vec!["/**".to_string(), "talker".to_string()];
        let params = params_from_file(yaml, &sections);
        assert_eq!(
            params.get("use_sim_time"),
            Some(&ParameterValue::Bool(true))
        );
        assert_eq!(params.get("rate"), Some(&ParameterValue::Integer(10)));
        assert_eq!(params.get("gains.p"), Some(&ParameterValue::Double(1.5)));
        assert_eq!(
            params.get("ids"),
            Some(&ParameterValue::IntegerArray(vec![1, 2, 3]))
        );
    }

    #[test]
    fn params_file_exact_section_overrides_wildcard() {
        let yaml = "\
/**:
  ros__parameters:
    rate: 1
talker:
  ros__parameters:
    rate: 99
";
        let sections = vec!["/**".to_string(), "talker".to_string()];
        let params = params_from_file(yaml, &sections);
        assert_eq!(params.get("rate"), Some(&ParameterValue::Integer(99)));
    }
}
