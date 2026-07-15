/*
 * roscmp.h — C-ABI for the roscmp runtime.
 *
 * Speak ROS2 (RTPS/DDS) from C or ctypes with no ROS installation, no codegen:
 * load a `.msg`/`.srv` at runtime, allocate its C-ABI struct memory, and
 * publish / subscribe / serve / call over real DDS. All message logic (parsing,
 * layout, CDR, QoS, transport, correlation) lives in Rust behind this boundary.
 *
 * Handles
 * -------
 *  Every object is addressed by an opaque, generation-counted RcmHandle (a
 *  uint64, not a pointer). Freeing a handle bumps its slot's generation, so a
 *  stale handle — use-after-free, double-free, use-after-shutdown — is reported
 *  as RCM_ERR_STALE_HANDLE instead of dereferencing dangling memory. Each handle
 *  also carries its kind, so passing a handle of the wrong kind is
 *  RCM_ERR_WRONG_KIND, never a type-confused access. A handle value of 0 is
 *  always invalid (it is the null/error sentinel returned by allocating calls).
 *
 * Conventions
 * -----------
 *  - Create/destroy in pairs. Destroying a context (rcm_shutdown) additionally
 *    invalidates every endpoint handle created against it (see below); type and
 *    message handles are independent of any context and outlive it.
 *  - Fallible functions return `int` (0 = success, negative = an RCM_ERR_* code)
 *    or a 0 / NULL sentinel. On failure, rcm_last_error() returns a
 *    human-readable message (thread-local; valid until the next failing call on
 *    the same thread).
 *  - Strings returned by rcm_*_json / rcm_*_name are heap-owned; free them with
 *    rcm_string_free(). The rcm_last_error() and rcm_version_string() strings are
 *    NOT freed by the caller.
 *  - A message's struct memory is allocated and freed in Rust (rcm_msg_alloc /
 *    _fini / _free). Its base pointer, for ctypes field access, comes from
 *    rcm_msg_data() and is valid only while the message handle is live. Memory is
 *    laid out per the platform C ABI, exactly as described by
 *    rcm_type_layout_json(), so a ctypes.Structure over the same layout aliases
 *    it field-for-field.
 *
 * Threading
 * ---------
 *  The handle table is internally synchronized, so any handle may be validated
 *  and used from any thread. The supported concurrency pattern is:
 *
 *    - One background thread drives rcm_wait() over a set of subscriber handles
 *      and rcm_take() on the readied one.
 *    - Other threads concurrently call rcm_publish() / rcm_call() / rcm_node() /
 *      rcm_graph_json() on the SAME context and its other handles.
 *
 *  This is safe: table lookups take a short internal lock (released before any
 *  encode/decode or DDS I/O), and each object then serializes its own operations
 *  under a per-object lock. rcm_wait() polls in brief non-blocking bursts and
 *  never holds the table lock across its sleep, so it does not stall a concurrent
 *  publisher.
 *
 *  What is NOT supported (undefined at the app level, though never memory-unsafe
 *  thanks to the handle table): driving the SAME handle from two threads at once
 *  — e.g. two threads calling rcm_take() on one subscriber, or rcm_msg_* on one
 *  message. Give each thread its own message buffer. rcm_shutdown() may race with
 *  in-flight calls on that context's children; the children simply start
 *  returning RCM_ERR_STALE_HANDLE — no crash — but coordinate shutdown for
 *  predictable results.
 */
#ifndef ROSCMP_H
#define ROSCMP_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- ABI version -------------------------------------------------------- */

/*
 * The ABI this header describes. Bumped on any incompatible change to the
 * signatures, struct layouts, or handle encoding. A loader must refuse to bind
 * if rcm_abi_version() disagrees with this constant. Version 2 is the
 * generation-counted-handle ABI; version 1 was the raw-opaque-pointer ABI.
 */
#define RCM_ABI_VERSION 2u

/* The ABI version implemented by the loaded library. */
uint32_t rcm_abi_version(void);

/* The crate version string (for diagnostics). Static; do NOT free. */
const char *rcm_version_string(void);

/* ---- status codes ------------------------------------------------------- */

#define RCM_OK 0
#define RCM_ERR (-1)              /* generic / unspecified failure           */
#define RCM_ERR_NULL_HANDLE (-2)  /* handle was 0                            */
#define RCM_ERR_STALE_HANDLE (-3) /* freed, double-freed, or shut down       */
#define RCM_ERR_WRONG_KIND (-4)   /* handle is of a different kind           */
#define RCM_ERR_TYPE_MISMATCH (-5)/* message type != endpoint type           */
#define RCM_ERR_ENCODE (-6)       /* message encode failed                   */
#define RCM_ERR_DECODE (-7)       /* message decode failed                   */
#define RCM_ERR_UNKNOWN_TOKEN (-8)/* reply token unknown / already answered  */

/* ---- handles ------------------------------------------------------------ */

/*
 * An opaque, generation-counted handle to any roscmp object (context, type,
 * publisher, subscriber, service, client, or message). 0 is never valid.
 */
typedef uint64_t RcmHandle;

/* ---- QoS ---------------------------------------------------------------- */

/*
 * A QoS descriptor. reliability: 0 = best-effort, 1 = reliable. durability:
 * 0 = volatile, 1 = transient-local. keep_all: nonzero keeps every sample
 * (ignoring depth). deadline_ms / lifespan_ms: negative means unset.
 */
typedef struct RcmQos {
  uint8_t reliability;
  uint8_t durability;
  uint8_t keep_all;
  uint32_t depth;
  int64_t deadline_ms;
  int64_t lifespan_ms;
} RcmQos;

/* A descriptor for a named preset: "default", "sensor_data", or "latched". */
RcmQos rcm_qos_preset(const char *name);

/* ---- errors ------------------------------------------------------------- */

/* Most recent error on this thread (empty if none). Do not free. */
const char *rcm_last_error(void);

/* Free a heap string returned by an rcm_*_json / rcm_*_name function. */
void rcm_string_free(char *s);

/* ---- context ------------------------------------------------------------ */

/* Create a DDS context on `domain`. Returns 0 on failure. */
RcmHandle rcm_init(int domain);

/*
 * Destroy a context and its participant/node advertisement, and invalidate
 * every publisher/subscriber/service/client handle created against it (each
 * becomes RCM_ERR_STALE_HANDLE on next use). Type and message handles are
 * unaffected. No-op on an already-stale/0 handle.
 */
void rcm_shutdown(RcmHandle ctx);

/* ---- types -------------------------------------------------------------- */

/*
 * Load a `.msg`/`.srv`/`.action`/`.idl` file plus optional dependency files
 * into a message type. `deps`/`n_deps` may be NULL/0. Returns 0 on failure.
 */
RcmHandle rcm_type_load(const char *msg_path, const char *const *deps, size_t n_deps);

/*
 * Load a `.srv` into request and response types, written to *out_req / *out_resp.
 * Returns 0 on success, negative on failure.
 */
int rcm_type_load_srv(const char *srv_path, const char *const *deps, size_t n_deps,
                      RcmHandle *out_req, RcmHandle *out_resp);

/*
 * JSON describing the whole dependency closure's C-ABI layout. Owned string
 * (free with rcm_string_free). Shape:
 *
 *   { "root": "pkg/Name", "dds_type": "pkg::msg::dds_::Name_",
 *     "size": N, "align": N,
 *     "messages": {
 *       "pkg/Name": {
 *         "size": N, "align": N,
 *         "fields": [
 *           { "name": "field", "offset": N,
 *             "multiplicity": "scalar" | "array" | "sequence",
 *             "array_len": N,               // only when multiplicity == "array"
 *             "element": {
 *               "size": N, "align": N,
 *               "kind": "prim" | "string" | "message",
 *               "prim": "i32",              // kind == "prim" (CDR stem, see below)
 *               "wide": false, "bound": N,  // kind == "string" (bound optional)
 *               "message": "pkg/Name"       // kind == "message" (key into messages)
 *             } } ] } } }
 *
 * prim stems: bool, u8, i8, u16, i16, u32, i32, u64, i64, f32, f64. A string or
 * sequence field is the {data,size,capacity} pointer/size/size triple in memory.
 */
char *rcm_type_layout_json(RcmHandle ty);

/* The ROS2 DDS type name. Owned string (free with rcm_string_free). NULL on error. */
char *rcm_type_dds_name(RcmHandle ty);

/* Free a type handle. 0 on success, negative if already stale / wrong kind. */
int rcm_type_free(RcmHandle ty);

/* ---- messages ----------------------------------------------------------- */

/*
 * Allocate zeroed, aligned memory for one message of type `ty` and fill it with
 * its `.msg` defaults, returning a message handle. Free with rcm_msg_free.
 * Returns 0 on error.
 */
RcmHandle rcm_msg_alloc(RcmHandle ty);

/*
 * The base pointer of a message's C-ABI struct memory, for ctypes field access.
 * Valid ONLY while the message handle is live (until rcm_msg_free). NULL on error.
 */
uint8_t *rcm_msg_data(RcmHandle msg);

/*
 * Free every string/sequence buffer the message owns (recursive, idempotent),
 * keeping the message re-usable. 0 on success, negative on a handle error.
 */
int rcm_msg_fini(RcmHandle msg);

/*
 * Finalize (if needed) and free a message's backing allocation, invalidating its
 * handle. 0 on success, negative if already freed / wrong kind.
 */
int rcm_msg_free(RcmHandle msg);

/*
 * Overwrite a PRIMITIVE {data,size,capacity} sequence triple located at byte
 * `offset` within message `msg` with `count` elements of `elem_size` bytes copied
 * from `src`, freeing any buffer it previously owned. Allocation stays in Rust so
 * rcm_msg_fini can free it. Returns 0 on success, negative on error. Primitive
 * elements only.
 */
int rcm_seq_assign(RcmHandle msg, size_t offset, size_t elem_size, size_t elem_align,
                   const uint8_t *src, size_t count);

/*
 * Overwrite a ROS string {data,size,capacity} triple located at byte `offset`
 * within message `msg` with the UTF-8 bytes src[..len], freeing any buffer it
 * previously owned. Allocation stays in Rust so rcm_msg_fini can free it. Returns
 * 0 on success, negative on error.
 */
int rcm_str_assign(RcmHandle msg, size_t offset, const uint8_t *src, size_t len);

/* ---- publish / subscribe ------------------------------------------------ */

/* Create a publisher on `topic` for `ty` (qos NULL = default preset). 0 on error. */
RcmHandle rcm_publisher(RcmHandle ctx, const char *topic, RcmHandle ty, const RcmQos *qos);

/*
 * Encode message `msg` and publish it. The message's type must match the
 * publisher's, else RCM_ERR_TYPE_MISMATCH. 0 on success, negative on error.
 */
int rcm_publish(RcmHandle pub, RcmHandle msg);

/* Pending publisher-side QoS events as a JSON array (see rcm_subscriber_events). */
char *rcm_publisher_events(RcmHandle pub);

/* Free a publisher handle. 0 on success, negative if already stale. */
int rcm_publisher_free(RcmHandle pub);

/* Create a subscriber on `topic` for `ty` (qos NULL = default preset). 0 on error. */
RcmHandle rcm_subscriber(RcmHandle ctx, const char *topic, RcmHandle ty, const RcmQos *qos);

/*
 * Take the next message into message `out` (reusable; its previous contents are
 * finalized first). The message's type must match the subscriber's, else
 * RCM_ERR_TYPE_MISMATCH. Returns 1 if a message was decoded, 0 if none available,
 * negative on error.
 */
int rcm_take(RcmHandle sub, RcmHandle out);

/*
 * Block up to `timeout_ms` until one of subs[0..n] has a message ready; returns
 * its index, or -1 on timeout. The readied message is delivered by the next
 * rcm_take() on that subscriber. Multiplexes many readers on one thread without
 * stalling a concurrent publisher (see "Threading").
 */
int rcm_wait(const RcmHandle *subs, size_t n, int timeout_ms);

/*
 * Pending subscriber-side QoS events as a JSON array. Owned string (free with
 * rcm_string_free). Each element: {"event": "...", ...}. The "incompatible_qos"
 * event carries "policy" (reliability|durability|deadline|liveliness|history|
 * other) and "count"; others carry their counts. NULL on error.
 */
char *rcm_subscriber_events(RcmHandle sub);

/* Free a subscriber handle. 0 on success, negative if already stale. */
int rcm_subscriber_free(RcmHandle sub);

/* ---- services ----------------------------------------------------------- */

/* Create a service server named `name`. 0 on error. */
RcmHandle rcm_service(RcmHandle ctx, const char *name, RcmHandle req_ty, RcmHandle resp_ty);

/*
 * Take the next request into message `out_req`, writing a correlation token to
 * *out_token. The message's type must match the service's request type, else
 * RCM_ERR_TYPE_MISMATCH. Returns 1 if a request was decoded, 0 if none, negative
 * on error. Answer with rcm_service_send_reply() using the token.
 */
int rcm_service_take_request(RcmHandle svc, RcmHandle out_req, uint64_t *out_token);

/*
 * Encode message `resp` and reply to `token`. The message's type must match the
 * service's response type, else RCM_ERR_TYPE_MISMATCH. 0 on success, negative if
 * the token is unknown (RCM_ERR_UNKNOWN_TOKEN) or on error.
 */
int rcm_service_send_reply(RcmHandle svc, uint64_t token, RcmHandle resp);

/* Free a service handle. 0 on success, negative if already stale. */
int rcm_service_free(RcmHandle svc);

/* Create a service client for `name`. 0 on error. */
RcmHandle rcm_client(RcmHandle ctx, const char *name, RcmHandle req_ty, RcmHandle resp_ty);

/*
 * Encode message `req`, send it, and block up to `timeout_ms` for the correlated
 * reply, decoded into message `resp_out` (finalized first). Both messages' types
 * must match the client's request/response types, else RCM_ERR_TYPE_MISMATCH.
 * Returns 1 on reply, 0 on timeout, negative on error.
 */
int rcm_call(RcmHandle client, RcmHandle req, RcmHandle resp_out, int timeout_ms);

/* Free a client handle. 0 on success, negative if already stale. */
int rcm_client_free(RcmHandle client);

/* ---- graph & node identity ---------------------------------------------- */

/*
 * Discover the ROS graph (listening `listen_ms` for node announcements) as JSON:
 *   { "topics":   [ { "name": "/t", "type": "pkg/msg/T" }, ... ],
 *     "services": [ { "name": "/s", "request_type": "...", "response_type": "..." }, ... ],
 *     "actions":  [ { "name": "/a", "channels": [ "send_goal", ... ] }, ... ],
 *     "nodes":    [ "/ns/node", ... ] }
 * Owned string (free with rcm_string_free). NULL on error.
 */
char *rcm_graph_json(RcmHandle ctx, int listen_ms);

/*
 * Advertise a ROS node `name` under `namespace` on this context (so it appears
 * in `ros2 node list`). Repeated calls add more nodes. 0 on success, <0 on error.
 */
int rcm_node(RcmHandle ctx, const char *name, const char *node_namespace);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* ROSCMP_H */
