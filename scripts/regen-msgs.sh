#!/usr/bin/env bash
# Regenerate the runtime-dependent parts of roswell-ros2-compat/src/msgs.rs after editing
# src/cdr_runtime.rs or the codegen `to_cdr*` templates in src/codegen/rust.rs.
#
# msgs.rs was produced by `roswell --dds` over a ROS 2 interface tree that is not
# tracked in this repo, so a from-scratch regen is not reproducible here. What IS
# reproducible — and all that a cdr_runtime.rs edit needs — is:
#   1. re-embed src/cdr_runtime.rs verbatim (codegen include_str's it), and
#   2. keep each message's generated `to_cdr*` methods in sync with the codegen
#      templates (currently: add `to_cdr_xcdr2` alongside `to_cdr`).
# Both are done in place below, then `cargo fmt` normalizes the result exactly as
# a full `generate_dds` + fmt would. Idempotent.
set -euo pipefail
cd "$(dirname "$0")/.."

MSGS=roswell-ros2-compat/src/msgs.rs
RUNTIME=src/cdr_runtime.rs

python3 - "$MSGS" "$RUNTIME" <<'PY'
import re, sys
msgs_path, runtime_path = sys.argv[1], sys.argv[2]
msgs = open(msgs_path).read()
runtime = open(runtime_path).read().rstrip("\n") + "\n"

# 1. Replace the embedded CDR runtime block. It spans from its first line
#    (the `// CDR (Classic CDR ...` banner) up to — but not including — the
#    start of the ROS_TYPES section (`/// ABI of a ROS string:`).
start = msgs.index("// CDR (Classic CDR")
end = msgs.index("/// ABI of a ROS string:")
msgs = msgs[:start] + runtime + "\n" + msgs[end:]

# 2. Ensure every `to_cdr` is followed by its `to_cdr_xcdr2` twin (idempotent).
to_cdr = (
    "    pub fn to_cdr(&self, endian: Endian) -> Vec<u8> {\n"
    "        let mut w = Writer::new(endian);\n"
    "        self.serialize_into(&mut w);\n"
    "        w.finish()\n"
    "    }\n"
)
to_cdr2 = (
    "    pub fn to_cdr_xcdr2(&self, endian: Endian) -> Vec<u8> {\n"
    "        let mut w = Writer::with_encoding(endian, Encoding::Xcdr2);\n"
    "        self.serialize_into(&mut w);\n"
    "        w.finish()\n"
    "    }\n"
)
# Only insert where the twin is not already present immediately after.
out, i = [], 0
while True:
    j = msgs.find(to_cdr, i)
    if j == -1:
        out.append(msgs[i:])
        break
    k = j + len(to_cdr)
    out.append(msgs[i:k])
    if not msgs[k:].lstrip().startswith("pub fn to_cdr_xcdr2"):
        out.append(to_cdr2)
    i = k
open(msgs_path, "w").write("".join(out))
PY

cargo fmt -p roswell-ros2-compat
echo "regen: re-embedded $RUNTIME into $MSGS and synced to_cdr_xcdr2, then fmt'd."
