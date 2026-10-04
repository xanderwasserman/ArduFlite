#!/usr/bin/env bash
# update_vendor.sh — vendor mavlink/c_library_v2 at a given commit.
#
# The vendored files under src/third_party/mavlink/ change only through this
# script. It copies the root helpers and the common, standard and minimal
# dialects (common builds on the other two), and records the commit in VERSION.
#
# Usage: tools/mavlink/update_vendor.sh <commit-sha>
set -euo pipefail

COMMIT="${1:?usage: $0 <c_library_v2 commit sha>}"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
DEST="${ROOT}/src/third_party/mavlink"
TMP="$(mktemp -d)"
trap 'rm -rf "${TMP}"' EXIT

curl -fsSL "https://codeload.github.com/mavlink/c_library_v2/tar.gz/${COMMIT}" \
    | tar -xz -C "${TMP}"
SRC="$(find "${TMP}" -mindepth 1 -maxdepth 1 -type d)"

rm -rf "${DEST}"
mkdir -p "${DEST}"
cp "${SRC}"/*.h "${DEST}/"
for dialect in common standard minimal; do
    mkdir -p "${DEST}/${dialect}"
    find "${SRC}/${dialect}" -maxdepth 1 -name '*.h' ! -name 'testsuite.h' \
        -exec cp {} "${DEST}/${dialect}/" \;
done

cat > "${DEST}/VERSION" <<VERSION
${COMMIT}
VERSION

cat > "${DEST}/README.md" <<README
# Vendored MAVLink C library

Upstream: https://github.com/mavlink/c_library_v2 at commit \`${COMMIT}\`
(see \`VERSION\`). Root helpers plus the \`common\`, \`standard\` and \`minimal\`
dialects.

The generated MAVLink C library is distributed under the MIT licence
(https://mavlink.io/en/#license).

Do not edit these files. Update them with:

    tools/mavlink/update_vendor.sh <commit-sha>

Include them only through \`src/telemetry/mavlink/Mavlink.h\`.
README

echo "Vendored c_library_v2 @ ${COMMIT} into ${DEST}"
