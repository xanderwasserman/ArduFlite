#!/usr/bin/env bash
# check_full_warnings.sh — compile EVERY translation unit with warnings on.
#
# check_target_warnings.sh covers the ~21 files that build without Arduino or
# ESP-IDF headers. This covers the other ~54, which can only be compiled with
# the full sketch include set — and which ./build.sh compiles with `-w`, so
# nothing about them is ever reported.
#
# It works by building once, capturing the exact command lines arduino-cli
# used, then REPLAYING each one with `-w` stripped and -Wall -Wextra added.
# Replaying rather than reconstructing is the point: the include paths, defines
# and response files are whatever the real build used, so there is no second
# description of the build to drift out of step with the first.
#
# Slow (one full build plus a syntax-only pass per file). Not part of
# run_tests.sh; run it before a release or after a broad refactor.
#
# Found on first run: four dropped [[nodiscard]] Status returns — including one
# that logged "Config erased from NVS" whether or not the erase succeeded — two
# -Wreorder constructors, and an unbounded array index. ADR-059.

set -uo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/../.."

BOARD="${1:-lolin}"
LOG="$(mktemp -t arduflite_build)"
trap 'rm -f "$LOG"' EXIT

echo "── Building ${BOARD} to capture compile commands ──"
if ! ./build.sh "$BOARD" > "$LOG" 2>&1; then
    echo "FAIL: the build itself failed; fix that before scanning for warnings."
    tail -20 "$LOG"
    exit 1
fi

SCAN="$(mktemp -t arduflite_scan)"
trap 'rm -f "$LOG" "$SCAN"' EXIT

python3 - "$LOG" > "$SCAN" <<'PYEOF'
import re, sys

commands = {}
for line in open(sys.argv[1], errors='ignore').read().split('\n'):
    if 'esp-elf-g++ ' not in line:
        continue
    # arduino-cli runs a PREPROCESS pass (-E) over every file as well as the
    # real compile. Preprocessing never emits these warnings, so scanning it
    # reports a clean tree no matter what the code says. This filter is the
    # difference between the check working and the check lying.
    if ' -E ' in line or ' -c ' not in line:
        continue

    match = re.search(r'(\S+\.cpp)\s+-o\s+(\S+\.o)\b', line)
    if not match:
        continue

    source = match.group(1)
    if '/sketch/' not in source:      # skip core and library sources
        continue

    name = source.split('/sketch/')[-1]
    if name in commands:
        continue

    command = line.replace(' -w ', ' ').replace(' -w ', ' ')
    command = command.replace('-o ' + match.group(2), '-o /dev/null')
    commands[name] = command + ' -Wall -Wextra'

for name, command in sorted(commands.items()):
    print(f'echo "@@@ {name}"')
    print(command + ' 2>&1 | grep "warning:" || true')

print(f'echo "@@@COUNT {len(commands)}"')
PYEOF

echo "── Recompiling with -Wall -Wextra ──"
OUTPUT="$(bash "$SCAN" 2>/dev/null)"

UNITS=$(echo "$OUTPUT" | sed -n 's/^@@@COUNT //p')
WARNINGS=$(echo "$OUTPUT" | grep -c "warning:" || true)

# A scan that compiled nothing would report zero warnings and look like a pass.
if [ -z "$UNITS" ] || [ "$UNITS" -lt 40 ]; then
    echo "FAIL: only ${UNITS:-0} translation units scanned — the command capture"
    echo "      broke (arduino-cli output format changed?). A zero-warning result"
    echo "      here would be meaningless, so this is a failure, not a pass."
    exit 1
fi

if [ "$WARNINGS" -gt 0 ]; then
    echo "$OUTPUT" | grep -B1 "warning:" | sed 's|'"$PWD"'/||'
    echo ""
    echo "FAIL: $WARNINGS warnings across $UNITS translation units."
    echo "      None of these appear in ./build.sh, which compiles with -w."
    exit 1
fi

echo "  $UNITS translation units, no warnings."
echo "Full warning checks passed."
