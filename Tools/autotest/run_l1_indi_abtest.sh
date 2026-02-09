#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

MISSION_FILE="${REPO_ROOT}/Tools/autotest/Generic_Missions/CMAC-loiter-unlim.txt"
VEHICLE_BIN="${REPO_ROOT}/build/sitl/bin/arduplane"

if [[ ! -x "${VEHICLE_BIN}" ]]; then
    echo "Missing ${VEHICLE_BIN}. Run './waf plane' first."
    exit 1
fi

if [[ ! -f "${MISSION_FILE}" ]]; then
    echo "Missing mission file: ${MISSION_FILE}"
    exit 1
fi

exec python3 "${REPO_ROOT}/Tools/autotest/run_l1_indi_abtest.py" \
    --vehicle-binary "${VEHICLE_BIN}" \
    --mission "${MISSION_FILE}" \
    "$@"
