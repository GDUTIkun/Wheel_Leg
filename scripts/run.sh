#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [[ $# -ne 1 ]]; then
  echo "usage: $0 {sim|sim_only|hw|rc|control_only}" >&2
  exit 1
fi

MODE="$1"

# shellcheck disable=SC1091
source "${SCRIPT_DIR}/env.sh"

case "${MODE}" in
  sim)
    exec "${SCRIPT_DIR}/run_host.sh" sim
    ;;
  sim_only)
    exec "${SCRIPT_DIR}/run_host.sh" sim_only
    ;;
  hw)
    exec "${SCRIPT_DIR}/run_pi.sh" hw
    ;;
  rc)
    exec "${SCRIPT_DIR}/run_pi.sh" rc
    ;;
  control_only)
    exec "${SCRIPT_DIR}/run_host.sh" control_only
    ;;
  *)
    echo "unknown mode: ${MODE}" >&2
    exit 1
    ;;
esac
