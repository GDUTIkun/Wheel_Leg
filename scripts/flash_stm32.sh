#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

echo "STM32 flashing entrypoint placeholder."
echo "Put board-specific flashing commands under ${REPO_ROOT}/firmware/stm32 when the firmware project is added."
