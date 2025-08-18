#!/usr/bin/env bash
# Build & Run: Voice-Based Motions
# - Non-interactive, one-liner friendly
# - Reuses build/ unless --clean is passed
# - Loads OPENAI_API_KEY from ~/.config/openai/api_key if not set
# - Exports OPENAI_REALTIME_MODEL, ROBOT_IFACE, and optional audio gate vars
# - Ensures motion runner binary name compatibility via symlink

set -euo pipefail

# ---------- Defaults ----------
DEFAULT_IFACE="eth0"
DEFAULT_MODEL="gpt-4o-realtime-preview"

# Robot mic multicast (exported for future use; current code uses #defines)
DEFAULT_MCAST_IP="239.168.123.161"
DEFAULT_MCAST_PORT="5555"

# Simple gate knobs (for your AudioProcessingPipeline fallback)
DEFAULT_VAD_THRESHOLD="800"
DEFAULT_VAD_START_FRAMES="3"

API_KEY_FILE="${HOME}/.config/openai/api_key"

# ---------- Args ----------
CLEAN=0
if [[ "${1:-}" == "--clean" ]]; then
  CLEAN=1
  shift
fi

INTERFACE_NAME="${1:-$DEFAULT_IFACE}"
MCAST_IP="${2:-$DEFAULT_MCAST_IP}"
MCAST_PORT="${3:-$DEFAULT_MCAST_PORT}"

# ---------- Paths ----------
ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"
BUILD_DIR="${ROOT_DIR}/build"
BIN_DIR="${BUILD_DIR}/bin"

# ---------- Model & API Key ----------
export OPENAI_REALTIME_MODEL="${OPENAI_REALTIME_MODEL:-$DEFAULT_MODEL}"

if [[ -z "${OPENAI_API_KEY:-}" ]]; then
  if [[ -r "${API_KEY_FILE}" ]]; then
    export OPENAI_API_KEY="$(cat "${API_KEY_FILE}")"
    echo "Loaded OPENAI_API_KEY from ${API_KEY_FILE}"
  else
    echo "ERROR: OPENAI_API_KEY not set and ${API_KEY_FILE} not found."
    echo "Export OPENAI_API_KEY or create ${API_KEY_FILE}."
    exit 1
  fi
fi

# ---------- Runtime env (read by your app where applicable) ----------
export ROBOT_IFACE="${INTERFACE_NAME}"
export ROBOT_MCAST_IP="${MCAST_IP}"
export ROBOT_MCAST_PORT="${MCAST_PORT}"
export VAD_THRESHOLD_RMS="${VAD_THRESHOLD_RMS:-$DEFAULT_VAD_THRESHOLD}"
export VAD_MIN_VOICE_FRMS="${VAD_MIN_VOICE_FRMS:-$DEFAULT_VAD_START_FRAMES}"

# Optional wake-word gate (set WAKEWORD_REQUIRED=1 to enforce)
export WAKEWORD="${WAKEWORD:-hasan}"

# ---------- Build ----------
if (( CLEAN )) && [[ -d "${BUILD_DIR}" ]]; then
  echo "Cleaning previous build..."
  rm -rf "${BUILD_DIR}"
fi

# Prefer explicit SDK include path if present
CMAKE_ARGS=()
if [[ -d "/opt/unitree_robotics/include" ]]; then
  CMAKE_ARGS+=(-DUNITREE_SDK2_INCLUDE_DIR=/opt/unitree_robotics/include)
fi
# Respect user overrides if already exported
if [[ -n "${UNITREE_SDK2_INCLUDE_DIR:-}" ]]; then
  CMAKE_ARGS+=(-DUNITREE_SDK2_INCLUDE_DIR="${UNITREE_SDK2_INCLUDE_DIR}")
fi
if [[ -n "${UNITREE_SDK2_ROOT:-}" ]]; then
  CMAKE_ARGS+=(-DUNITREE_SDK2_ROOT="${UNITREE_SDK2_ROOT}")
fi

echo "Configuring (cmake ${CMAKE_ARGS[*]:-}<none>)..."
cmake -S "${ROOT_DIR}" -B "${BUILD_DIR}" "${CMAKE_ARGS[@]:-}"

# Cross-platform core count
CORES="$(command -v nproc >/dev/null 2>&1 && nproc || true)"
if [[ -z "${CORES}" ]]; then
  CORES="$(command -v getconf >/dev/null 2>&1 && getconf _NPROCESSORS_ONLN || echo 2)"
fi

echo "Building with ${CORES} parallel jobs..."
cmake --build "${BUILD_DIR}" -- -j"${CORES}"

# ---------- Binary checks ----------
cd "${BIN_DIR}"

# voice_interface must exist
if [[ ! -x "./voice_interface" ]]; then
  echo "ERROR: ${BIN_DIR}/voice_interface not found or not executable."
  exit 1
fi

# Motion runner: handle both names for compatibility
LOWER_MOTION="abdullah_elnahrawy_g1_motions"
UPPER_MOTION="AbdullahElnahrawy_g1_motions"

if [[ -x "./${LOWER_MOTION}" && ! -e "./${UPPER_MOTION}" ]]; then
  ln -s "./${LOWER_MOTION}" "./${UPPER_MOTION}" || true
fi
if [[ ! -x "./${LOWER_MOTION}" && -x "./${UPPER_MOTION}" ]]; then
  ln -s "./${UPPER_MOTION}" "./${LOWER_MOTION}" || true
fi

if [[ ! -x "./${LOWER_MOTION}" && ! -x "./${UPPER_MOTION}" ]]; then
  echo "ERROR: motion runner binary not found."
  echo "Expected one of:"
  echo "  ${BIN_DIR}/${LOWER_MOTION}"
  echo "  ${BIN_DIR}/${UPPER_MOTION}"
  exit 1
fi

# ---------- Run ----------
echo
echo "=============================================="
echo " OPENAI_REALTIME_MODEL = ${OPENAI_REALTIME_MODEL}"
echo " ROBOT_IFACE           = ${ROBOT_IFACE}"
echo " ROBOT_MCAST_IP        = ${ROBOT_MCAST_IP}   (note: current code uses #defines)"
echo " ROBOT_MCAST_PORT      = ${ROBOT_MCAST_PORT} (note: current code uses #defines)"
echo " VAD_THRESHOLD_RMS     = ${VAD_THRESHOLD_RMS}"
echo " VAD_MIN_VOICE_FRMS    = ${VAD_MIN_VOICE_FRMS}"
echo " WAKEWORD_REQUIRED     = ${WAKEWORD_REQUIRED:-0}"
echo " WAKEWORD              = ${WAKEWORD}"
echo "----------------------------------------------"
echo "Running voice interface…"
echo "=============================================="
echo

exec ./voice_interface "${INTERFACE_NAME}"
