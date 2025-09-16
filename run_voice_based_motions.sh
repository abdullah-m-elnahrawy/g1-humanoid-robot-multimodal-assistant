#!/usr/bin/env bash
# Build & Run: Voice-Based Motions
# - Non-interactive, one-liner friendly
# - Reuses build/ unless --clean is passed
# - Loads OPENAI_API_KEY from $XDG_CONFIG_HOME/openai/api_key or ~/.config/openai/api_key if not set
# - Exports OPENAI_REALTIME_MODEL, ROBOT_IFACE, and audio gate vars
# - Ensures motion runner binary name compatibility via symlink
set -euo pipefail

# ---------- Defaults ----------
DEFAULT_IFACE="eth0"
DEFAULT_MODEL="gpt-4o-realtime-preview"

# Multicast defaults (now read at runtime by the app; you can still override here if you want)
DEFAULT_MCAST_IP="239.168.123.161"
DEFAULT_MCAST_PORT="5555"

# Simple gate knobs (for your AudioProcessingPipeline fallback)
DEFAULT_VAD_THRESHOLD="900"
DEFAULT_VAD_START_FRAMES="3"

# Intent router defaults (runtime-tunable; app also reads config/runtime.env)
DEFAULT_INTENT_STRATEGY="openai_only" # openai_only | local_first | openai_first
DEFAULT_OPENAI_INTENT_TIMEOUT_MS="250" # used when openai_first

CONFIG_HOME="${XDG_CONFIG_HOME:-$HOME/.config}"
API_KEY_FILE="${CONFIG_HOME}/openai/api_key"

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
    export OPENAI_API_KEY="$(tr -d '\r' < "${API_KEY_FILE}" | sed -e 's/^[[:space:]]*//' -e 's/[[:space:]]*$//')"
    echo "Loaded OPENAI_API_KEY from ${API_KEY_FILE}"
  else
    echo "ERROR: OPENAI_API_KEY not set and ${API_KEY_FILE} not found."
    echo "Create the file with your key, e.g.:"
    echo " mkdir -p \"${CONFIG_HOME}/openai\" && echo 'sk-...' > \"${API_KEY_FILE}\""
    exit 1
  fi
fi

# ---------- Runtime env (read by the app; app also sets defaults if you forget) ----------
export ROBOT_IFACE="${INTERFACE_NAME}"

# (Optional) multicast hints — the app now reads config/runtime.env and allows per-source config.
export ROBOT_MCAST_IP="${ROBOT_MCAST_IP:-$DEFAULT_MCAST_IP}"
export ROBOT_MCAST_PORT="${ROBOT_MCAST_PORT:-$DEFAULT_MCAST_PORT}"

# Conversation/ASR defaults
export AUTO_RESPOND="${AUTO_RESPOND:-0}"
export WAKEWORD_REQUIRED="${WAKEWORD_REQUIRED:-0}"
export ASR_LANG="${ASR_LANG:-auto}"
export PRINT_TRANSCRIPT="${PRINT_TRANSCRIPT:-1}"

# Audio gate defaults
export VAD_THRESHOLD_RMS="${VAD_THRESHOLD_RMS:-$DEFAULT_VAD_THRESHOLD}"
export VAD_MIN_VOICE_FRMS="${VAD_MIN_VOICE_FRMS:-$DEFAULT_VAD_START_FRAMES}"

# Optional wake-word name (unused when WAKEWORD_REQUIRED=0)
export WAKEWORD="${WAKEWORD:-hasan}"

# Intent strategy (optional; can also be set in config/runtime.env without rebuild)
export INTENT_STRATEGY="${INTENT_STRATEGY:-$DEFAULT_INTENT_STRATEGY}"
export OPENAI_INTENT_TIMEOUT_MS="${OPENAI_INTENT_TIMEOUT_MS:-$DEFAULT_OPENAI_INTENT_TIMEOUT_MS}"

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
  echo " ${BIN_DIR}/${LOWER_MOTION}"
  echo " ${BIN_DIR}/${UPPER_MOTION}"
  exit 1
fi

# ---------- Run ----------
echo
echo "=============================================="
echo " OPENAI_REALTIME_MODEL = ${OPENAI_REALTIME_MODEL}"
echo " ROBOT_IFACE           = ${ROBOT_IFACE}"
echo " AUDIO_INPUT_SOURCE    = ${AUDIO_INPUT_SOURCE:-robot} (flip in config/runtime.env)"
echo " ROBOT_MCAST_IP:PORT   = ${ROBOT_MCAST_IP}:${ROBOT_MCAST_PORT}"
echo " AUTO_RESPOND          = ${AUTO_RESPOND}"
echo " WAKEWORD_REQUIRED     = ${WAKEWORD_REQUIRED}"
echo " ASR_LANG              = ${ASR_LANG}"
echo " PRINT_TRANSCRIPT      = ${PRINT_TRANSCRIPT}"
echo " VAD_THRESHOLD_RMS     = ${VAD_THRESHOLD_RMS}"
echo " VAD_MIN_VOICE_FRMS    = ${VAD_MIN_VOICE_FRMS}"
echo " WAKEWORD              = ${WAKEWORD}"
echo " INTENT_STRATEGY       = ${INTENT_STRATEGY}"
echo " OPENAI_INTENT_TIMEOUT_MS= ${OPENAI_INTENT_TIMEOUT_MS}"
echo "----------------------------------------------"
echo "Running voice interface…"
echo "=============================================="
echo
exec ./voice_interface "${INTERFACE_NAME}"
