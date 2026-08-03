#!/usr/bin/env bash

set -euo pipefail

usage() {
  echo "Usage: flash_spresense.sh SERIAL_PORT [--preflight|--execute]" >&2
  exit 2
}

if [[ $# -lt 1 || $# -gt 2 ]]; then
  usage
fi

ROOT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
PORT=$1
MODE=${2:---preflight}
ARTIFACT_DIR=${SPFC_ARTIFACT_DIR:-"${ROOT_DIR}/build/spresense-m1-pwbimu-gnss-gcs-artifacts"}
PROFILE=${SPFC_PROFILE:-spresense-m1-pwbimu-gnss-gcs}
MANIFEST="${ARTIFACT_DIR}/ARTIFACTS.manifest"
IMAGE="${ARTIFACT_DIR}/nuttx.spk"
WRITER="${ROOT_DIR}/modules/Spresense/sdk/tools/flash_writer/scripts/flash_writer.py"

if [[ ${MODE} != --preflight && ${MODE} != --execute ]]; then
  usage
fi
if [[ ! -f ${MANIFEST} || ! -f ${IMAGE} || ! -f ${WRITER} ]]; then
  echo "Flash artifact, manifest, or Sony writer is missing." >&2
  exit 1
fi

manifest_value() {
  local key=$1
  awk -F= -v key="${key}" '$1 == key { count++; value=$2 }
    END { if (count == 1) print value }' "${MANIFEST}"
}

if [[ $(manifest_value profile) != "${PROFILE}" ]]; then
  echo "Artifact profile does not match ${PROFILE}." >&2
  exit 1
fi
if [[ $(manifest_value project_tree) != clean ]]; then
  echo "Refusing dirty-tree artifact." >&2
  exit 1
fi
for contract in \
  'm1.gnss.builtin=disabled' \
  'm1.gnss.addon=required' \
  'm1.gnss.ram=required' \
  'm1.outputs=disabled' \
  'm1.arming=always-denied' \
  'm1.physical_write_expected=0'; do
  key=${contract%%=*}
  expected=${contract#*=}
  if [[ $(manifest_value "${key}") != "${expected}" ]]; then
    echo "Artifact safety contract mismatch: ${contract}" >&2
    exit 1
  fi
done

if [[ ${PROFILE} == spresense-m1-pwbimu-gnss-gcs ]]; then
  for contract in \
    'm1.pwbimu.addon=required' \
    'm1.pwbimu.device=/dev/imu0' \
    'm1.pwbimu.probe=one-bounded-sample' \
    'm1.pwbimu.bus=SPI5' \
    'm1.pwbimu.pinshare=eMMC' \
    'm1.pwbimu.link_guard=required' \
    'm1.sensor_fallback=disabled'; do
    key=${contract%%=*}
    expected=${contract#*=}
    if [[ $(manifest_value "${key}") != "${expected}" ]]; then
      echo "Artifact Multi-IMU contract mismatch: ${contract}" >&2
      exit 1
    fi
  done
fi

for artifact in nuttx.spk nuttx nuttx.map nuttx.config memory-layout.json; do
  path="${ARTIFACT_DIR}/${artifact}"
  expected=$(manifest_value "artifact.${artifact}.sha256")
  if [[ ! -f ${path} || -z ${expected} ]]; then
    echo "Artifact or manifest hash is missing: ${artifact}" >&2
    exit 1
  fi
  actual=$(shasum -a 256 "${path}" | awk '{print $1}')
  if [[ ${actual} != "${expected}" ]]; then
    echo "Artifact SHA-256 mismatch: ${artifact}" >&2
    exit 1
  fi
done
ACTUAL_SHA=$(shasum -a 256 "${IMAGE}" | awk '{print $1}')
if [[ ! -c ${PORT} || $(uname -s) == Darwin && ${PORT} != /dev/cu.* ]]; then
  echo "Use an available /dev/cu.* character device: ${PORT}" >&2
  exit 1
fi
case $(basename "${PORT}") in
  *Bluetooth*|*debug-console*)
    echo "Refusing non-Spresense serial port: ${PORT}" >&2
    exit 1
    ;;
esac
if command -v lsof >/dev/null 2>&1 &&
   [[ -n $(lsof "${PORT}" 2>/dev/null || true) ]]; then
  echo "Serial port is already open: ${PORT}" >&2
  exit 1
fi

PROJECT_COMMIT=$(manifest_value project_commit)
CURRENT_COMMIT=$(git -C "${ROOT_DIR}" rev-parse HEAD)
if [[ ${CURRENT_COMMIT} != "${PROJECT_COMMIT}" ||
      -n $(git -C "${ROOT_DIR}" status --porcelain) ]]; then
  echo "Repository HEAD/tree does not match the clean artifact." >&2
  exit 1
fi
if [[ ${MODE} == --preflight ]]; then
  printf 'flash_preflight=PASS port=%s project_commit=%s image_sha256=%s action=not-executed\n' \
    "${PORT}" "${PROJECT_COMMIT}" "${ACTUAL_SHA}"
  exit 0
fi
if [[ ${SPFC_FLASH_ACK:-} != "${PROJECT_COMMIT}" ]]; then
  echo "Refusing flash: set SPFC_FLASH_ACK=${PROJECT_COMMIT}." >&2
  exit 1
fi

printf 'flash=START port=%s project_commit=%s reset=dtr outputs=disabled\n' \
  "${PORT}" "${PROJECT_COMMIT}"
python3 -u "${WRITER}" -s -c "${PORT}" -d -n "${IMAGE}"
printf 'flash=COMPLETE port=%s project_commit=%s readback=not-supported\n' \
  "${PORT}" "${PROJECT_COMMIT}"
