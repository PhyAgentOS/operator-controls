#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
BUILD_ENV="${ROOT_DIR}/.venv_build"
DIST_DIR="${ROOT_DIR}/dist"
WORK_DIR="${ROOT_DIR}/build/pyinstaller"
SPEC_FILE="${SCRIPT_DIR}/controls.spec"
CONTROLLER="${DIST_DIR}/joint_trajectory_controller"
FAKE_PLANT="${DIST_DIR}/fake_joint_plant"

cleanup() {
  rm -rf "${BUILD_ENV}"
}
trap cleanup EXIT

expect_parse_error() {
  local name="$1"
  shift
  local output_file="${WORK_DIR}/${name}.log"
  local status

  set +e
  "$@" >"${output_file}" 2>&1
  status=$?
  set -e
  if [[ "${status}" -ne 2 ]]; then
    echo "ERROR: ${name} 参数解析应返回 2，实际返回 ${status}" >&2
    cat "${output_file}" >&2
    return 1
  fi
}

cd "${ROOT_DIR}"
rm -rf "${BUILD_ENV}" "${WORK_DIR}"
rm -f "${CONTROLLER}" "${FAKE_PLANT}"
mkdir -p "${DIST_DIR}" "${WORK_DIR}"

echo "==> 初始化隔离 Python 3.12 构建环境..."
uv venv --no-workspace "${BUILD_ENV}" --python 3.12

echo "==> 从 frozen lock 同步运行时与构建依赖..."
UV_PROJECT_ENVIRONMENT="${BUILD_ENV}" uv sync \
  --project "${ROOT_DIR}" \
  --frozen \
  --no-default-groups \
  --group build \
  --python 3.12

echo "==> 验证构建环境关键依赖..."
"${BUILD_ENV}/bin/python" - <<'PY'
import dora
import forge_msgs
import numpy
import pyarrow
import yaml

print("关键依赖导入成功")
PY

echo "==> 构建 PyInstaller 单文件产物..."
"${BUILD_ENV}/bin/pyinstaller" \
  --noconfirm \
  --clean \
  --distpath "${DIST_DIR}" \
  --workpath "${WORK_DIR}" \
  "${SPEC_FILE}"

for executable in "${CONTROLLER}" "${FAKE_PLANT}"; do
  if [[ ! -x "${executable}" ]]; then
    echo "ERROR: 未找到可执行产物 ${executable}" >&2
    exit 1
  fi
done

echo "==> 验证帮助信息和参数解析..."
"${CONTROLLER}" --help >/dev/null
"${FAKE_PLANT}" --help >/dev/null
expect_parse_error "controller-parse-error" "${CONTROLLER}"
expect_parse_error "fake-plant-parse-error" "${FAKE_PLANT}" --period-ms not-a-number

echo "==> 构建及验证成功"
ls -lh "${CONTROLLER}" "${FAKE_PLANT}"
