#!/usr/bin/env bash
# Build daq-plc-interface wheel into dist/.
#
#   bash scripts/build-wheel.sh
#   PKG_VERSION=0.1.4+sha.abc123 bash scripts/build-wheel.sh
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
cd "$PROJECT_ROOT"

BUILD_VENV="$PROJECT_ROOT/.build-venv"
if [[ -z "${VIRTUAL_ENV:-}" ]]; then
    if [[ ! -x "$BUILD_VENV/bin/python" ]]; then
        echo "==> Creating build venv at $BUILD_VENV ..."
        python3 -m venv "$BUILD_VENV"
    fi
    # shellcheck disable=SC1091
    source "$BUILD_VENV/bin/activate"
fi

if ! python -c "import build, setuptools, setuptools_scm" &>/dev/null; then
    echo "==> Installing build toolchain ..."
    pip install --quiet build setuptools setuptools-scm wheel
fi

if [[ -f ci-version.env ]]; then
    set -a
    # shellcheck disable=SC1091
    source ./ci-version.env
    set +a
fi

if [[ -n "${PKG_VERSION:-}" ]]; then
    export SETUPTOOLS_SCM_PRETEND_VERSION="$PKG_VERSION"
fi

echo "==> Building daq-plc-interface wheel (version=${SETUPTOOLS_SCM_PRETEND_VERSION:-scm})"
rm -rf dist build *.egg-info daq_plc_interface/*.egg-info
python -m build --wheel

echo "Build complete:"
ls -la dist/*.whl
