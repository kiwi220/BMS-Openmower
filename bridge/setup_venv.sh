#!/usr/bin/env bash
# Creates the Python 3.12 environment for the aiobmsble bridge using uv
# (uv downloads a standalone CPython, no system packages needed).
#
#   bridge/setup_venv.sh [VENV_DIR]     default: ~/.local/share/bms_ble/venv
set -euo pipefail

VENV_DIR="${1:-$HOME/.local/share/bms_ble/venv}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if ! command -v uv >/dev/null 2>&1; then
  echo "uv not found, installing to ~/.local/bin ..."
  curl -LsSf https://astral.sh/uv/install.sh | sh
  export PATH="$HOME/.local/bin:$PATH"
fi

uv venv --python 3.12 "$VENV_DIR"
VIRTUAL_ENV="$VENV_DIR" uv pip install -r "$HERE/requirements.txt"
"$VENV_DIR/bin/python" -c "import aiobmsble, bleak; print('aiobmsble OK:', aiobmsble.__version__)"
echo
echo "Set in your config:  bridge_python: \"$VENV_DIR/bin/python\""
