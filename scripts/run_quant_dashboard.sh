#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PORT="${PORT:-8501}"
ADDRESS="${ADDRESS:-0.0.0.0}"
DASHBOARD_PYTHON="${DASHBOARD_PYTHON:-python3}"

cd "$ROOT_DIR"

if [[ -x "$ROOT_DIR/.venv-dashboard/bin/python" ]] && \
   "$ROOT_DIR/.venv-dashboard/bin/python" -c \
     "import numpy, pandas, streamlit, altair" >/dev/null 2>&1; then
  DASHBOARD_PYTHON="$ROOT_DIR/.venv-dashboard/bin/python"
fi

if [[ -d "$ROOT_DIR/.dashboard-deps" ]]; then
  export PYTHONPATH="$ROOT_DIR/.dashboard-deps${PYTHONPATH:+:$PYTHONPATH}"
fi

if ! "$DASHBOARD_PYTHON" -c "import numpy, pandas, streamlit, altair" >/dev/null 2>&1; then
  echo "[ERROR] Streamlit dashboard dependencies are missing."
  echo "Install them first:"
  echo "  python3 -m pip install --target .dashboard-deps -r requirements-dashboard.txt"
  exit 1
fi

echo "[INFO] starting quantification dashboard"
echo "[INFO] repo_root=$ROOT_DIR"
echo "[INFO] url=http://${ADDRESS}:$PORT"

exec "$DASHBOARD_PYTHON" -m streamlit run scripts/quant_dashboard.py \
  --global.developmentMode false \
  --server.address "$ADDRESS" \
  --server.port "$PORT" \
  -- \
  --repo-root "$ROOT_DIR"
