#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PORT="${PORT:-8501}"

cd "$ROOT_DIR"

if ! python3 -c "import pandas, streamlit" >/dev/null 2>&1; then
  echo "[ERROR] Streamlit dashboard dependencies are missing."
  echo "Install them first:"
  echo "  python3 -m pip install -r requirements-dashboard.txt"
  exit 1
fi

echo "[INFO] starting quantification dashboard"
echo "[INFO] repo_root=$ROOT_DIR"
echo "[INFO] url=http://localhost:$PORT"

exec streamlit run scripts/quant_dashboard.py \
  --server.port "$PORT" \
  -- \
  --repo-root "$ROOT_DIR"
