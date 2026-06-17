#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="/home/ws/workspace/BoxFusion"
PY="/home/ws/miniconda3/envs/boxfusion/bin/python"
cd "${REPO_ROOT}"
exec "${PY}" -m tools.rslg_pipeline.export_runtime_inputs --repo-root "${REPO_ROOT}" "$@"
