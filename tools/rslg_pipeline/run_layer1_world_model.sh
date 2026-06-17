#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="/home/ws/workspace/BoxFusion"
PY="/home/ws/miniconda3/envs/boxfusion/bin/python"
cd "${REPO_ROOT}"

if [[ "${RSLG_LAYER1_ALLOW_RERUN:-0}" == "1" ]]; then
  exec "${PY}" -m tools.rslg_pipeline.build_world_model "$@"
fi

exec "${PY}" -m tools.rslg_pipeline.build_world_model --preflight-only "$@"
