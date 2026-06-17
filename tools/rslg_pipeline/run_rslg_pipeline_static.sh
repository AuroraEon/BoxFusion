#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="/home/ws/workspace/BoxFusion"
PY="/home/ws/miniconda3/envs/boxfusion/bin/python"
cd "${REPO_ROOT}"

"${PY}" -m tools.rslg_pipeline.validate_artifacts --repo-root "${REPO_ROOT}"

if [[ "${RSLG_PIPELINE_REBUILD:-0}" == "1" ]]; then
  tools/rslg_pipeline/run_layer2_formal_artifacts.sh
  tools/rslg_pipeline/run_layer3_navigation_interface.sh
fi

tools/rslg_pipeline/run_layer4_runtime_validation_static.sh "$@"
