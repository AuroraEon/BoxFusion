#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="/home/ws/workspace/BoxFusion"
PY="/home/ws/miniconda3/envs/boxfusion/bin/python"
cd "${REPO_ROOT}"
exec "${PY}" -m tools.rslg_pipeline.build_layer2_formal_artifacts --repo-root "${REPO_ROOT}" "$@"
