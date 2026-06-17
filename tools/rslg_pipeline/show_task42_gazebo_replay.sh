#!/usr/bin/env bash
set -euo pipefail

ROOT="/home/ws/workspace/BoxFusion"
SCENE_ID="00843-DYehNKdT76V"
DEMO_DIR="${ROOT}/stage_outputs/rslg_slam/${SCENE_ID}/canonical/demo_evidence_pack"
ROOM_REPORT="${ROOT}/stage_outputs/rslg_slam/${SCENE_ID}/canonical/layer4_runtime_validation/validation_reports/task39_runtime_result_v0_1.json"
OBJECT_REPORT="${ROOT}/stage_outputs/rslg_slam/${SCENE_ID}/tasks/task41_authorized_object_route_runtime_validation/task41_object_route_runtime_result_v0_1.json"

if [[ "${RSLG_TASK42_ALLOW_RUNTIME:-0}" != "1" ]]; then
  echo "RSLG_TASK42_ALLOW_RUNTIME=1 is required for Gazebo showcase commands." >&2
  exit 2
fi

cd "${ROOT}"

if [[ ! -d "${DEMO_DIR}" ]]; then
  /home/ws/miniconda3/envs/boxfusion/bin/python tools/rslg_pipeline/finalize_task42_demo_evidence_pack.py \
    --live-not-run-reason replay_only_strategy_requested_by_showcase_wrapper
fi

echo "Task42 Gazebo replay context is ready."
echo "This wrapper does not rerun live route validation."
echo "Room-level Gazebo evidence: ${ROOM_REPORT}"
echo "Object-level Gazebo evidence: ${OBJECT_REPORT}"
echo "Demo evidence pack: ${DEMO_DIR}"
echo "To rerun live validation, use the task39/task41 authorized runtime scripts directly with their guard variables."
