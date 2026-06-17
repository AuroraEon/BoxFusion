# RSLG-SLAM AI 交接上下文包

本文是给新 ChatGPT 对话使用的轻量交接摘要，来源于当前 `docs/rslg_slam/` 文档、核心 manifests、`tools/rslg_pipeline/` 当前状态，以及 task25i 至 task25l 的轻量任务报告。本文不是新的项目真相源，不应覆盖或替代 manifests。若本文与 manifests 冲突，以 manifests 和实际文件系统状态为准。

## 1. 项目名称与定位

项目名称是 RSLG-SLAM。

仓库路径是 `/home/ws/workspace/BoxFusion`。BoxFusion 只是历史代码路径，不应作为新项目文档中的项目名。

RSLG-SLAM 是 rich-semantic + light-geometry 的 semantic-topological world-model backend。它把 RGB-D 图像和提供的相机位姿转换为结构化 world-model artifacts，并进一步支持导航接口输出。

RSLG-SLAM 不是 dense reconstruction 系统，也不是完整 embodied navigation benchmark。真实部署时更准确的表述是：SLAM/localization front-end + RSLG-SLAM semantic-topological world-modeling backend + downstream routing/execution interface。

## 2. 输入边界与禁止 claim

场景侧输入边界：

- RGB 图像。
- depth 图像。
- provided pose，也就是数据集提供的相机位姿。
- 如适用，camera intrinsics/extrinsics 和 scene config metadata。

禁止作为 world-model source 的外部来源：

- external GT floorplan。
- external GT occupancy map。
- manual stair centerline。
- manual object target pose。
- simulator navmesh。

禁止 claim：

- 不 claim dataset-side SLAM/localization accuracy。
- 不 claim dense reconstruction。
- 不 claim physical stair climbing。
- 不 claim footstep planning。
- 不 claim gait control。
- 不 claim contact dynamics。
- 不 claim AMCL success。
- 不 claim real robot stair climbing。
- 不 claim full object-navigation benchmark。

## 3. Layer architecture

必须使用以下 layer 名称：

- Layer 0: Input Layer
- Layer 1: World Model Layer
- Layer 2: Formal Artifact Layer
- Layer 3: Navigation Interface Layer
- Layer 4: Runtime Validation Layer

Stage-A 只是历史实现入口，例如 `stage_a_demo.py`。不要把 Layer 1 叫做 “Stage-A Layer”，必须叫 `Layer 1: World Model Layer`。

## 4. Workspace policy

`docs/rslg_slam/` 存放项目 contracts、manifests，以及人类可读的项目真相说明。

`tools/rslg_pipeline/` 是新的 canonical pipeline skeleton 和 wrapper 位置。

`stage_outputs/` 是 generated output workspace，不是永久项目真相。它可以由用户备份、删除或重新生成。当前本地 `stage_outputs/` 可能只包含小型任务证据目录，不应要求历史 outputs 存在。

旧 00824 baseline 不再是当前 active protected baseline；如果存在，它只是历史生成证据，普通任务只读。旧 00843 `clean_rerun` 不是当前本地依赖，普通任务不得写入或依赖它。

## 5. Current historical route/object truth

Cross-floor route truth:

```text
room_2 on floor_1
-> room_3 on floor_1
-> vt_1 / vc_vt_1 connector
-> room_7 on floor_2
-> room_13
-> room_14
```

Transition edge truth:

- `vt_1_centerline_e001` is the real transition edge.
- `vt_1_centerline_e003` is not the transition edge.

Object truth:

- query: `curtain in room_14 on floor_2`
- object_id: `obj_175`
- object_label: `curtain`
- target_floor: `floor_2`
- target_room: `room_14`
- approach_candidate: `generated_ring_037`
- object_centroid_navigation_used: false

这些是 validated milestone truths，但历史 evidence path 可能已经不在当前本地仓库中。

## 6. Completed task summary

- task25c: project contract / manifest bootstrap。
- task25d: manifest refinement / gitignore plan / migration plan。
- task25e: tool entrypoint mapping / pipeline test plan。
- task25f: manifest retention / pruning plan。
- task25g: `tools/rslg_pipeline/` skeleton and no-op validators。
- task25h: first real static validator。
- task25i: first route-contract dry-run wrapper。
- task25j: first generated route contract stubs。
- task25k: route contract stub schema validator。
- task25l: route contract stub to candidate contract promotion dry-run。

## 7. Current layer progress

当前处于 `Layer 3: Navigation Interface Layer`。

- Layer 3.1 dry-run planning completed。
- Layer 3.2 stub generation completed。
- Layer 3.3 stub schema validation completed。
- Layer 3.4 stub-to-candidate promotion dry-run completed。

澄清：

- Layer 1 has not been rerun in the clean pipeline。
- Layer 2 real artifacts have not been regenerated in the clean pipeline。
- Layer 4 runtime validation has not been rerun in the clean pipeline。

## 8. Current `tools/rslg_pipeline/` state

重要模块与当前角色：

- `common.py`: repo、JSON、path、manifest、stat 等轻量公共工具。
- `artifact_registry.py`: manifest index 读取、manifest path 解析和引用摘要。
- `validate_artifacts.py`: static project validation。
- `build_world_model.py`: World Model Layer 未来入口占位，不是当前 Stage-A 替代品。
- `build_stable_maps.py`: stable occupancy map 未来 builder 占位。
- `build_vertical_connectors.py`: vertical connector artifact 未来 builder 占位。
- `build_object_interfaces.py`: object query / approach interface 未来 builder 占位。
- `build_route_contracts.py`: dry-run planning and route contract stub generation。
- `route_contract_schema.py`: route contract stub schema validation。
- `route_contract_promotion.py`: candidate promotion dry-run。
- `export_runtime_inputs.py`: runtime input export 未来入口占位，不启动 runtime。

已经超过 skeleton 的实现：

- `validate_artifacts.py`: static project validation。
- `build_route_contracts.py`: dry-run planning and route contract stub generation。
- `route_contract_schema.py`: route contract stub schema validation。
- `route_contract_promotion.py`: candidate promotion dry-run。

## 9. What is still missing

clean pipeline 中仍缺少的真实 artifacts：

- stable occupancy map package。
- vertical connector artifact。
- cross-floor topology artifact。
- object query resolution artifact。
- object approach candidate artifact。
- real A* waypoint route。
- executable route。
- runtime validation output。

## 10. Recommended next engineering task

下一个工程任务应为：

`task25n_candidate_route_contract_schema_dry_run`

原因：`task25m` 是本 AI handoff context pack，因此原本工程下一步应顺延编号为 task25n。

下一个任务应：

- 继续停留在 `Layer 3: Navigation Interface Layer`。
- 定义 candidate route contract schema dry-run validation。
- 验证 promotion preview 与未来 candidate contract schema 的兼容性。
- 不生成 final route contract。
- 不生成 A* route。
- 不要求 historical `stage_outputs` 存在。
- 不运行 Stage-A 或 runtime systems。

## 11. How to use this handoff in a new conversation

把这个文件粘贴或上传到新的 ChatGPT 对话中。

先要求新的 AI 复述当前项目状态和当前 Layer。只有当它正确复述后，再要求它生成下一条 Codex prompt。

新的 AI 不应假设自己可以读取本地 manifests，除非用户上传或粘贴了这些文件。

## 12. For Codex prompts

Codex 可以读取本地文件，所以未来 Codex prompts 应要求 Codex 先读取 `docs/rslg_slam/` 和核心 manifests，但仍必须验证实际 filesystem state。
