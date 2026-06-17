# RSLG-SLAM 新对话上传文件清单

本清单用于把 RSLG-SLAM 项目状态交给新的 ChatGPT 对话。新 ChatGPT 通常不能读取本地仓库，因此需要用户上传或粘贴必要文件。

## 推荐最小上传

- `docs/rslg_slam/ai_handoff_context.md`

## 推荐更强上传

- `docs/rslg_slam/ai_handoff_context.md`
- `docs/rslg_slam/project_contract.md`
- `docs/rslg_slam/pipeline_architecture.md`
- `docs/rslg_slam/workspace_contract.md`
- `docs/rslg_slam/rslg_pipeline_skeleton.md`
- `docs/rslg_slam/rslg_pipeline_test_plan.md`

## 推荐可选 manifest 上传

- `docs/rslg_slam/manifests/project_truth_manifest_v0_1.json`
- `docs/rslg_slam/manifests/pipeline_contract_manifest_v0_1.json`
- `docs/rslg_slam/manifests/layer_artifacts_manifest_v0_1.json`
- `docs/rslg_slam/manifests/workspace_policy_manifest_v0_1.json`
- `docs/rslg_slam/manifests/validated_milestones_manifest_v0_1.json`

## 上传提醒

不要把所有 planning、cleanup、gitignore manifests 都上传到新对话，除非新任务明确与 cleanup、migration、retention 或 gitignore policy 有关。

新对话开始后，建议先让 AI 复述当前项目状态、当前 layer、禁止 claim 和下一任务编号；确认无误后，再让它生成下一条 Codex prompt。
