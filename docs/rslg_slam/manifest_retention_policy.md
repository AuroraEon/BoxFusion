# RSLG-SLAM Manifest Retention Policy

This policy exists because RSLG-SLAM needs stable project contracts without letting every planning task add a permanent manifest. The task25c/task25d/task25e manifests are useful, but several of them are migration notes, gitignore notes, static inventories, or temporary validation plans. Those files should not all become permanent project truth.

## Minimal Long-Term Set

The preferred long-term manifest set is:

| File | Role | Update pattern |
| --- | --- | --- |
| `rslg_slam_manifest_index_v0_1.json` | Manifest index and anti-bloat policy | Rarely updated |
| `project_truth_manifest_v0_1.json` | Project identity, input contract, and claim boundaries | Stable/read-only unless explicitly authorized |
| `pipeline_contract_manifest_v0_1.json` | Layer names and pipeline contract | Stable/read-only unless explicitly authorized |
| `layer_artifacts_manifest_v0_1.json` | Layer artifact expectations and map distinctions | Stable with rare contract updates |
| `workspace_policy_manifest_v0_1.json` | Repository organization and protected workspace policies | Rarely updated |
| `protected_assets_manifest_v0_1.json` | Protected baseline and evidence paths | Stable, updated only when protected assets change |
| `validated_milestones_manifest_v0_1.json` | Current validated milestone truth | Frequently updated by validation tasks |
| `manifest_retention_plan_v0_1.json` | Retention and pruning plan for current docs/manifests | Updated while pruning is active, then stable |

Human-readable stable contract docs may remain alongside these manifests:

| File | Role |
| --- | --- |
| `project_contract.md` | Human-readable project contract |
| `pipeline_architecture.md` | Human-readable pipeline and layer contract |
| `workspace_contract.md` | Human-readable workspace contract |
| `manifest_retention_policy.md` | Human-readable retention policy |

## Stable Versus Frequently Updated

Stable files should be treated as read-only project contracts unless a task is explicitly authorized to change project truth. This includes project identity, layer naming, claim boundaries, protected assets, and core artifact contracts.

Frequently updated files are allowed to evolve when new validation evidence exists. At present, `validated_milestones_manifest_v0_1.json` is the main frequently updated long-term manifest.

## Temporary Planning Files

Planning files should be kept only while they actively guide implementation or cleanup. They should later be merged, archived, or deleted after their useful content is represented in code, tests, or a smaller canonical contract.

Examples:

| File family | Expected future action |
| --- | --- |
| Gitignore plan files | Delete after `.gitignore` is manually finalized or the policy is merged into workspace policy |
| Tool migration files | Merge into implemented `tools/rslg_pipeline/` structure or a smaller entrypoint map |
| Tool entrypoint mapping files | Keep during migration, then archive or delete once canonical modules and validators exist |
| Pipeline test plan files | Keep until real validators/tests exist, then merge stable test expectations into the pipeline contract or a future compact test contract |
| Cleanup plan manifest | Keep while cleanup is active, then merge remaining policy into this retention policy or workspace policy |
| Legacy inventory manifest | Keep until legacy scripts are migrated, archived, or clearly marked in code/docs |

## Deletion And Merge Preconditions

No file is deleted by this policy. A future cleanup task may delete or merge a file only when its preconditions in `manifest_retention_plan_v0_1.json` are satisfied.

Key preconditions:

- `gitignore_plan_manifest_v0_1.json` and `gitignore_plan.md` may be deleted only after the user manually finalizes `.gitignore`, or explicitly decides not to apply the candidate rules, and any useful policy is merged into `workspace_policy_manifest_v0_1.json`.
- `gitignore_candidate.patch` should not remain as a long-term doc. It may be deleted only after the user finalizes `.gitignore` or rejects the patch.
- `tool_migration_plan_manifest_v0_1.json` and `tool_migration_plan.md` may be deleted only after the migration plan is reflected in actual `tools/rslg_pipeline/` implementation or merged into `tool_entrypoint_mapping_manifest_v0_1.json`.
- `tool_entrypoint_mapping_manifest_v0_1.json` and `tool_entrypoint_mapping.md` may be archived or deleted only after canonical entrypoints, validators, and migration notes exist elsewhere.
- `rslg_pipeline_test_plan_manifest_v0_1.json` and `rslg_pipeline_test_plan.md` may be deleted only after real validators/tests are implemented and the remaining stable test contract is merged into the pipeline contract or a future compact test contract.
- `cleanup_plan_manifest_v0_1.json` may be deleted only after cleanup is complete, or after remaining policy is merged into this retention policy and workspace policy.
- `legacy_inventory_manifest_v0_1.json` may be deleted only after legacy scripts are migrated, archived, or clearly marked in code/docs.

## Future Task Rule

Future Codex tasks should update the minimum required files. Do not create a new long-term manifest unless an existing manifest cannot reasonably hold the information and the task is explicitly authorized to add one.

Ordinary tasks should prefer:

1. Update `validated_milestones_manifest_v0_1.json` when new validation truth is established.
2. Update the appropriate existing planning manifest while an implementation or cleanup is active.
3. Put task-specific evidence under `stage_outputs/.../tasks/<task_id>_<task_name>/`.
4. Avoid adding new long-term docs/manifests for single-task evidence.
