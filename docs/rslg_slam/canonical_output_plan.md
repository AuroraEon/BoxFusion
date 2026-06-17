# RSLG-SLAM Canonical Output Plan

This document defines the planning boundary between task evidence, candidate
artifacts, future final canonical artifacts, and runtime artifacts. It is a
human-readable plan, not a manifest, and it does not generate or promote any
artifact.

## Artifact Categories

Task evidence artifacts live under task-local generated-output directories such
as:

`stage_outputs/stage1_generalization/<scene_id>/tasks/<task_id>/`

They are useful for audit and validation, but they are reproducible,
deletable, not permanent project truth, and not final canonical outputs.

Candidate artifacts are non-final outputs used for interface and schema
validation. This includes Layer 2 vertical connector candidates, stable map
package candidates, object interface candidates, Layer 3 candidate route
contracts, and route plan previews. Candidate artifacts are not final route
planner inputs or runtime inputs. Real A* route generation must not consume
them unless a later task explicitly promotes or regenerates them against final
formal evidence.

Final canonical artifacts are future generated outputs that should exist only
after sufficient current Layer 1 / Layer 2 evidence is available. Task33 does
not create them.

Runtime artifacts are future Layer 4 packages and evidence, including
executable route packages, runtime input packages, RViz overlay inputs,
validation logs, and runtime reports. Task33 does not create them.

## Proposed Canonical Root

The proposed generated-output root for future final outputs is:

`stage_outputs/rslg_slam/00843-DYehNKdT76V/canonical/`

This root is separate from task evidence directories. It is also separate from
historical `clean_rerun` paths; task33 does not write into `clean_rerun` or
promote candidates there.

Future subdirectories should be:

| Subdirectory | Intended role |
| --- | --- |
| `layer0_input/` | Input references and provenance. |
| `layer1_world_model/` | World Model Layer outputs and provenance. |
| `layer2_formal_artifacts/` | Final formal artifacts consumed by Layer 3. |
| `layer3_navigation_interface/` | Final route contracts, planner requests, and route outputs. |
| `layer4_runtime_validation/` | Runtime packages, logs, overlays, and validation reports. |
| `manifests/` | Generated-output manifests for the canonical run. |
| `logs/` | Command and validation logs for the canonical run. |

Task33 does not create these directories except as a plan in task evidence.

## Candidate-To-Final Decision Matrix

| Candidate family | Decision | Reason |
| --- | --- | --- |
| Vertical connector candidate | `must_regenerate_from_current_layer1_outputs` | Candidate truth is useful, but real geometry and centerline evidence must be regenerated or verified from current World Model Layer/formal evidence. |
| Connector graph candidate | `must_regenerate_from_current_layer2_inputs` | It depends on final vertical connector and topology or planner graph inputs. |
| Cross-floor topology candidate | `must_regenerate_from_current_layer2_inputs` | It depends on final room topology, connector graph, and stable map/planner compatibility. |
| Stable occupancy map package candidate | `must_regenerate_from_current_layer1_outputs` | The current candidate is metadata-only and has no pixels, PGM, YAML, or final map metadata. |
| Object query resolution candidate | `must_regenerate_from_current_layer1_outputs` | Current evidence is validated milestone truth, not a regenerated object-resolution pass from current World Model Layer outputs. |
| Object approach candidate | `requires_runtime_or_planner_validation_before_final` | `generated_ring_037` is useful, but approach geometry and feasibility were not regenerated or revalidated. |
| Object interface package candidate | `must_regenerate_from_current_layer2_inputs` | It depends on final query-resolution and object-approach artifacts. |
| Candidate route contracts | `must_regenerate_from_final_layer2_inputs` | Current contracts depend on candidate Layer 2 artifacts. |
| Route plan previews | `must_regenerate_from_final_route_contracts_and_planner_inputs` | Current previews are not real planner output. |

## Minimum Final Artifact Set

For `cross_floor_room`, real route generation requires:

- final stable occupancy map package
- final stable map metadata
- final vertical connector artifact
- final connector graph artifact
- final cross-floor topology artifact
- final route planner graph or planner-ready topology package

For `cross_floor_object`, all `cross_floor_room` dependencies are required,
plus:

- final object query resolution artifact
- final object approach artifact
- final object interface package
- approach feasibility / clearance validation evidence

After final Layer 2 artifacts exist, Layer 3 still needs final route contracts,
a final route plan or planner request artifact, real A* route output, and an
executable route candidate package.

After an executable route exists, Layer 4 needs a runtime input package, RViz
overlay package, launch/config package, execution log, and validation report.

## One-Command Launch Implications

Future launch structure should be two-level:

| Phase script | Role |
| --- | --- |
| `run_layer0_input_check.sh` | Validate scene input references. |
| `run_layer1_world_model.sh` | Run or verify the World Model Layer. |
| `run_layer2_formal_artifacts.sh` | Generate final formal artifacts. |
| `run_layer3_navigation_interface.sh` | Generate final route contracts, planner requests, and routes. |
| `run_layer4_runtime_validation.sh` | Generate runtime input packages and run authorized validation. |

A future wrapper may be named `run_rslg_pipeline_full.sh`.

These scripts should not be implemented yet. Final Layer 2 artifacts and real
route generation are still blocked, and future launch scripts should consume
canonical outputs rather than task evidence directories.
