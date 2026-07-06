# Preview Wrappers Retired (RSLG-SLAM)

RSLG-SLAM is the project name. `BoxFusion` is only a historical repository path.

## What was retired (task49c)

The preview-only, dry-run route contract / route plan wrapper surface was
**deleted** because it conflicted with the RouteResult-centered Layer 3
architecture and could mislead readers into thinking there were multiple formal
route surfaces.

Deleted source files:

- `tools/rslg_pipeline/build_route_contracts.py` (compatibility wrapper)
- `tools/rslg_pipeline/build_route_plans.py` (compatibility wrapper)
- `tools/rslg_pipeline/planning/route_contracts.py` (preview CLI + stub builder)
- `tools/rslg_pipeline/planning/route_plans.py` (preview CLI + dry-run builder)

These modules generated `*_preview` / `*_candidate` dry-run stubs. They did not
produce the current canonical final route artifacts and were not part of the
`RSLGRouteResult` route-interface surface. No useful pure function was unique to
them, so nothing was migrated.

## What was retired (task49d)

The `planning/wrap_current_routes_as_route_results.py` seed wrapper was
**deleted** in task49d. It wrapped the two fixed canonical seed routes into
`RSLGRouteResult` structures and had become the only route builder, which
conflicted with the goal of a generic static planner core. It is superseded by
`planning/route_planner.py`, which resolves target, room/floor route, connector,
and approach from canonical Layer 2/3 artifacts (with only the metric-path length
as a documented canonical fallback). See
`docs/rslg_slam_planner/GENERIC_STATIC_PLANNER_CORE.md`.

## Verification

The deleted modules are no longer importable:

```bash
/home/ws/miniconda3/envs/boxfusion/bin/python - <<'PY'
import importlib
for name in [
    "tools.rslg_pipeline.planning.route_contracts",
    "tools.rslg_pipeline.planning.route_plans",
    "tools.rslg_pipeline.build_route_contracts",
    "tools.rslg_pipeline.build_route_plans",
]:
    try:
        importlib.import_module(name)
        print("STILL_IMPORTABLE", name)
    except ModuleNotFoundError:
        print("OK_NOT_IMPORTABLE", name)
PY
```

All four report `OK_NOT_IMPORTABLE`.

## Current formal Layer 3 surface

```
QueryTask JSON
  -> plan_query_static.py / batch_plan_query_static.py
  -> RSLGRouteResult JSON
  -> audits/validate_route_result.py
```

Not:

- `build_route_contracts.py` preview
- `build_route_plans.py` preview
- `planning/route_contracts.py` preview CLI
- `planning/route_plans.py` preview CLI

## Note on canonical route contract / plan directories

The canonical `layer3_navigation_interface/route_contracts/` and
`route_plans/` JSON artifact directories are unrelated to the deleted preview
modules and are untouched. They remain read-only inputs wrapped into
`RSLGRouteResult`.
