from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Dict, List, Optional

from boxfusion.floor_artifacts import display_floor_label
from boxfusion.query_api import RoomTopologyQueryAPI
from boxfusion.room_topology import _canonical_room_id


def _canonical_room_from_timeline(value: Any) -> Optional[str]:
    canonical = _canonical_room_id(value)
    if canonical is not None:
        return canonical
    if value is None:
        return None
    text = str(value).strip()
    if not text:
        return None
    if text.isdigit():
        return f"room_{int(text)}"
    return None


def _round_float(value: Any, digits: int = 3) -> Optional[float]:
    if value is None:
        return None
    return round(float(value), digits)


def load_replay_observations(timeline_json: Path, query_api: RoomTopologyQueryAPI) -> List[Dict[str, Any]]:
    rows = json.loads(Path(timeline_json).read_text(encoding="utf-8"))
    observations: List[Dict[str, Any]] = []
    for row in rows:
        if row.get("row_type") not in {None, "replay_frame"}:
            continue
        room_id = _canonical_room_from_timeline(row.get("current_room_id"))
        room_record = query_api.topology.get_room(room_id) if room_id else {}
        floor_id = room_record.get("floor_id") if room_record else None
        display_floor_id = room_record.get("display_floor_id") if room_record else None
        observations.append(
            {
                "frame_idx": row.get("frame_idx"),
                "timestamp": _round_float(row.get("timestamp")),
                "replay_frame_idx": row.get("replay_frame_idx"),
                "snapshot_idx": row.get("snapshot_idx"),
                "current_room_id": room_id,
                "current_floor_id": floor_id,
                "current_display_floor_id": display_floor_id,
                "current_floor_label": display_floor_label(floor_id, display_floor_id),
                "vector_map_path": row.get("vector_map_path"),
                "rgb_path": row.get("rgb_path"),
                "raw_row": dict(row),
            }
        )
    return observations


def extract_room_transition_observations(observations: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    transitions: List[Dict[str, Any]] = []
    previous_room_id = None
    previous_floor_id = None
    for observation in observations:
        room_id = observation.get("current_room_id")
        floor_id = observation.get("current_floor_id")
        if room_id is None:
            continue
        if room_id == previous_room_id and floor_id == previous_floor_id:
            continue
        transitions.append(
            {
                "frame_idx": observation.get("frame_idx"),
                "timestamp": observation.get("timestamp"),
                "room_id": room_id,
                "floor_id": floor_id,
                "display_floor_id": observation.get("current_display_floor_id"),
                "floor_label": observation.get("current_floor_label"),
            }
        )
        previous_room_id = room_id
        previous_floor_id = floor_id
    return transitions


def _target_query_kind(task_input: Dict[str, Any]) -> str:
    target = dict(task_input.get("target") or {})
    target_type = str(target.get("target_type") or "").strip().lower()
    if target_type in {"room", "anchor", "object"}:
        return target_type
    return "unknown"


def _unsupported_query_result(query_api: RoomTopologyQueryAPI, task_input: Dict[str, Any], route_policy: str) -> Dict[str, Any]:
    target_type = _target_query_kind(task_input)
    target = dict(task_input.get("target") or {})
    return {
        "success": False,
        "found": False,
        "query_type": "unsupported_task",
        "start_room_id": _canonical_room_id(task_input.get("start_room_id")),
        "resolved_goal_room_id": None,
        "target_resolution": {
            "success": False,
            "resolved": False,
            "target_type": target_type,
            "input": target,
            "resolved_room_id": None,
            "resolved_floor_id": None,
            "resolved_display_floor_id": None,
            "matched_entity_ids": [],
            "candidate_matches": [],
            "ambiguity": None,
            "failure_reason": "unsupported_target_type",
            "notes": [f"Unsupported target_type {target_type!r}."],
        },
        "route": {
            "attempted": False,
            "found": False,
            "route_policy": route_policy,
            "policy_settings": None,
            "failure_reason": "unsupported_target_type",
            "room_sequence": [],
            "edges": [],
            "used_relation_types": [],
            "edge_confidences": [],
            "hop_count": 0,
            "total_cost": None,
            "route_confidence": None,
            "score_summary": {"hop_count": 0, "total_cost": None, "route_confidence": None},
            "search_space": {},
        },
        "route_policy": route_policy,
        "failure_reason": "unsupported_target_type",
        "metadata": {"route_policy_preset": None, "available_route_policies": sorted(query_api.route_policy_presets)},
        "explanation": {
            "summary": f"Unsupported target_type {target_type!r}.",
            "target_summary": f"Unsupported target_type {target_type!r}.",
            "route_summary": "Route search was not attempted.",
            "hop_summaries": [],
            "notes": [],
        },
    }


def _resolve_query(query_api: RoomTopologyQueryAPI, task_input: Dict[str, Any], route_policy: str) -> Dict[str, Any]:
    start_room_id = task_input.get("start_room_id")
    target = dict(task_input.get("target") or {})
    query_kind = _target_query_kind(task_input)
    if query_kind == "room":
        return query_api.query_route(
            start_room_id=start_room_id,
            goal_room_id=target.get("goal_room_id"),
            route_policy=route_policy,
        )
    if query_kind == "anchor":
        return query_api.query_route_to_anchor(
            start_room_id=start_room_id,
            anchor_id=target.get("anchor_id"),
            route_policy=route_policy,
        )
    if query_kind == "object":
        query_kwargs = {"start_room_id": start_room_id, "route_policy": route_policy}
        if target.get("object_id") is not None:
            query_kwargs["object_id"] = target.get("object_id")
        if target.get("object_label") is not None:
            query_kwargs["object_label"] = target.get("object_label")
        return query_api.query_route_to_object(**query_kwargs)
    return _unsupported_query_result(query_api, task_input, route_policy)


def build_symbolic_plan(
    query_api: RoomTopologyQueryAPI,
    task_input: Dict[str, Any],
    route_policy: str = "balanced",
) -> Dict[str, Any]:
    query_result = _resolve_query(query_api, task_input, route_policy)
    query_kind = _target_query_kind(task_input)
    target = dict(task_input.get("target") or {})
    target_resolution = dict(query_result.get("target_resolution") or {})
    explanation = dict(query_result.get("explanation") or {})
    route = dict(query_result.get("route") or {})
    start_room_id = _canonical_room_id(task_input.get("start_room_id"))
    start_room = query_api.topology.get_room(start_room_id) or {}

    steps: List[Dict[str, Any]] = []
    if route.get("found"):
        for edge in route.get("edges", []):
            source_room = query_api.topology.get_room(edge.get("source_room_id")) or {}
            target_room = query_api.topology.get_room(edge.get("target_room_id")) or {}
            relation_type = str(edge.get("relation_type"))
            source_floor_id = source_room.get("floor_id")
            target_floor_id = target_room.get("floor_id")
            source_display_floor_id = source_room.get("display_floor_id")
            target_display_floor_id = target_room.get("display_floor_id")
            if relation_type == "vertical_transition" or source_floor_id != target_floor_id:
                step = {
                    "action": "use_vertical_transition",
                    "source_room_id": edge.get("source_room_id"),
                    "target_room_id": edge.get("target_room_id"),
                    "from_floor_id": source_floor_id,
                    "to_floor_id": target_floor_id,
                    "from_display_floor_id": source_display_floor_id,
                    "to_display_floor_id": target_display_floor_id,
                    "transition_ids": list((edge.get("metadata") or {}).get("transition_ids", [])),
                    "relation_type": relation_type,
                    "confidence": edge.get("confidence"),
                    "edge_cost": edge.get("edge_cost"),
                }
                transition_ids = ", ".join(step["transition_ids"]) or "vt_unknown"
                step["description"] = (
                    f"use_vertical_transition({transition_ids}) from {step['source_room_id']} "
                    f"on {display_floor_label(step['from_floor_id'], step['from_display_floor_id'])} "
                    f"to {step['target_room_id']} on {display_floor_label(step['to_floor_id'], step['to_display_floor_id'])}"
                )
            else:
                step = {
                    "action": "move_to_room",
                    "source_room_id": edge.get("source_room_id"),
                    "target_room_id": edge.get("target_room_id"),
                    "target_floor_id": target_floor_id,
                    "target_display_floor_id": target_display_floor_id,
                    "relation_type": relation_type,
                    "confidence": edge.get("confidence"),
                    "edge_cost": edge.get("edge_cost"),
                }
                step["description"] = (
                    f"move_to_room({step['target_room_id']}) via {relation_type} "
                    f"on {display_floor_label(step['target_floor_id'], step['target_display_floor_id'])}"
                )
            steps.append(step)

        resolved_goal_room_id = target_resolution.get("resolved_room_id")
        if query_kind == "room":
            terminal = {
                "action": "arrive_at_room",
                "target_room_id": resolved_goal_room_id,
                "target_floor_id": target_resolution.get("resolved_floor_id"),
                "target_display_floor_id": target_resolution.get("resolved_display_floor_id"),
                "description": f"arrive_at_room({resolved_goal_room_id})",
            }
        elif query_kind == "anchor":
            terminal = {
                "action": "arrive_at_anchor",
                "anchor_id": target.get("anchor_id"),
                "target_room_id": resolved_goal_room_id,
                "target_floor_id": target_resolution.get("resolved_floor_id"),
                "target_display_floor_id": target_resolution.get("resolved_display_floor_id"),
                "description": f"arrive_at_anchor({target.get('anchor_id')}) in {resolved_goal_room_id}",
            }
        else:
            terminal = {
                "action": "arrive_at_object",
                "object_id": target.get("object_id"),
                "object_label": target.get("object_label"),
                "target_room_id": resolved_goal_room_id,
                "target_floor_id": target_resolution.get("resolved_floor_id"),
                "target_display_floor_id": target_resolution.get("resolved_display_floor_id"),
                "description": (
                    f"arrive_at_object({target.get('object_id') or target.get('object_label')}) "
                    f"in {resolved_goal_room_id}"
                ),
            }
        steps.append(terminal)

    for index, step in enumerate(steps):
        step["step_index"] = index
        step["is_terminal"] = index == len(steps) - 1

    planning_reason = None
    if not route.get("found"):
        if route.get("attempted"):
            planning_reason = "route_unavailable"
        elif target_resolution.get("resolved") is False:
            planning_reason = "target_unresolved"
        else:
            planning_reason = str(query_result.get("failure_reason") or "planning_failed")

    return {
        "task_input": dict(task_input),
        "query_kind": query_kind,
        "route_policy": route_policy,
        "start_room_id": start_room_id,
        "start_floor_id": start_room.get("floor_id"),
        "start_display_floor_id": start_room.get("display_floor_id"),
        "target_resolution": target_resolution,
        "query_result": query_result,
        "plan_found": bool(route.get("found")),
        "planning_status": "success" if route.get("found") else "failure",
        "planning_reason": planning_reason,
        "route_summary": explanation.get("route_summary"),
        "target_summary": explanation.get("target_summary"),
        "hop_summaries": list(explanation.get("hop_summaries", [])),
        "floor_switches": list(explanation.get("floor_switches", [])),
        "symbolic_plan": {
            "room_sequence": list(route.get("room_sequence", [])),
            "relation_sequence": list(route.get("used_relation_types", [])),
            "hop_count": route.get("hop_count"),
            "route_confidence": route.get("route_confidence"),
            "total_cost": route.get("total_cost"),
            "steps": steps,
        },
    }


class VLNClosedLoopExecutor:
    def __init__(
        self,
        query_api: RoomTopologyQueryAPI,
        observations: List[Dict[str, Any]],
        *,
        topology_json: Optional[Path] = None,
        timeline_json: Optional[Path] = None,
    ) -> None:
        self.query_api = query_api
        self.observations = list(observations)
        self.topology_json = None if topology_json is None else str(topology_json)
        self.timeline_json = None if timeline_json is None else str(timeline_json)

    @classmethod
    def from_paths(cls, topology_json: Path, timeline_json: Path) -> "VLNClosedLoopExecutor":
        query_api = RoomTopologyQueryAPI.from_json(Path(topology_json))
        observations = load_replay_observations(Path(timeline_json), query_api)
        return cls(
            query_api=query_api,
            observations=observations,
            topology_json=Path(topology_json),
            timeline_json=Path(timeline_json),
        )

    def build_plan(self, task_input: Dict[str, Any], route_policy: str = "balanced") -> Dict[str, Any]:
        return build_symbolic_plan(self.query_api, task_input=task_input, route_policy=route_policy)

    def execute(
        self,
        task_input: Dict[str, Any],
        *,
        route_policy: str = "balanced",
        start_frame_idx: Optional[int] = None,
        end_frame_idx: Optional[int] = None,
        max_offroute_room_changes: int = 0,
    ) -> Dict[str, Any]:
        plan = self.build_plan(task_input=task_input, route_policy=route_policy)
        if not plan.get("plan_found"):
            return self._build_planning_failure(plan, start_frame_idx, end_frame_idx)

        filtered_observations = [
            observation
            for observation in self.observations
            if (start_frame_idx is None or (-1 if observation.get("frame_idx") is None else int(observation.get("frame_idx"))) >= int(start_frame_idx))
            and (end_frame_idx is None or (-1 if observation.get("frame_idx") is None else int(observation.get("frame_idx"))) <= int(end_frame_idx))
        ]
        first_room_observation = next(
            (observation for observation in filtered_observations if observation.get("current_room_id") is not None),
            None,
        )
        if first_room_observation is None:
            return self._build_execution_failure(
                plan,
                outcome_category="failure",
                outcome_reason="no_room_observations_in_window",
                trace=[],
                filtered_observations=filtered_observations,
                start_frame_idx=start_frame_idx,
                end_frame_idx=end_frame_idx,
            )
        if first_room_observation.get("current_room_id") != plan.get("start_room_id"):
            return self._build_execution_failure(
                plan,
                outcome_category="failure",
                outcome_reason="start_room_not_observed",
                trace=[
                    {
                        "event_type": "unexpected_initial_room",
                        "frame_idx": first_room_observation.get("frame_idx"),
                        "observed_room_id": first_room_observation.get("current_room_id"),
                        "expected_start_room_id": plan.get("start_room_id"),
                    }
                ],
                filtered_observations=filtered_observations,
                start_frame_idx=start_frame_idx,
                end_frame_idx=end_frame_idx,
            )

        steps = list(((plan.get("symbolic_plan") or {}).get("steps")) or [])
        trace: List[Dict[str, Any]] = []
        observed_room_sequence: List[str] = []
        floor_switch_events: List[Dict[str, Any]] = []
        completed_step_count = 0
        offroute_room_changes = 0
        last_room_id = None
        last_floor_id = None
        completion_observation: Optional[Dict[str, Any]] = None

        for observation in filtered_observations:
            room_id = observation.get("current_room_id")
            floor_id = observation.get("current_floor_id")
            if room_id is None:
                continue
            if room_id == last_room_id and floor_id == last_floor_id:
                continue

            trace.append(
                {
                    "event_type": "room_observed",
                    "frame_idx": observation.get("frame_idx"),
                    "timestamp": observation.get("timestamp"),
                    "observed_room_id": room_id,
                    "observed_floor_id": floor_id,
                    "observed_display_floor_id": observation.get("current_display_floor_id"),
                    "observed_floor_label": observation.get("current_floor_label"),
                }
            )
            observed_room_sequence.append(room_id)

            if last_floor_id is not None and floor_id != last_floor_id:
                floor_switch_payload = {
                    "event_type": "floor_switch_observed",
                    "frame_idx": observation.get("frame_idx"),
                    "from_room_id": last_room_id,
                    "to_room_id": room_id,
                    "from_floor_id": last_floor_id,
                    "to_floor_id": floor_id,
                    "from_display_floor_id": self._room_display_floor(last_room_id),
                    "to_display_floor_id": observation.get("current_display_floor_id"),
                }
                floor_switch_events.append(floor_switch_payload)
                trace.append(floor_switch_payload)

            advanced = False
            while completed_step_count < len(steps):
                step = steps[completed_step_count]
                validation = self._step_satisfied(step=step, observation=observation, plan=plan)
                if validation.get("target_invalidated"):
                    trace.append(
                        {
                            "event_type": "target_invalidated",
                            "frame_idx": observation.get("frame_idx"),
                            "step_index": step.get("step_index"),
                            "details": validation.get("validation_result"),
                        }
                    )
                    return self._build_execution_failure(
                        plan,
                        outcome_category="failure",
                        outcome_reason="target_invalidated",
                        trace=trace,
                        filtered_observations=filtered_observations,
                        start_frame_idx=start_frame_idx,
                        end_frame_idx=end_frame_idx,
                        observed_room_sequence=observed_room_sequence,
                        floor_switch_events=floor_switch_events,
                        completed_step_count=completed_step_count,
                    )
                if not validation.get("satisfied"):
                    break
                advanced = True
                trace.append(
                    {
                        "event_type": "step_completed",
                        "frame_idx": observation.get("frame_idx"),
                        "timestamp": observation.get("timestamp"),
                        "step_index": step.get("step_index"),
                        "action": step.get("action"),
                        "description": step.get("description"),
                        "observed_room_id": room_id,
                        "observed_floor_id": floor_id,
                    }
                )
                completed_step_count += 1
                if completed_step_count == len(steps):
                    completion_observation = observation
                    break

            if completion_observation is not None:
                break

            if not advanced and completed_step_count < len(steps):
                is_initial_start_observation = (
                    completed_step_count == 0
                    and room_id == plan.get("start_room_id")
                    and len(observed_room_sequence) == 1
                )
                if not is_initial_start_observation:
                    remaining_steps = steps[completed_step_count:]
                    divergence = self._classify_divergence(
                        observation=observation,
                        remaining_steps=remaining_steps,
                        max_offroute_room_changes=max_offroute_room_changes,
                        offroute_room_changes=offroute_room_changes,
                    )
                    offroute_room_changes = divergence.get("offroute_room_changes", offroute_room_changes)
                    if divergence.get("stop"):
                        trace.append(
                            {
                                "event_type": "execution_divergence",
                                "frame_idx": observation.get("frame_idx"),
                                "observed_room_id": room_id,
                                "expected_room_id": divergence.get("expected_room_id"),
                                "reason": divergence.get("reason"),
                                "details": divergence.get("details"),
                            }
                        )
                        return self._build_execution_failure(
                            plan,
                            outcome_category="replan_needed",
                            outcome_reason="execution_divergence",
                            trace=trace,
                            filtered_observations=filtered_observations,
                            start_frame_idx=start_frame_idx,
                            end_frame_idx=end_frame_idx,
                            observed_room_sequence=observed_room_sequence,
                            floor_switch_events=floor_switch_events,
                            completed_step_count=completed_step_count,
                        )

            last_room_id = room_id
            last_floor_id = floor_id

        if completion_observation is not None:
            return self._build_success_result(
                plan,
                trace=trace,
                filtered_observations=filtered_observations,
                completion_observation=completion_observation,
                start_frame_idx=start_frame_idx,
                end_frame_idx=end_frame_idx,
                observed_room_sequence=observed_room_sequence,
                floor_switch_events=floor_switch_events,
            )

        pending_step = steps[completed_step_count] if completed_step_count < len(steps) else None
        if pending_step and pending_step.get("action") == "use_vertical_transition":
            outcome_category = "failure"
            outcome_reason = "missing_vertical_transition"
        else:
            outcome_category = "replan_needed"
            outcome_reason = "replay_ended_before_completion"
        return self._build_execution_failure(
            plan,
            outcome_category=outcome_category,
            outcome_reason=outcome_reason,
            trace=trace,
            filtered_observations=filtered_observations,
            start_frame_idx=start_frame_idx,
            end_frame_idx=end_frame_idx,
            observed_room_sequence=observed_room_sequence,
            floor_switch_events=floor_switch_events,
            completed_step_count=completed_step_count,
        )

    def _build_planning_failure(
        self,
        plan: Dict[str, Any],
        start_frame_idx: Optional[int],
        end_frame_idx: Optional[int],
    ) -> Dict[str, Any]:
        return {
            "version": "0.1",
            "topology_json": self.topology_json,
            "timeline_json": self.timeline_json,
            "task_input": dict(plan.get("task_input") or {}),
            "route_policy": plan.get("route_policy"),
            "plan": plan,
            "execution_trace": [],
            "window": {
                "start_frame_idx": start_frame_idx,
                "end_frame_idx": end_frame_idx,
                "observations_in_window": 0,
            },
            "outcome": {
                "outcome_category": "failure",
                "outcome_reason": plan.get("planning_reason") or "planning_failed",
                "success": False,
                "completed_step_count": 0,
                "total_step_count": len(((plan.get("symbolic_plan") or {}).get("steps")) or []),
            },
        }

    def _build_execution_failure(
        self,
        plan: Dict[str, Any],
        *,
        outcome_category: str,
        outcome_reason: str,
        trace: List[Dict[str, Any]],
        filtered_observations: List[Dict[str, Any]],
        start_frame_idx: Optional[int],
        end_frame_idx: Optional[int],
        observed_room_sequence: Optional[List[str]] = None,
        floor_switch_events: Optional[List[Dict[str, Any]]] = None,
        completed_step_count: int = 0,
    ) -> Dict[str, Any]:
        return {
            "version": "0.1",
            "topology_json": self.topology_json,
            "timeline_json": self.timeline_json,
            "task_input": dict(plan.get("task_input") or {}),
            "route_policy": plan.get("route_policy"),
            "plan": plan,
            "execution_trace": trace,
            "window": {
                "start_frame_idx": start_frame_idx,
                "end_frame_idx": end_frame_idx,
                "observations_in_window": len(filtered_observations),
            },
            "observed_room_sequence": list(observed_room_sequence or []),
            "observed_floor_switches": list(floor_switch_events or []),
            "outcome": {
                "outcome_category": outcome_category,
                "outcome_reason": outcome_reason,
                "success": False,
                "completed_step_count": int(completed_step_count),
                "total_step_count": len(((plan.get("symbolic_plan") or {}).get("steps")) or []),
            },
        }

    def _build_success_result(
        self,
        plan: Dict[str, Any],
        *,
        trace: List[Dict[str, Any]],
        filtered_observations: List[Dict[str, Any]],
        completion_observation: Dict[str, Any],
        start_frame_idx: Optional[int],
        end_frame_idx: Optional[int],
        observed_room_sequence: List[str],
        floor_switch_events: List[Dict[str, Any]],
    ) -> Dict[str, Any]:
        return {
            "version": "0.1",
            "topology_json": self.topology_json,
            "timeline_json": self.timeline_json,
            "task_input": dict(plan.get("task_input") or {}),
            "route_policy": plan.get("route_policy"),
            "plan": plan,
            "execution_trace": trace,
            "window": {
                "start_frame_idx": start_frame_idx,
                "end_frame_idx": end_frame_idx,
                "observations_in_window": len(filtered_observations),
            },
            "observed_room_sequence": list(observed_room_sequence),
            "observed_floor_switches": list(floor_switch_events),
            "outcome": {
                "outcome_category": "success",
                "outcome_reason": "plan_completed",
                "success": True,
                "completed_step_count": len(((plan.get("symbolic_plan") or {}).get("steps")) or []),
                "total_step_count": len(((plan.get("symbolic_plan") or {}).get("steps")) or []),
                "completion_frame_idx": completion_observation.get("frame_idx"),
                "completion_room_id": completion_observation.get("current_room_id"),
                "completion_floor_id": completion_observation.get("current_floor_id"),
                "completion_display_floor_id": completion_observation.get("current_display_floor_id"),
            },
        }

    def _step_satisfied(self, *, step: Dict[str, Any], observation: Dict[str, Any], plan: Dict[str, Any]) -> Dict[str, Any]:
        action = step.get("action")
        observed_room_id = observation.get("current_room_id")
        observed_floor_id = observation.get("current_floor_id")
        if action == "move_to_room":
            return {"satisfied": observed_room_id == step.get("target_room_id")}
        if action == "use_vertical_transition":
            return {
                "satisfied": (
                    observed_room_id == step.get("target_room_id")
                    and observed_floor_id == step.get("to_floor_id")
                )
            }
        if action == "arrive_at_room":
            return {"satisfied": observed_room_id == step.get("target_room_id")}
        if action in {"arrive_at_anchor", "arrive_at_object"}:
            if observed_room_id != step.get("target_room_id"):
                return {"satisfied": False}
            validation = self._revalidate_target(plan)
            if not validation.get("resolved") or validation.get("resolved_room_id") != step.get("target_room_id"):
                return {"satisfied": False, "target_invalidated": True, "validation_result": validation}
            return {"satisfied": True, "validation_result": validation}
        return {"satisfied": False}

    def _revalidate_target(self, plan: Dict[str, Any]) -> Dict[str, Any]:
        task_input = dict(plan.get("task_input") or {})
        target = dict(task_input.get("target") or {})
        query_kind = str(plan.get("query_kind") or "")
        if query_kind == "room":
            return self.query_api.resolve_room_target(target.get("goal_room_id"))
        if query_kind == "anchor":
            return self.query_api.resolve_anchor_room(target.get("anchor_id"))
        if query_kind == "object":
            return self.query_api.resolve_object_room(
                object_id=target.get("object_id"),
                object_label=target.get("object_label"),
            )
        return {"success": False, "resolved": False, "resolved_room_id": None, "failure_reason": "unsupported_target_type"}

    def _room_display_floor(self, room_id: Optional[str]) -> Optional[str]:
        room = self.query_api.topology.get_room(room_id) if room_id else None
        return None if not room else room.get("display_floor_id")

    def _classify_divergence(
        self,
        *,
        observation: Dict[str, Any],
        remaining_steps: List[Dict[str, Any]],
        max_offroute_room_changes: int,
        offroute_room_changes: int,
    ) -> Dict[str, Any]:
        observed_room_id = observation.get("current_room_id")
        expected_room_id = self._next_expected_room_id(remaining_steps)
        remaining_room_ids = [
            room_id
            for room_id in (self._step_target_room_id(step) for step in remaining_steps)
            if room_id is not None
        ]
        if observed_room_id is None or expected_room_id is None:
            return {"stop": False, "offroute_room_changes": offroute_room_changes}
        if observed_room_id == expected_room_id:
            return {"stop": False, "offroute_room_changes": offroute_room_changes}
        if observed_room_id in remaining_room_ids[1:]:
            return {
                "stop": True,
                "reason": "step_skipped",
                "details": f"Observed {observed_room_id} before expected next room {expected_room_id}.",
                "expected_room_id": expected_room_id,
                "offroute_room_changes": offroute_room_changes,
            }
        offroute_room_changes += 1
        if offroute_room_changes > int(max_offroute_room_changes):
            return {
                "stop": True,
                "reason": "off_route_room",
                "details": f"Observed off-route room {observed_room_id}; remaining plan rooms are {remaining_room_ids}.",
                "expected_room_id": expected_room_id,
                "offroute_room_changes": offroute_room_changes,
            }
        return {"stop": False, "offroute_room_changes": offroute_room_changes}

    def _next_expected_room_id(self, remaining_steps: List[Dict[str, Any]]) -> Optional[str]:
        for step in remaining_steps:
            room_id = self._step_target_room_id(step)
            if room_id is not None:
                return room_id
        return None

    def _step_target_room_id(self, step: Dict[str, Any]) -> Optional[str]:
        if step.get("action") in {"move_to_room", "use_vertical_transition", "arrive_at_room", "arrive_at_anchor", "arrive_at_object"}:
            return step.get("target_room_id")
        return None
