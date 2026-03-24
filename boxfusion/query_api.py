from __future__ import annotations

from copy import deepcopy
from pathlib import Path
from typing import Any, Dict, List, Optional

from boxfusion.floor_artifacts import display_floor_label
from boxfusion.room_topology import (
    RoomTopology,
    _canonical_anchor_id,
    _canonical_object_id,
    _canonical_room_id,
)
from boxfusion.scene_graph_builder import normalize_open_vocab_label


DEFAULT_QUERY_ROUTE_POLICIES: Dict[str, Dict[str, Any]] = {
    "strict": {
        "allowed_relations": ["transition", "adjacent", "vertical_transition"],
        "min_conf": 0.35,
        "max_candidate_paths": 1,
        "description": "Prefer stronger traversable links, including explicit cross-floor links, and reject weak fallback edges.",
    },
    "balanced": {
        "allowed_relations": ["transition", "adjacent", "possible_connection", "vertical_transition"],
        "min_conf": 0.0,
        "max_candidate_paths": 1,
        "description": "Default room-level routing with conservative-but-usable weak-edge fallback and explicit cross-floor links.",
    },
    "exploratory": {
        "allowed_relations": ["transition", "adjacent", "possible_connection", "vertical_transition"],
        "min_conf": 0.0,
        "max_candidate_paths": 3,
        "description": "Keep weak fallback edges, preserve explicit cross-floor links, and surface alternate candidate routes for inspection.",
    },
}


def _round_float(value: Any, digits: int = 3) -> Optional[float]:
    if value is None:
        return None
    return round(float(value), digits)


def _sanitize_label_token(value: Any) -> Optional[str]:
    if value is None:
        return None
    text = str(value).strip().lower().replace("-", " ")
    if not text:
        return None
    return "_".join(text.split())


class RoomTopologyQueryAPI:
    def __init__(
        self,
        topology: RoomTopology,
        route_policy_presets: Optional[Dict[str, Dict[str, Any]]] = None,
    ) -> None:
        self.topology = topology
        self.route_policy_presets = deepcopy(route_policy_presets or DEFAULT_QUERY_ROUTE_POLICIES)

    @classmethod
    def from_json(cls, path: Path) -> "RoomTopologyQueryAPI":
        return cls(RoomTopology.from_json(Path(path)))

    def resolve_anchor_room(self, anchor_id: Any) -> Dict[str, Any]:
        canonical_anchor_id = _canonical_anchor_id(anchor_id)
        resolution = self._make_resolution_result(
            target_type="anchor",
            input_payload={"anchor_id": anchor_id},
            matched_entity_ids=[],
            candidate_matches=[],
        )
        if canonical_anchor_id is None:
            return self._finalize_resolution(
                resolution,
                resolved=False,
                failure_reason="invalid_anchor_id",
                notes=["Anchor id could not be canonicalized."],
            )

        anchor_record = self.topology.get_anchor(canonical_anchor_id)
        room_id = self.topology.get_room_of_anchor(canonical_anchor_id)
        candidate_matches = [self._serialize_anchor_candidate(anchor_record)] if anchor_record else []
        resolution["candidate_matches"] = candidate_matches
        resolution["matched_entity_ids"] = [canonical_anchor_id] if anchor_record or room_id else []
        if anchor_record is None and room_id is None:
            return self._finalize_resolution(
                resolution,
                resolved=False,
                failure_reason="anchor_not_found",
                notes=[f"Anchor {canonical_anchor_id} is not present in the topology export."],
            )
        if room_id is None or room_id not in self.topology.graph.nodes:
            return self._finalize_resolution(
                resolution,
                resolved=False,
                failure_reason="anchor_has_no_resolved_room",
                notes=[f"Anchor {canonical_anchor_id} exists but does not map to a valid room node."],
            )
        room_record = self.topology.get_room(room_id) or {}
        return self._finalize_resolution(
            resolution,
            resolved=True,
            resolved_room_id=room_id,
            resolved_floor_id=room_record.get("floor_id"),
            resolved_display_floor_id=room_record.get("display_floor_id"),
            notes=[f"Anchor {canonical_anchor_id} resolves to {room_id}."],
        )

    def resolve_room_target(self, goal_room_id: Any) -> Dict[str, Any]:
        return self._resolve_room_target(goal_room_id)

    def resolve_object_room(
        self,
        object_id: Any = None,
        object_label: Optional[str] = None,
    ) -> Dict[str, Any]:
        if object_id is not None:
            return self._resolve_object_by_id(object_id, object_label=object_label)
        if object_label is not None:
            return self._resolve_object_by_label(object_label)
        resolution = self._make_resolution_result(
            target_type="object",
            input_payload={"object_id": object_id, "object_label": object_label},
            matched_entity_ids=[],
            candidate_matches=[],
        )
        return self._finalize_resolution(
            resolution,
            resolved=False,
            failure_reason="object_selector_missing",
            notes=["Provide either object_id or object_label."],
        )

    def query_route(
        self,
        start_room_id: Any,
        goal_room_id: Any,
        route_policy: str = "balanced",
    ) -> Dict[str, Any]:
        target_resolution = self._resolve_room_target(goal_room_id)
        return self._execute_route_query(
            query_type="route_to_room",
            start_room_id=start_room_id,
            target_resolution=target_resolution,
            route_policy=route_policy,
            query_input={
                "start_room_id": start_room_id,
                "goal_room_id": goal_room_id,
                "route_policy": route_policy,
            },
        )

    def query_route_to_anchor(
        self,
        start_room_id: Any,
        anchor_id: Any,
        route_policy: str = "balanced",
    ) -> Dict[str, Any]:
        target_resolution = self.resolve_anchor_room(anchor_id)
        return self._execute_route_query(
            query_type="route_to_anchor",
            start_room_id=start_room_id,
            target_resolution=target_resolution,
            route_policy=route_policy,
            query_input={
                "start_room_id": start_room_id,
                "anchor_id": anchor_id,
                "route_policy": route_policy,
            },
        )

    def query_route_to_object(
        self,
        start_room_id: Any,
        object_id: Any = None,
        object_label: Optional[str] = None,
        route_policy: str = "balanced",
    ) -> Dict[str, Any]:
        target_resolution = self.resolve_object_room(object_id=object_id, object_label=object_label)
        return self._execute_route_query(
            query_type="route_to_object",
            start_room_id=start_room_id,
            target_resolution=target_resolution,
            route_policy=route_policy,
            query_input={
                "start_room_id": start_room_id,
                "object_id": object_id,
                "object_label": object_label,
                "route_policy": route_policy,
            },
        )

    def explain_route(self, query_result: Dict[str, Any]) -> Dict[str, Any]:
        target_summary = self.summarize_target_resolution(query_result=query_result)
        route = dict(query_result.get("route") or {})
        start_room_id = query_result.get("start_room_id")
        resolved_goal_room_id = query_result.get("resolved_goal_room_id")
        route_policy = query_result.get("route_policy")

        if not route.get("attempted"):
            summary = target_summary["summary"]
            if query_result.get("failure_reason") == "invalid_route_policy":
                summary = f"Route policy {route_policy!r} is not defined."
            return {
                "summary": summary,
                "target_summary": target_summary["summary"],
                "route_summary": "Route search was not attempted.",
                "hop_summaries": [],
                "notes": list(target_summary.get("notes", [])),
            }

        if not route.get("found"):
            return {
                "summary": (
                    f"No route from {start_room_id} to {resolved_goal_room_id} under "
                    f"the {route_policy} policy."
                ),
                "target_summary": target_summary["summary"],
                "route_summary": f"Route search failed with reason: {route.get('failure_reason')}.",
                "hop_summaries": [],
                "notes": list(target_summary.get("notes", [])),
            }

        relation_summary = " -> ".join(route.get("used_relation_types", [])) or "same_room"
        floor_switches = []
        hop_summaries = []
        for edge in route.get("edges", []):
            source_room = self.topology.get_room(edge["source_room_id"]) or {}
            target_room = self.topology.get_room(edge["target_room_id"]) or {}
            source_floor_label = display_floor_label(
                source_room.get("floor_id"),
                source_room.get("display_floor_id"),
            )
            target_floor_label = display_floor_label(
                target_room.get("floor_id"),
                target_room.get("display_floor_id"),
            )
            if edge.get("relation_type") == "vertical_transition" or source_room.get("floor_id") != target_room.get("floor_id"):
                switch_payload = {
                    "source_room_id": edge["source_room_id"],
                    "target_room_id": edge["target_room_id"],
                    "from_floor_id": source_room.get("floor_id"),
                    "to_floor_id": target_room.get("floor_id"),
                    "from_display_floor_id": source_room.get("display_floor_id"),
                    "to_display_floor_id": target_room.get("display_floor_id"),
                    "relation_type": edge.get("relation_type"),
                    "transition_ids": list((edge.get("metadata") or {}).get("transition_ids", [])),
                }
                floor_switches.append(switch_payload)
                hop_summaries.append(
                    f"{edge['source_room_id']} -> {edge['target_room_id']} via {edge['relation_type']} "
                    f"(floor switch {source_floor_label} -> {target_floor_label}, conf={edge['confidence']}, cost={edge['edge_cost']})"
                )
            else:
                hop_summaries.append(
                    f"{edge['source_room_id']} -> {edge['target_room_id']} via {edge['relation_type']} "
                    f"({source_floor_label}, conf={edge['confidence']}, cost={edge['edge_cost']})"
                )
        floor_switch_summary = (
            " no floor switches."
            if not floor_switches
            else " floor switches=" + "; ".join(
                f"{item['source_room_id']}->{item['target_room_id']} ({display_floor_label(item.get('from_floor_id'), item.get('from_display_floor_id'))} -> {display_floor_label(item.get('to_floor_id'), item.get('to_display_floor_id'))} via {item['relation_type']})"
                for item in floor_switches
            )
        )
        return {
            "summary": (
                f"Route from {start_room_id} to {resolved_goal_room_id} found under the "
                f"{route_policy} policy."
            ),
            "target_summary": target_summary["summary"],
            "route_summary": (
                f"{route.get('hop_count', 0)} hop(s), relations={relation_summary}, "
                f"route_confidence={route.get('route_confidence')}, total_cost={route.get('total_cost')}."
                f"{floor_switch_summary}"
            ),
            "hop_summaries": hop_summaries,
            "floor_switches": floor_switches,
            "notes": list(target_summary.get("notes", [])),
        }

    def summarize_target_resolution(
        self,
        target_resolution: Optional[Dict[str, Any]] = None,
        query_result: Optional[Dict[str, Any]] = None,
    ) -> Dict[str, Any]:
        resolution = dict(target_resolution or (query_result or {}).get("target_resolution") or {})
        target_type = resolution.get("target_type", "unknown")
        input_payload = dict(resolution.get("input") or {})
        notes = list(resolution.get("notes", []))
        if not resolution:
            return {
                "summary": "No target resolution available.",
                "target_type": target_type,
                "resolved_room_id": None,
                "matched_entity_ids": [],
                "notes": notes,
            }

        if resolution.get("resolved"):
            resolved_room_id = resolution.get("resolved_room_id")
            if target_type == "room":
                summary = f"Room target {resolved_room_id} resolved directly."
            elif target_type == "anchor":
                summary = f"Anchor {input_payload.get('anchor_id')} resolved to {resolved_room_id}."
            elif input_payload.get("object_id") is not None:
                summary = f"Object {input_payload.get('object_id')} resolved to {resolved_room_id}."
            else:
                summary = (
                    f"Object label {input_payload.get('object_label')!r} resolved to "
                    f"{resolved_room_id} via {', '.join(resolution.get('matched_entity_ids', []))}."
                )
            floor_ref = display_floor_label(
                resolution.get("resolved_floor_id"),
                resolution.get("resolved_display_floor_id"),
            )
            if resolution.get("resolved_floor_id") is not None:
                summary = f"{summary.rstrip('.')} on {floor_ref}."
            return {
                "summary": summary,
                "target_type": target_type,
                "resolved_room_id": resolved_room_id,
                "resolved_floor_id": resolution.get("resolved_floor_id"),
                "resolved_display_floor_id": resolution.get("resolved_display_floor_id"),
                "matched_entity_ids": list(resolution.get("matched_entity_ids", [])),
                "notes": notes,
            }

        if resolution.get("failure_reason") == "object_label_ambiguous":
            candidate_room_ids = list((resolution.get("ambiguity") or {}).get("candidate_room_ids", []))
            summary = (
                f"Object label {input_payload.get('object_label')!r} is ambiguous across rooms: "
                f"{', '.join(candidate_room_ids)}."
            )
        elif target_type == "anchor":
            summary = f"Anchor lookup failed: {resolution.get('failure_reason')}."
        elif target_type == "room":
            summary = f"Room lookup failed: {resolution.get('failure_reason')}."
        else:
            summary = f"Object lookup failed: {resolution.get('failure_reason')}."
        return {
            "summary": summary,
            "target_type": target_type,
            "resolved_room_id": resolution.get("resolved_room_id"),
            "resolved_floor_id": resolution.get("resolved_floor_id"),
            "resolved_display_floor_id": resolution.get("resolved_display_floor_id"),
            "matched_entity_ids": list(resolution.get("matched_entity_ids", [])),
            "notes": notes,
        }

    def _resolve_room_target(self, goal_room_id: Any) -> Dict[str, Any]:
        canonical_room_id = _canonical_room_id(goal_room_id)
        resolution = self._make_resolution_result(
            target_type="room",
            input_payload={"goal_room_id": goal_room_id},
            matched_entity_ids=[],
            candidate_matches=[],
        )
        if canonical_room_id is None:
            return self._finalize_resolution(
                resolution,
                resolved=False,
                failure_reason="invalid_goal_room_id",
                notes=["Goal room id could not be canonicalized."],
            )
        resolution["matched_entity_ids"] = [canonical_room_id]
        if canonical_room_id not in self.topology.graph.nodes:
            return self._finalize_resolution(
                resolution,
                resolved=False,
                failure_reason="goal_room_not_in_topology",
                notes=[f"Room {canonical_room_id} is not present in the topology graph."],
            )
        room_record = self.topology.get_room(canonical_room_id) or {}
        return self._finalize_resolution(
            resolution,
            resolved=True,
            resolved_room_id=canonical_room_id,
            resolved_floor_id=room_record.get("floor_id"),
            resolved_display_floor_id=room_record.get("display_floor_id"),
            notes=[f"Goal room {canonical_room_id} can be used directly for routing."],
        )

    def _resolve_object_by_id(self, object_id: Any, object_label: Optional[str] = None) -> Dict[str, Any]:
        canonical_object_id = _canonical_object_id(object_id)
        resolution = self._make_resolution_result(
            target_type="object",
            input_payload={"object_id": object_id, "object_label": object_label},
            matched_entity_ids=[],
            candidate_matches=[],
        )
        if canonical_object_id is None:
            return self._finalize_resolution(
                resolution,
                resolved=False,
                failure_reason="invalid_object_id",
                notes=["Object id could not be canonicalized."],
            )

        object_record = self.topology.get_object(canonical_object_id)
        room_id = self.topology.get_room_of_object(canonical_object_id)
        candidate_matches = [self._serialize_object_candidate(object_record, rank=1)] if object_record else []
        resolution["candidate_matches"] = candidate_matches
        resolution["matched_entity_ids"] = [canonical_object_id] if object_record or room_id else []
        if object_record is None and room_id is None:
            return self._finalize_resolution(
                resolution,
                resolved=False,
                failure_reason="object_not_found",
                notes=[f"Object {canonical_object_id} is not present in the topology export."],
            )
        if room_id is None or room_id not in self.topology.graph.nodes:
            return self._finalize_resolution(
                resolution,
                resolved=False,
                failure_reason="object_has_no_resolved_room",
                notes=[f"Object {canonical_object_id} exists but does not map to a valid room node."],
            )
        room_record = self.topology.get_room(room_id) or {}
        return self._finalize_resolution(
            resolution,
            resolved=True,
            resolved_room_id=room_id,
            resolved_floor_id=room_record.get("floor_id"),
            resolved_display_floor_id=room_record.get("display_floor_id"),
            notes=[f"Object {canonical_object_id} resolves to {room_id}."],
        )

    def _resolve_object_by_label(self, object_label: str) -> Dict[str, Any]:
        resolution = self._make_resolution_result(
            target_type="object",
            input_payload={"object_id": None, "object_label": object_label},
            matched_entity_ids=[],
            candidate_matches=[],
        )
        query_token = _sanitize_label_token(object_label)
        if query_token is None:
            return self._finalize_resolution(
                resolution,
                resolved=False,
                failure_reason="invalid_object_label",
                notes=["Object label is empty after normalization."],
            )

        query_normalized = normalize_open_vocab_label(object_label)
        labeled_records = [
            record
            for record in self.topology.object_records.values()
            if record.get("label") or record.get("normalized_label")
        ]
        if not labeled_records:
            return self._finalize_resolution(
                resolution,
                resolved=False,
                failure_reason="object_label_lookup_unavailable",
                notes=["This topology export does not contain inspectable object labels for label-based lookup."],
            )

        candidates: List[Dict[str, Any]] = []
        for record in labeled_records:
            record_label = record.get("label") or record.get("normalized_label")
            raw_token = _sanitize_label_token(record_label)
            normalized_label = record.get("normalized_label") or normalize_open_vocab_label(record_label)
            if raw_token == query_token:
                match_rank = 0
                match_type = "exact_label"
            elif normalized_label == query_normalized:
                match_rank = 1
                match_type = "normalized_label"
            else:
                continue
            candidates.append(
                {
                    "record": dict(record),
                    "match_rank": match_rank,
                    "match_type": match_type,
                    "match_score": self._object_record_score(record),
                }
            )

        if not candidates:
            return self._finalize_resolution(
                resolution,
                resolved=False,
                failure_reason="object_not_found",
                notes=[f"No object label match found for {object_label!r}."],
            )

        candidates = sorted(
            candidates,
            key=lambda item: (
                int(item["match_rank"]),
                -float(item["match_score"]),
                str(item["record"].get("id")),
            ),
        )
        resolution["candidate_matches"] = [
            self._serialize_object_candidate(item["record"], rank=idx + 1, match_type=item["match_type"])
            for idx, item in enumerate(candidates)
        ]
        resolution["matched_entity_ids"] = [item["record"]["id"] for item in candidates]

        resolved_room_ids = sorted(
            {
                item["record"].get("room_id")
                for item in candidates
                if item["record"].get("room_id") in self.topology.graph.nodes
            }
        )
        if not resolved_room_ids:
            return self._finalize_resolution(
                resolution,
                resolved=False,
                failure_reason="object_has_no_resolved_room",
                notes=[f"Matched object label {object_label!r}, but none of the candidates has a valid room assignment."],
            )
        if len(resolved_room_ids) > 1:
            resolution["ambiguity"] = {
                "candidate_room_ids": resolved_room_ids,
                "candidate_count": len(candidates),
            }
            return self._finalize_resolution(
                resolution,
                resolved=False,
                failure_reason="object_label_ambiguous",
                notes=[f"Matched object label {object_label!r} across multiple rooms."],
            )

        notes: List[str] = []
        if len(candidates) > 1:
            notes.append(
                f"Matched {len(candidates)} objects for label {object_label!r}; all resolved to {resolved_room_ids[0]}."
            )
        else:
            notes.append(f"Object label {object_label!r} resolved via {candidates[0]['record']['id']}.")
        room_record = self.topology.get_room(resolved_room_ids[0]) or {}
        return self._finalize_resolution(
            resolution,
            resolved=True,
            resolved_room_id=resolved_room_ids[0],
            resolved_floor_id=room_record.get("floor_id"),
            resolved_display_floor_id=room_record.get("display_floor_id"),
            notes=notes,
        )

    def _execute_route_query(
        self,
        query_type: str,
        start_room_id: Any,
        target_resolution: Dict[str, Any],
        route_policy: str,
        query_input: Dict[str, Any],
    ) -> Dict[str, Any]:
        canonical_start_room_id = _canonical_room_id(start_room_id)
        policy_settings = self.route_policy_presets.get(str(route_policy))
        if policy_settings is None:
            route_result = self._make_route_result(
                attempted=False,
                found=False,
                route_policy=str(route_policy),
                policy_settings=None,
                failure_reason="invalid_route_policy",
            )
            result = {
                "success": False,
                "found": False,
                "query_type": query_type,
                "query": dict(query_input),
                "start_room_id": canonical_start_room_id,
                "resolved_goal_room_id": target_resolution.get("resolved_room_id"),
                "target_resolution": dict(target_resolution),
                "route": route_result,
                "route_policy": str(route_policy),
                "failure_reason": "invalid_route_policy",
                "metadata": {
                    "route_policy_preset": None,
                    "available_route_policies": sorted(self.route_policy_presets),
                },
            }
            result["explanation"] = self.explain_route(result)
            return result

        route_policy_payload = dict(policy_settings)
        if not target_resolution.get("resolved"):
            route_result = self._make_route_result(
                attempted=False,
                found=False,
                route_policy=str(route_policy),
                policy_settings=route_policy_payload,
                failure_reason=target_resolution.get("failure_reason"),
            )
            result = {
                "success": False,
                "found": False,
                "query_type": query_type,
                "query": dict(query_input),
                "start_room_id": canonical_start_room_id,
                "resolved_goal_room_id": target_resolution.get("resolved_room_id"),
                "target_resolution": dict(target_resolution),
                "route": route_result,
                "route_policy": str(route_policy),
                "failure_reason": target_resolution.get("failure_reason"),
                "metadata": {
                    "route_policy_preset": route_policy_payload,
                    "available_route_policies": sorted(self.route_policy_presets),
                },
            }
            result["explanation"] = self.explain_route(result)
            return result

        route = self.topology.find_room_path(
            canonical_start_room_id,
            target_resolution["resolved_room_id"],
            allowed_relations=route_policy_payload["allowed_relations"],
            min_conf=route_policy_payload["min_conf"],
            method="shortest",
        )
        route_result = self._make_route_result(
            attempted=True,
            found=bool(route.get("found")),
            route_policy=str(route_policy),
            policy_settings=route_policy_payload,
            failure_reason=route.get("failure_reason"),
            route_payload=route,
        )
        if int(route_policy_payload.get("max_candidate_paths", 1)) > 1:
            candidate_routes = self.topology.find_candidate_room_paths(
                canonical_start_room_id,
                target_resolution["resolved_room_id"],
                allowed_relations=route_policy_payload["allowed_relations"],
                min_conf=route_policy_payload["min_conf"],
                method="shortest",
                max_paths=int(route_policy_payload["max_candidate_paths"]),
            )
            route_result["candidate_paths"] = list(candidate_routes.get("paths", []))

        result = {
            "success": bool(route_result.get("found")),
            "found": bool(route_result.get("found")),
            "query_type": query_type,
            "query": dict(query_input),
            "start_room_id": canonical_start_room_id,
            "resolved_goal_room_id": target_resolution.get("resolved_room_id"),
            "target_resolution": dict(target_resolution),
            "route": route_result,
            "route_policy": str(route_policy),
            "failure_reason": None if route_result.get("found") else route_result.get("failure_reason"),
            "metadata": {
                "route_policy_preset": route_policy_payload,
                "available_route_policies": sorted(self.route_policy_presets),
            },
        }
        result["explanation"] = self.explain_route(result)
        return result

    def _make_resolution_result(
        self,
        target_type: str,
        input_payload: Dict[str, Any],
        matched_entity_ids: List[str],
        candidate_matches: List[Dict[str, Any]],
    ) -> Dict[str, Any]:
        return {
            "success": False,
            "resolved": False,
            "target_type": str(target_type),
            "input": dict(input_payload),
            "resolved_room_id": None,
            "resolved_floor_id": None,
            "resolved_display_floor_id": None,
            "matched_entity_ids": list(matched_entity_ids),
            "candidate_matches": list(candidate_matches),
            "ambiguity": None,
            "failure_reason": None,
            "notes": [],
        }

    def _finalize_resolution(
        self,
        resolution: Dict[str, Any],
        resolved: bool,
        resolved_room_id: Optional[str] = None,
        resolved_floor_id: Optional[str] = None,
        resolved_display_floor_id: Optional[str] = None,
        failure_reason: Optional[str] = None,
        notes: Optional[List[str]] = None,
    ) -> Dict[str, Any]:
        payload = dict(resolution)
        payload["resolved"] = bool(resolved)
        payload["success"] = bool(resolved)
        payload["resolved_room_id"] = resolved_room_id
        payload["resolved_floor_id"] = resolved_floor_id
        payload["resolved_display_floor_id"] = resolved_display_floor_id
        payload["failure_reason"] = None if resolved else failure_reason
        payload["notes"] = list(notes or [])
        return payload

    def _make_route_result(
        self,
        attempted: bool,
        found: bool,
        route_policy: str,
        policy_settings: Optional[Dict[str, Any]],
        failure_reason: Optional[str],
        route_payload: Optional[Dict[str, Any]] = None,
    ) -> Dict[str, Any]:
        payload = {
            "attempted": bool(attempted),
            "found": bool(found),
            "route_policy": str(route_policy),
            "policy_settings": None if policy_settings is None else dict(policy_settings),
            "failure_reason": failure_reason,
            "room_sequence": [],
            "edges": [],
            "used_relation_types": [],
            "edge_confidences": [],
            "hop_count": 0,
            "total_cost": None,
            "route_confidence": None,
            "score_summary": {
                "hop_count": 0,
                "total_cost": None,
                "route_confidence": None,
            },
            "search_space": {},
        }
        if route_payload:
            payload.update(dict(route_payload))
            payload["attempted"] = bool(attempted)
            payload["found"] = bool(found)
            payload["route_policy"] = str(route_policy)
            payload["policy_settings"] = None if policy_settings is None else dict(policy_settings)
        return payload

    def _serialize_object_candidate(
        self,
        record: Optional[Dict[str, Any]],
        rank: int,
        match_type: Optional[str] = None,
    ) -> Dict[str, Any]:
        if record is None:
            return {}
        return {
            "rank": int(rank),
            "entity_id": record.get("id"),
            "label": record.get("label"),
            "normalized_label": record.get("normalized_label"),
            "category": record.get("category"),
            "room_id": record.get("room_id"),
            "floor_id": record.get("floor_id"),
            "display_floor_id": record.get("display_floor_id"),
            "display_order": record.get("display_order"),
            "score": record.get("score"),
            "detection_confidence": record.get("detection_confidence"),
            "semantic_confidence": record.get("semantic_confidence"),
            "match_type": match_type,
        }

    def _serialize_anchor_candidate(self, record: Optional[Dict[str, Any]]) -> Dict[str, Any]:
        if record is None:
            return {}
        return {
            "entity_id": record.get("id"),
            "anchor_type": record.get("anchor_type"),
            "room_id": record.get("room_id"),
            "floor_id": record.get("floor_id"),
            "display_floor_id": record.get("display_floor_id"),
            "display_order": record.get("display_order"),
            "target_id": record.get("target_id"),
            "valid": record.get("valid"),
            "score": record.get("score"),
        }

    def _object_record_score(self, record: Dict[str, Any]) -> float:
        return float(
            record.get("semantic_confidence")
            or record.get("detection_confidence")
            or record.get("score")
            or 0.0
        )
