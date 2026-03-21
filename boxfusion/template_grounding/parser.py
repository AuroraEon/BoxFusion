from __future__ import annotations

from typing import Sequence

from boxfusion.template_grounding.catalog import TEMPLATE_CATALOG
from boxfusion.template_grounding.normalizer import normalize_slot, preprocess_instruction
from boxfusion.template_grounding.types import ParsedTemplate, TemplateSpec


_UNSUPPORTED_ROOM_SLOT_PREFIXES = (
    "anchor ",
    "object ",
    "room with ",
)


class TemplateParser:
    def __init__(self, catalog: Sequence[TemplateSpec] = TEMPLATE_CATALOG) -> None:
        self.catalog = tuple(catalog)

    def parse(self, instruction: str) -> ParsedTemplate:
        normalized_instruction = preprocess_instruction(instruction)
        if not normalized_instruction:
            return ParsedTemplate(
                instruction=instruction,
                normalized_instruction=normalized_instruction,
                ok=False,
                failure_reason="UNSUPPORTED_TEMPLATE",
                notes=["Instruction is empty after minimal preprocessing."],
            )

        for spec in self.catalog:
            match = spec.pattern.fullmatch(normalized_instruction)
            if match is None:
                continue

            raw_slots = {}
            normalized_slots = {}
            for slot_name in spec.slot_names:
                raw_value = (match.group(slot_name) or "").strip()
                if not raw_value:
                    return ParsedTemplate(
                        instruction=instruction,
                        normalized_instruction=normalized_instruction,
                        ok=False,
                        template_id=spec.template_id,
                        intent=spec.intent,
                        failure_reason="MISSING_REQUIRED_SLOT",
                        notes=[f"Required slot {slot_name!r} is empty."],
                    )
                normalized_value = normalize_slot(slot_name, raw_value)
                if not normalized_value:
                    return ParsedTemplate(
                        instruction=instruction,
                        normalized_instruction=normalized_instruction,
                        ok=False,
                        template_id=spec.template_id,
                        intent=spec.intent,
                        raw_slots={slot_name: raw_value},
                        failure_reason="SLOT_EXTRACTION_FAILED",
                        notes=[f"Slot {slot_name!r} became empty after normalization."],
                    )
                raw_slots[slot_name] = raw_value
                normalized_slots[slot_name] = normalized_value

            if spec.intent == "go_to_room":
                room_slot = raw_slots.get("room", "")
                if any(room_slot.startswith(prefix) for prefix in _UNSUPPORTED_ROOM_SLOT_PREFIXES):
                    return ParsedTemplate(
                        instruction=instruction,
                        normalized_instruction=normalized_instruction,
                        ok=False,
                        failure_reason="UNSUPPORTED_TEMPLATE",
                        notes=[
                            "Instruction looks like a non-room target request and is outside Template Grounding v0.1.",
                        ],
                    )

            return ParsedTemplate(
                instruction=instruction,
                normalized_instruction=normalized_instruction,
                ok=True,
                template_id=spec.template_id,
                intent=spec.intent,
                raw_slots=raw_slots,
                normalized_slots=normalized_slots,
            )

        return ParsedTemplate(
            instruction=instruction,
            normalized_instruction=normalized_instruction,
            ok=False,
            failure_reason="UNSUPPORTED_TEMPLATE",
            notes=["Instruction does not match any v0.1 template."],
        )
