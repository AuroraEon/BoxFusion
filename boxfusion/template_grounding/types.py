from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Pattern, Tuple


TEMPLATE_GROUNDING_VERSION = "0.1"


@dataclass(frozen=True)
class TemplateSpec:
    template_id: str
    intent: str
    pattern: Pattern[str]
    slot_names: Tuple[str, ...]
    description: str


@dataclass
class ParsedTemplate:
    instruction: str
    normalized_instruction: str
    ok: bool
    template_id: Optional[str] = None
    intent: Optional[str] = None
    raw_slots: Dict[str, str] = field(default_factory=dict)
    normalized_slots: Dict[str, str] = field(default_factory=dict)
    failure_reason: Optional[str] = None
    notes: List[str] = field(default_factory=list)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "template_id": self.template_id,
            "intent": self.intent,
            "raw_slots": dict(self.raw_slots),
            "normalized_slots": dict(self.normalized_slots),
            "failure_reason": self.failure_reason,
            "notes": list(self.notes),
        }
