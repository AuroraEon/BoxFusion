from __future__ import annotations

import re
from typing import Any


_TRAILING_PUNCT_RE = re.compile(r"[.!?,;:]+$")
_INSTRUCTION_SPACE_RE = re.compile(r"\s+")
_REFERENCE_TOKEN_RE = re.compile(r"[^a-z0-9_/\- ]+")
_ANCHOR_SPACE_RE = re.compile(r"\s+")


def preprocess_instruction(instruction: Any) -> str:
    text = "" if instruction is None else str(instruction).strip().lower()
    text = _TRAILING_PUNCT_RE.sub("", text)
    return _INSTRUCTION_SPACE_RE.sub(" ", text).strip()


def normalize_reference_slot(value: Any) -> str:
    text = preprocess_instruction(value)
    text = _REFERENCE_TOKEN_RE.sub(" ", text)
    text = text.replace("-", " ")
    return "_".join(text.split())


def normalize_anchor_slot(value: Any) -> str:
    text = preprocess_instruction(value)
    return _ANCHOR_SPACE_RE.sub("", text)


def normalize_slot(slot_name: str, value: Any) -> str:
    if str(slot_name) == "anchor_id":
        return normalize_anchor_slot(value)
    return normalize_reference_slot(value)
