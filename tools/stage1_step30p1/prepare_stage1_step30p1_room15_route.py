#!/usr/bin/env python3
"""Backward-compatible entrypoint for Step30S7 semantic route preparation."""

from __future__ import annotations

from prepare_stage1_step30p1_semantic_route import main


if __name__ == "__main__":
    raise SystemExit(main())

