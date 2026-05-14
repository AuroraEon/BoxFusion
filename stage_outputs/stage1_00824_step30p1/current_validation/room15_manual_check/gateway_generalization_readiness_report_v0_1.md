# Step30S7 Gateway Generalization Readiness

Selected gateway ids, forbidden pairs, and projection input paths are externalized in `config/step30s7_gateway_projection_config_v0_1.json`.

Active route/projection code hard-codes room15: `False`
Active route/projection code hard-codes route_room_ids: `False`
Full multi-scene gateway generalization claimed: `False`

Remaining scene-specific work: build and validate a reusable multi-scene gateway extractor, replace scene-local config bootstrap, and broaden cross-scene regression coverage.
