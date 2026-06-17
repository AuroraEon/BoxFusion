# Stable Map / Object Coordinate Alignment Audit

## Why task36b was needed

Task36 produced the canonical Layer 2: Formal Artifact Layer package and found
that `generated_ring_037` was occupied, had zero clearance, and had a blocked
ray to `obj_175`. The stable-map preview appeared vertically flipped to the
user, so task36b audited whether that blocker was real or caused by a raster
orientation or world-to-grid mismatch.

This audit did not rerun Layer 1: World Model Layer, regenerate Layer 2
artifacts, change the approach candidate, or generate Layer 3 routes.

## Raster orientation

The floor-2 NPZ planning grid and preview use row 0 for low world y. Raster
viewers draw row 0 at the top, so that preview can look vertically flipped
relative to a conventional Cartesian display.

The PGM is the vertical flip of the NPZ planning grid. Its row 0 therefore
represents high world y, which is the expected top-origin image ordering for
the map-server raster. A visually flipped preview does not automatically mean
that world coordinates are mapped to the wrong cells.

The canonical metadata provides width, height, resolution, origin, and cell
semantics, but it does not explicitly declare a frame id or the preview/NPZ
raster orientation. Task36b inferred orientation by comparing the NPZ,
preview, PGM, canonical object artifact, and canonical builder convention.

## Transforms tested

Task36b tested:

- Project transform: round-based, no y flip, matching task36's
  `grid_rc` calculation.
- Required no-y-flip transform: floor-based row and column.
- Required y-flip transform: floor-based column with the raster row inverted.

The project and no-y-flip transforms both place the room 7, room 13, room 14,
and floor-2 connector references on free cells. The y-flip transform places
all of those references in unknown space.

## Conclusion

The alignment classification is
`alignment_passed_object_approach_blocker_is_real`.

For `generated_ring_037`:

- Project transform: `[1041, 851]`, occupied, `0.0 m` clearance, ray blocked.
- No-y-flip transform: `[1041, 851]`, occupied, `0.0 m` clearance, ray blocked.
- Y-flip transform: `[113, 851]`, unknown, `0.0 m` traversable clearance, ray
  not clear because all sampled cells are unknown.

The y-flip alternative does not provide a more consistent explanation. It
moves the room and connector references away from mapped free space. The
task36 object-approach blocker is therefore likely real, not caused by a simple
y-axis flip.

The next recommended task is
`task37_layer3_navigation_interface_canonicalization_and_route_generation_with_object_approach_blocker`.
Layer 3 may preserve the object-approach blocker, but must not emit an
executable object-approach route for `generated_ring_037`.
