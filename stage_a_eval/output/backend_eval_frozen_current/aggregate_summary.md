# Backend Evaluation Summary

## Scope

- Evaluates Tier 1 core backend artifacts only: manifest, summary, topology, topology query report, vertical-transition evidence, and floor diagnostics.
- Query tasks reuse the existing room-centric backend layers against seeded truth-layer overlap probes.
- Query tasks reuse the current Query API; execute tasks reuse symbolic plan generation from the existing closed-loop executor stack.
- Tier 2 demo outputs such as PNG, MP4, markdown, and appendix-style diagnostics are optional and excluded from backend compactness totals.
- Limitation: the seeded tasks are backend-overlap probes generated from current scene exports, not external human-annotated navigation benchmarks.

## Aggregate

- Task success rate: 97.7% (43/44)
- Resolve success rate: 100.0% (8/8)
- Exact room hit rate: 97.7% (43/44)
- Exact floor hit rate: 97.7% (43/44)
- Route-found rate: 97.2% (35/36)
- Query API latency mean/p50/p90 (ms): 1.304 / 0.702 / 2.694
- Symbolic plan latency mean/p50/p90 (ms): 1.248 / 0.895 / 2.349
- Aggregate Tier 1 backend artifact bytes: 18766991
- Aggregate Tier 2 optional demo artifact bytes: 0

## Probe Slice Breakdown

| Slice | Tasks | Expected-Failure Tasks | Success Rate | Room Hit | Floor Hit | Route Found | Mean Latency (ms) | Status Histogram |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| positive_seeded | 44 | 0 | 97.7% | 97.7% | 97.7% | 97.2% | 1.297 | {"not_found": 1, "success": 43} |

## Policy Breakdown

| Policy | Tasks | Success Rate | Route-Found Rate | Mean Latency (ms) |
| --- | ---: | ---: | ---: | ---: |
| balanced | 32 | 96.9% | 95.8% | 0.899 |
| exploratory | 6 | 100.0% | 100.0% | 3.752 |
| strict | 6 | 100.0% | 100.0% | 0.962 |

## Task-Type Breakdown

| Task Type | Tasks | Success Rate | Route-Found Rate | Mean Latency (ms) |
| --- | ---: | ---: | ---: | ---: |
| execute_room_route | 6 | 100.0% | 100.0% | 1.248 |
| query_anchor_route | 6 | 83.3% | 83.3% | 1.15 |
| query_object_route | 6 | 100.0% | 100.0% | 1.237 |
| query_room_route | 18 | 100.0% | 100.0% | 1.946 |
| resolve_anchor | 4 | 100.0% | n/a | 0.024 |
| resolve_object | 4 | 100.0% | n/a | 0.027 |

## Scene Runtime Summary

| Scene | Floors | Public Rooms | Objects | Nodes | Public Edges | VT | Tier 1 Bytes | Tier 2 Bytes | Tier 1 Bytes/Room |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 00824-Dd4bFSTQ8gi | 1 | 3 | 142 | 177 | 2 | 0 | 2308011 | 0 | 288501.375 |
| 00829-QaLdnwvtxbs | 1 | 0 | 145 | 161 | 0 | 0 | 1342526 | 0 | 671263.0 |
| 00843-DYehNKdT76V | 2 | 10 | 86 | 120 | 15 | 1 | 2271452 | 0 | 206495.636 |
| 00862-LT9Jq6dN3Ea | 3 | 23 | 469 | 566 | 73 | 2 | 12845002 | 0 | 428166.733 |

## Scene Task Breakdown

| Scene | Tasks | Success Rate | Room Hit | Floor Hit | Route Found | Mean Latency (ms) |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 00824-Dd4bFSTQ8gi | 8 | 100.0% | 100.0% | 100.0% | 100.0% | 0.806 |
| 00829-QaLdnwvtxbs | 8 | 87.5% | 87.5% | 87.5% | 83.3% | 0.187 |
| 00843-DYehNKdT76V | 14 | 100.0% | 100.0% | 100.0% | 100.0% | 0.908 |
| 00862-LT9Jq6dN3Ea | 14 | 100.0% | 100.0% | 100.0% | 100.0% | 2.6 |
