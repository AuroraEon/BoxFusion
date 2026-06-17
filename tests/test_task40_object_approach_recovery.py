from __future__ import annotations

import unittest

from tools.rslg_pipeline.recover_object_approach_candidates import (
    generated_ring_candidates,
    point_in_polygon,
)


class Task40ObjectApproachRecoveryTest(unittest.TestCase):
    def setUp(self) -> None:
        self.object_record = {
            "pose": [-8.149, 0.469],
            "footprint_2d": [
                [-7.258999824523926, 1.1419999599456787],
                [-9.038999557495117, 1.1419999599456787],
                [-9.038999557495117, -0.20399999618530273],
                [-7.258999824523926, -0.20399999618530273],
            ],
        }

    def test_existing_policy_reproduces_known_candidates(self) -> None:
        candidates = generated_ring_candidates(self.object_record)
        self.assertEqual(51, len(candidates))
        by_id = {candidate["candidate_id"]: candidate for candidate in candidates}
        self.assertEqual([-7.020484, 1.558795], by_id["generated_ring_002"]["world_xy"])
        self.assertEqual([-7.442624, 2.055545], by_id["generated_ring_037"]["world_xy"])
        self.assertFalse(by_id["generated_ring_002"]["object_centroid_navigation_used"])

    def test_polygon_boundary_can_be_used_as_object_endpoint(self) -> None:
        footprint = self.object_record["footprint_2d"]
        self.assertTrue(
            point_in_polygon(
                -7.258999824523926,
                1.1419999599456787,
                footprint,
            )
        )
        self.assertFalse(point_in_polygon(-7.020484, 1.558795, footprint))


if __name__ == "__main__":
    unittest.main()
