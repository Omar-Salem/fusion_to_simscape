"""Motion-axis conversion checks independent of Fusion's runtime."""
import json
import math
from pathlib import Path
import unittest

from FusionToSimscape.joint_frames import motion_frame, rotate, unit
from FusionToSimscape.transforms import identity, inverse, metres, multiply, point


class JointFrameTests(unittest.TestCase):
    def assert_rotation(self, frame):
        columns = [[frame[4*r+c] for r in range(3)] for c in range(3)]
        for a in range(3):
            for b in range(3):
                self.assertAlmostEqual(sum(x*y for x, y in zip(columns[a], columns[b])),
                                       float(a == b), places=12)
        x, y, z = columns
        cross = [x[1]*y[2]-x[2]*y[1], x[2]*y[0]-x[0]*y[2], x[0]*y[1]-x[1]*y[0]]
        for actual, expected in zip(cross, z):
            self.assertAlmostEqual(actual, expected, places=12)

    def test_standard_axes_preserve_origin_handedness_and_positive_direction(self):
        frame = [0., -1., 0., .3, 1., 0., 0., -.2, 0., 0., 1., .7, 0., 0., 0., 1.]
        for axis in range(3):
            with self.subTest(axis=axis):
                result = motion_frame(frame, axis)
                self.assertEqual(point(result, [0., 0., 0.]), point(frame, [0., 0., 0.]))
                self.assertEqual(result[2:12:4], frame[axis:12:4])
                self.assert_rotation(result)
        self.assertEqual(motion_frame(frame, 2), frame)

    def test_custom_axis_transports_solved_placement_instead_of_nominal_parent(self):
        native = [0., 0., 1., .2, 0., 1., 0., .3, -1., 0., 0., .4, 0., 0., 0., 1.]
        solved = [0., -1., 0., 1., 1., 0., 0., 2., 0., 0., 1., 3., 0., 0., 0., 1.]
        proxy = multiply(solved, native)
        axis = [1., 2., -3.]
        result = motion_frame(proxy, 3, axis, native)
        for actual, expected in zip(result[2:12:4], unit(rotate(solved, axis))):
            self.assertAlmostEqual(actual, expected, places=12)
        self.assertEqual(result[3:12:4], proxy[3:12:4])
        self.assert_rotation(result)

    def test_negative_and_parallel_custom_axes(self):
        for axis in ([1., 0., 0.], [-1., 0., 0.], [0., 0., -1.]):
            result = motion_frame(identity(), 3, axis, identity())
            self.assertEqual(result[2:12:4], axis)
            self.assert_rotation(result)

    def test_captured_native_motion_vectors_agree_with_solved_proxy_axes(self):
        folder = Path(__file__).parent / 'fixtures' / 'assembled'
        audits = json.loads((folder / 'joint_frame_audit.json').read_text())
        measurements = json.loads((folder / 'source_audit.json').read_text())['axes']
        for measurement in measurements:
            audit = next(record for record in audits if record['owner'] == measurement['path'])
            native = metres(audit['native']['geometry_two'])
            proxy = metres(audit['proxy']['geometry_two'])
            frame = motion_frame(proxy, 3, measurement['axis'], native)
            for actual, expected in zip(frame[2:12:4], proxy[2:12:4]):
                self.assertAlmostEqual(actual, expected, places=12)

    def test_rejects_undefined_motion_axes(self):
        for axis in ([0., 0., 0.], [math.nan, 0., 1.], [math.inf, 1., 0.]):
            with self.assertRaises(ValueError):
                motion_frame(identity(), 3, axis, identity())
        with self.assertRaises(ValueError):
            motion_frame(identity(), 3)
        with self.assertRaises(ValueError):
            motion_frame(identity(), 9)

    def test_remapped_frames_reconstruct_the_same_assembly_pose(self):
        base = identity(); base[3:12:4] = [.1, -.3, .2]
        follower = identity(); follower[3:12:4] = [-.4, .6, .8]
        frame = motion_frame(base, 0)
        base_local = multiply(inverse(base), frame)
        follower_local = multiply(inverse(follower), frame)
        recovered = multiply(multiply(base, base_local), inverse(follower_local))
        for actual, expected in zip(recovered, follower):
            self.assertAlmostEqual(actual, expected, places=12)
