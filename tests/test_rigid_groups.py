"""Rigid-group topology and captured-pose regression checks."""
import json
from pathlib import Path
import unittest

from FusionToSimscape.rigid_groups import normalize_rigid_connections
from FusionToSimscape.transforms import identity, inverse, multiply


def bodies(*names):
    result = []
    for index, name in enumerate(names):
        transform = identity()
        transform[3:12:4] = [.1*index, -.03*index, .05]
        result.append(dict(uid=name, transform=transform))
    return result


def joint(name, base, follower, kind='Weld'):
    return dict(name=name, type=kind, base=base, follower=follower, frame=identity())


class RigidGroupTests(unittest.TestCase):
    def test_overlapping_groups_and_explicit_welds_form_a_minimal_forest(self):
        parts = bodies('a', 'b', 'c', 'd')
        existing = [joint('existing', 'a', 'b'), joint('duplicate', 'b', 'a')]
        groups = [dict(name='first', members=['a', 'b', 'c']),
                  dict(name='second', members=['b', 'c', 'd']),
                  dict(name='duplicate group', members=['a', 'd'])]
        result, diagnostics = normalize_rigid_connections(parts, existing, groups)
        self.assertEqual(len(result), 3)
        self.assertTrue(all(connection['type'] == 'Weld' for connection in result))
        self.assertEqual(diagnostics[0]['name'], 'duplicate')
        self.assertEqual(diagnostics[0]['status'], 'redundant')
        connected = {'a'}
        for _ in parts:
            for connection in result:
                endpoints = {connection['base'], connection['follower']}
                if endpoints & connected:
                    connected.update(endpoints)
        self.assertEqual(connected, {'a', 'b', 'c', 'd'})
        # Every exported weld must retain the original relative CAD pose.
        poses = {part['uid']: part['transform'] for part in parts}
        for connection in result:
            base = poses[connection['base']]
            follower = poses[connection['follower']]
            base_frame = multiply(inverse(base), connection['frame'])
            follower_frame = multiply(inverse(follower), connection['frame'])
            recovered = multiply(multiply(base, base_frame), inverse(follower_frame))
            for actual, expected in zip(recovered, follower):
                self.assertAlmostEqual(actual, expected, places=12)

    def test_external_hinge_and_slider_keep_their_freedoms(self):
        existing = [joint('pivot', 'b', 'hinge', 'Revolute'),
                    joint('slide', 'c', 'slider', 'Prismatic')]
        result, diagnostics = normalize_rigid_connections(
            bodies('a', 'b', 'c', 'hinge', 'slider'), existing,
            [dict(name='frame', members=['a', 'b', 'c'])])
        self.assertEqual(result[:2], existing)
        self.assertEqual(sum(connection['type'] == 'Weld' for connection in result), 2)
        self.assertEqual(diagnostics, [])

    def test_motion_within_a_rigid_group_is_reported_as_locked(self):
        result, diagnostics = normalize_rigid_connections(
            bodies('a', 'b', 'c'),
            [joint('pivot', 'a', 'b', 'Revolute'), joint('slide', 'b', 'c', 'Prismatic')],
            [dict(name='locked assembly', members=['a', 'b', 'c'])])
        self.assertEqual(len(result), 2)
        self.assertTrue(all(connection['type'] == 'Weld' for connection in result))
        self.assertEqual({record['name'] for record in diagnostics}, {'pivot', 'slide'})
        self.assertTrue(all(record['status'] == 'locked' for record in diagnostics))

    def test_small_and_duplicate_member_groups_do_not_invent_constraints(self):
        result, diagnostics = normalize_rigid_connections(
            bodies('a'), [], [dict(name='empty', members=[]), dict(name='single', members=['a', 'a'])])
        self.assertEqual(result, [])
        self.assertEqual(len(diagnostics), 2)
        with self.assertRaisesRegex(ValueError, 'missing bodies'):
            normalize_rigid_connections(bodies('a'), [], [dict(name='broken', members=['a', 'missing'])])

    def test_existing_robot_joint_graph_is_unchanged(self):
        path = Path(__file__).parent / 'fixtures' / 'assembled' / 'assembly.json'
        snapshot = json.loads(path.read_text())
        result, diagnostics = normalize_rigid_connections(snapshot['bodies'], snapshot['joints'], [])
        self.assertEqual(result, snapshot['joints'])
        self.assertEqual(diagnostics, [])
