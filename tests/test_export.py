"""Checks for coordinate conventions and captured assembly integrity."""
import json
import math
from pathlib import Path
import struct
import unittest

from FusionToSimscape.transforms import identity, inverse, metres, multiply, point, principal_inertia


class TransformTests(unittest.TestCase):
    def test_inverse_rotated_translated_frame(self):
        matrix = [0., -1., 0., 2., 1., 0., 0., 3., 0., 0., 1., 4., 0., 0., 0., 1.]
        for actual, expected in zip(multiply(inverse(matrix), matrix), identity()):
            self.assertAlmostEqual(actual, expected)
        self.assertEqual(point(matrix, [1., 0., 0.]), [2., 4., 4.])

    def test_units_and_principal_inertia_rotation(self):
        matrix = identity()
        matrix[3] = 100.
        self.assertEqual(metres(matrix)[3], 1.)
        a = math.sqrt(0.5)
        inertia = principal_inertia([1., 3., 4.], [[a, a, 0.], [-a, a, 0.], [0., 0., 1.]])
        for actual, expected in zip(inertia, [2e-4, 2e-4, 4e-4, 0., 0., -1e-4]):
            self.assertAlmostEqual(actual, expected)


class CapturedAssemblyTests(unittest.TestCase):
    def test_wheel_frames_match_solved_fusion_joint_proxies(self):
        folder = Path(__file__).resolve().parent / 'fixtures' / 'assembled'
        snapshot = json.loads((folder / 'assembly.json').read_text())
        source = json.loads((folder / 'joint_frame_audit.json').read_text())
        for measurement in source:
            joint = next(joint for joint in snapshot['joints']
                         if joint['name'] == measurement['owner'] + '/Revolute 8')
            for actual, expected in zip(joint['frame'], metres(measurement['proxy']['geometry_two'])):
                self.assertAlmostEqual(actual, expected, places=12)
            body = next(body for body in snapshot['bodies'] if body['uid'] == joint['follower'])
            centre = point(body['transform'], body['center_of_mass'])
            offset = [centre[k] - joint['frame'][4*k+3] for k in range(3)]
            axis = [joint['frame'][4*k+2] for k in range(3)]
            along = sum(a*b for a, b in zip(axis, offset))
            radial = math.sqrt(sum((offset[k]-along*axis[k])**2 for k in range(3)))
            self.assertLess(radial, 1e-6, 'Wheel rotation axis must pass through its centre.')

    def test_centres_match_independent_fusion_measurements(self):
        folder = Path(__file__).resolve().parent / 'fixtures' / 'assembled'
        snapshot = json.loads((folder / 'assembly.json').read_text())
        source = json.loads((folder / 'source_audit.json').read_text())
        bodies = {body['occurrence']: body for body in snapshot['bodies']}
        for measurement in source['bodies']:
            body = bodies[measurement['path']]
            centre = point(body['transform'], body['center_of_mass'])
            for actual, expected in zip(centre, measurement['center']):
                self.assertAlmostEqual(actual, expected, places=12)

    def test_exported_assembly_is_connected_and_has_two_wheel_joints(self):
        folder = Path(__file__).resolve().parent / 'fixtures' / 'assembled'
        snapshot = json.loads((folder / 'assembly.json').read_text())
        bodies = {body['uid']: body for body in snapshot['bodies']}
        self.assertEqual(len(bodies), 13)
        self.assertEqual(sum(j['type'] == 'Revolute' for j in snapshot['joints']), 2)
        graph = {key: set() for key in bodies}
        for joint in snapshot['joints']:
            graph[joint['base']].add(joint['follower'])
            graph[joint['follower']].add(joint['base'])
            for key in ('base', 'follower'):
                local = multiply(inverse(bodies[joint[key]]['transform']), joint['frame'])
                recovered = multiply(bodies[joint[key]]['transform'], local)
                for actual, expected in zip(recovered, joint['frame']):
                    self.assertAlmostEqual(actual, expected, places=10)
        visited = set()
        pending = [key for key, body in bodies.items() if body['grounded']]
        self.assertEqual(len(pending), 1)
        while pending:
            key = pending.pop()
            if key not in visited:
                visited.add(key)
                pending.extend(graph[key] - visited)
        self.assertEqual(visited, set(bodies))
        for body in bodies.values():
            self.assertGreater(body['mass'], 0.)
            xx, yy, zz, yz, xz, xy = body['inertia']
            self.assertGreater(xx, 0.)
            self.assertGreater(xx*yy-xy*xy, 0.)
            self.assertGreater(xx*yy*zz+2*xy*xz*yz-xx*yz*yz-yy*xz*xz-zz*xy*xy, 0.)


class LocalExportTests(unittest.TestCase):
    @unittest.skipUnless(
        (Path(__file__).resolve().parents[1] / 'exports/assembled/assembly.json').exists(),
        'Requires a local Fusion export in exports/assembled.')
    def test_binary_stl_files_match_triangle_counts(self):
        folder = Path(__file__).resolve().parents[1] / 'exports/assembled'
        snapshot = json.loads((folder / 'assembly.json').read_text())
        for body in snapshot['bodies']:
            data = (folder / body['geometry']).read_bytes()
            count, = struct.unpack_from('<I', data, 80)
            self.assertEqual(len(data), 84 + 50*count)


if __name__ == '__main__':
    unittest.main()
