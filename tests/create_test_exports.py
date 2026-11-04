"""Create small, model-independent CAD interchange cases for MATLAB checks."""
from pathlib import Path
import json
import math
import struct
import sys

from FusionToSimscape.joint_frames import motion_frame, rotate, unit
from FusionToSimscape.rigid_groups import normalize_rigid_connections
from FusionToSimscape.transforms import identity, multiply, triangle_normal
from FusionToSimscape.xml_writer import write


def cube_mesh(path):
    vertices = [(-.025, -.025, -.025), (.025, -.025, -.025),
                (.025, .025, -.025), (-.025, .025, -.025),
                (-.025, -.025, .025), (.025, -.025, .025),
                (.025, .025, .025), (-.025, .025, .025)]
    faces = [(0, 2, 1), (0, 3, 2), (4, 5, 6), (4, 6, 7),
             (0, 1, 5), (0, 5, 4), (1, 2, 6), (1, 6, 5),
             (2, 3, 7), (2, 7, 6), (3, 0, 4), (3, 4, 7)]
    with path.open('wb') as stream:
        stream.write(b'Synthetic regression cube'.ljust(80, b' '))
        stream.write(struct.pack('<I', len(faces)))
        for face in faces:
            points = [vertices[index] for index in face]
            stream.write(struct.pack('<12fH', *triangle_normal(points),
                                     *(v for point in points for v in point), 0))


def body(uid, name, x, grounded):
    transform = identity()
    transform[3], transform[11] = x, .1
    return dict(uid=uid, name=name, occurrence=name, transform=transform,
                grounded=grounded, mass=.2, center_of_mass=[0., 0., 0.],
                inertia=[.2*.05**2/6]*3 + [0., 0., 0.],
                geometry=f'geometry/{uid}.stl', triangles=12, color=[.4, .6, .8])



def rotation_z(angle):
    c, s = math.cos(angle), math.sin(angle)
    return [c, -s, 0., 0., s, c, 0., 0., 0., 0., 1., 0., 0., 0., 0., 1.]


def joint_motion_cases():
    owner = rotation_z(.61)
    owner[3:12:4] = [.14, -.22, .31]
    native = [1., 0., 0., .04, 0., 0., -1., .03, 0., 1., 0., -.02, 0., 0., 0., 1.]
    proxy = multiply(owner, native)
    base = body('fixed_mount', 'Rotated mount', 0., True)
    base['transform'] = owner
    bodies, joints = [base], []
    for kind in ('Revolute', 'Prismatic'):
        for direction in range(4):
            uid = f'{kind.lower()}_{direction}'
            bodies.append(body(uid, f'Moving {kind} {direction}', .1*(len(bodies)+1), False))
            if direction < 3:
                expected = [proxy[4*r+direction] for r in range(3)]
                frame = motion_frame(proxy, direction)
            else:
                axis = [1., 2., -3.]
                expected = unit(rotate(owner, axis))
                frame = motion_frame(proxy, direction, axis, native)
            joints.append(dict(name=f'{kind} axis {direction}', type=kind, base=base['uid'],
                               follower=uid, frame=frame, expected_motion_axis=expected))
    return dict(schemaVersion=1, document='Joint motion checks', bodies=bodies,
                joints=joints, diagnostics=[])


def rigid_group_case():
    bodies = [body(name, name, .1*index, index == 0)
              for index, name in enumerate(('a', 'b', 'c', 'd', 'hinge', 'slider'))]
    for index, part in enumerate(bodies):
        orientation = rotation_z(.17*index)
        orientation[3:12:4] = part['transform'][3:12:4]
        part['transform'] = orientation
    joints = [dict(name='Existing weld', type='Weld', base='a', follower='b', frame=identity()),
              dict(name='Duplicate weld', type='Weld', base='b', follower='a', frame=identity()),
              dict(name='Free hinge', type='Revolute', base='b', follower='hinge', frame=identity()),
              dict(name='Free slider', type='Prismatic', base='c', follower='slider',
                   frame=motion_frame(identity(), 0))]
    groups = [dict(name='First group', members=['a', 'b', 'c']),
              dict(name='Overlapping group', members=['b', 'c', 'd'])]
    joints, diagnostics = normalize_rigid_connections(bodies, joints, groups)
    return dict(schemaVersion=1, document='Overlapping rigid groups', bodies=bodies,
                joints=joints, rigid_groups=groups, diagnostics=diagnostics)


def create_exports(destination):
    cases = {
        'single_body': dict(schemaVersion=1, document='Oddly named cube',
                            bodies=[body('cube_a', 'Payload / block', 0., True)],
                            joints=[], diagnostics=[]),
        'hinged_pair': dict(schemaVersion=1, document='Bench hinge test',
                            bodies=[body('support_z', 'Mount #7', 0., True),
                                    body('arm_q', 'Link with spaces', .1, False)],
                            joints=[dict(name='Pivot / unusual label', type='Revolute',
                                         base='support_z', follower='arm_q', frame=identity())],
                            diagnostics=[]),
    }
    cases['joint_motion_cases'] = joint_motion_cases()
    cases['rigid_groups'] = rigid_group_case()
    for name, snapshot in cases.items():
        folder = destination / name
        (folder / 'geometry').mkdir(parents=True, exist_ok=True)
        for part in snapshot['bodies']:
            cube_mesh(folder / part['geometry'])
        (folder / 'assembly.json').write_text(json.dumps(snapshot, indent=2))
        write(snapshot, folder / 'assembly.xml')


if __name__ == '__main__':
    create_exports(Path(sys.argv[1]))
