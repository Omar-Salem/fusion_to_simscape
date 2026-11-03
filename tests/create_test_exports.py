"""Create small, model-independent CAD interchange cases for MATLAB checks."""
from pathlib import Path
import json
import struct
import sys

from FusionToSimscape.transforms import identity, triangle_normal
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
    for name, snapshot in cases.items():
        folder = destination / name
        (folder / 'geometry').mkdir(parents=True, exist_ok=True)
        for part in snapshot['bodies']:
            cube_mesh(folder / part['geometry'])
        (folder / 'assembly.json').write_text(json.dumps(snapshot, indent=2))
        write(snapshot, folder / 'assembly.xml')


if __name__ == '__main__':
    create_exports(Path(sys.argv[1]))
