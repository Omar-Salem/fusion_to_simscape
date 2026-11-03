"""Serialize a Fusion-independent snapshot to the MathWorks v2 schema."""
from datetime import datetime, timezone
from pathlib import Path
import xml.etree.ElementTree as ET

from .transforms import inverse, multiply

NAMESPACE = 'urn:mathworks:SimscapeMultibody:import'


def element(parent, tag, text=None, **attributes):
    node = ET.SubElement(parent, tag, attributes)
    if text is not None:
        node.text = text
    return node


def numbers(values):
    return ' '.join(format(v, '.17g') for v in values)


def transform(parent, matrix):
    node = element(parent, 'Transform')
    element(node, 'Rotation', numbers([matrix[4*r+c] for r in range(3) for c in range(3)]))
    element(node, 'Translation', numbers([matrix[i] for i in (3, 7, 11)]))


def write(snapshot, destination):
    root = ET.Element('SimscapeMultibodyImportXML', version='2.0', xmlns=NAMESPACE)
    element(root, 'Created', by='FusionToSimscape',
            on=datetime.now(timezone.utc).isoformat(), using='Autodesk Fusion API',
            **{'from': snapshot['document']})
    element(root, 'ModelUnits', mass='kilogram', length='meter')
    element(root, 'DataUnits', mass='kilogram', length='meter')
    assembly = element(root, 'RootAssembly', name=snapshot['document'], uid='assembly')
    tree = element(assembly, 'InstanceTree')
    bodies = {body['uid']: body for body in snapshot['bodies']}
    for body in bodies.values():
        attributes = dict(name=body['name'].replace('/', '__'), uid=body['uid'], entityUid=body['uid'])
        if body['grounded']:
            attributes['grounded'] = 'true'
        node = element(tree, 'Instance', **attributes)
        transform(node, body['transform'])
    joints = element(assembly, 'Joints')
    for joint in snapshot['joints']:
        node = element(joints, joint['type'], name=joint['name'].replace('/', '__'))
        for tag, uid in (('BaseFrame', joint['base']), ('FollowerFrame', joint['follower'])):
            frame = element(node, tag)
            path = element(frame, 'InstancePath')
            element(path, 'Uid', uid)
            # A shared world frame preserves the captured pose with zero joint state.
            transform(frame, multiply(inverse(bodies[uid]['transform']), joint['frame']))
    element(root, 'Assemblies')
    parts = element(root, 'Parts')
    for body in bodies.values():
        part = element(parts, 'Part', name=body['name'].replace('/', '__'), uid=body['uid'])
        mass = element(part, 'MassProperties')
        element(mass, 'Mass', numbers([body['mass']]))
        element(mass, 'CenterOfMass', numbers(body['center_of_mass']))
        element(mass, 'Inertia', numbers(body['inertia']))
        element(part, 'GeometryFile', name=body['geometry'], type='STL')
        visual = element(part, 'VisualProperties')
        r, g, b = body['color']
        for kind in ('Ambient', 'Diffuse', 'Specular'):
            element(visual, kind, r=str(r), g=str(g), b=str(b), a='1')
        element(visual, 'Emissive', r='0', g='0', b='0', a='1')
        element(visual, 'Shininess', '0.3')
    ET.indent(root, space='  ')
    ET.ElementTree(root).write(Path(destination), encoding='utf-8', xml_declaration=True)
