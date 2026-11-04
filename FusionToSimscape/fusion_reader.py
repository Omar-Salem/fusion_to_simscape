"""Read assembly geometry and connections without changing the Fusion design."""
import hashlib
from pathlib import Path
import struct

import adsk.core
import adsk.fusion

from .joint_frames import motion_frame
from .rigid_groups import normalize_rigid_connections
from .transforms import identity, metres, principal_inertia, triangle_normal

JOINT_TYPES = {
    adsk.fusion.JointTypes.RigidJointType: 'Weld',
    adsk.fusion.JointTypes.RevoluteJointType: 'Revolute',
    adsk.fusion.JointTypes.SliderJointType: 'Prismatic',
}


def uid(text):
    return 'b_' + hashlib.sha256(text.encode()).hexdigest()[:16]


def write_mesh(body, destination):
    calculator = body.meshManager.createMeshCalculator()
    calculator.setQuality(adsk.fusion.TriangleMeshQualityOptions.HighQualityTriangleMesh)
    mesh = calculator.calculate()
    if not mesh or not mesh.triangleCount:
        raise ValueError('No mesh triangles for ' + body.name)
    nodes = mesh.nodeCoordinatesAsDouble
    indices = mesh.nodeIndices
    with Path(destination).open('wb') as stream:
        stream.write(b'FusionToSimscape; coordinates in metres'.ljust(80, b' '))
        stream.write(struct.pack('<I', mesh.triangleCount))
        for offset in range(0, len(indices), 3):
            vertices = [[nodes[3*i+k]*0.01 for k in range(3)] for i in indices[offset:offset+3]]
            values = triangle_normal(vertices) + [v for vertex in vertices for v in vertex]
            stream.write(struct.pack('<12fH', *values, 0))
    return mesh.triangleCount


class FusionReader:
    def __init__(self, design, output):
        self.design = design
        self.root = design.rootComponent
        self.output = Path(output)
        self.bodies = []
        self.joints = []
        self.occurrence_bodies = {}
        self.diagnostics = []
        self.occurrences = {}
        self.rigid_groups = []

    def read(self):
        self.output.mkdir(parents=True, exist_ok=True)
        (self.output / 'geometry').mkdir(exist_ok=True)
        self._visit(self.root, '', identity(), True)
        for path, occurrence in self.occurrences.items():
            self._connections(occurrence.component, path, occurrence)
        self._connections(self.root, '', None)
        self.joints, rigid_diagnostics = normalize_rigid_connections(self.bodies, self.joints, self.rigid_groups)
        self.diagnostics.extend(rigid_diagnostics)
        return dict(schemaVersion=1, document=self.design.parentDocument.name, bodies=self.bodies,
                    joints=self.joints, rigid_groups=self.rigid_groups, diagnostics=self.diagnostics)

    def _visit(self, component, path, world, grounded):
        if component.meshBodies.count:
            raise ValueError('Mesh bodies require explicit mass properties and are not supported: ' + path)
        owner_bodies = []
        for body in component.bRepBodies:
            if not body.isSolid or not body.isVisible:
                continue
            key = uid(path + '/' + body.name)
            geometry = 'geometry/' + key + '.stl'
            props = body.getPhysicalProperties(adsk.fusion.CalculationAccuracy.HighCalculationAccuracy)
            ok_moments, *moments = props.getPrincipalMomentsOfInertia()
            ok_axes, *axes = props.getPrincipalAxes()
            if not ok_moments or not ok_axes or props.mass <= 0:
                raise ValueError('Invalid mass properties: ' + path + '/' + body.name)
            count = write_mesh(body, self.output / geometry)
            color = component.componentColor
            self.bodies.append(dict(uid=key, name=(path or component.name) + '/' + body.name,
                                    occurrence=path, transform=world, grounded=grounded,
                                    mass=props.mass, center_of_mass=[v*0.01 for v in props.centerOfMass.asArray()],
                                    inertia=principal_inertia(moments, [v.asArray() for v in axes]),
                                    geometry=geometry, triangles=count,
                                    color=[color.red/255, color.green/255, color.blue/255]))
            owner_bodies.append(key)
        self.occurrence_bodies[path] = owner_bodies
        for other in owner_bodies[1:]:
            self._add_joint(path + '/BodyWeld', 'Weld', owner_bodies[0], other, world)
        # Traverse the real occurrence tree. allComponents also contains reference-only entities.
        children = self.root.occurrences if not path else self.occurrences[path].childOccurrences
        for occurrence in children:
            if not occurrence.isVisible:
                continue
            child_path = occurrence.fullPathName
            self.occurrences[child_path] = occurrence
            child_world = metres(occurrence.transform2.asArray())
            child_grounded = occurrence.isGrounded or (grounded and occurrence.isGroundToParent)
            self._visit(occurrence.component, child_path, child_world, child_grounded)

    def _occurrence_path(self, owner, endpoint):
        if endpoint is None:
            return None
        relative = endpoint.fullPathName
        candidate = owner + '+' + relative if owner else relative
        if candidate in self.occurrence_bodies:
            return candidate
        if relative in self.occurrence_bodies and (not owner or relative.startswith(owner + '+')):
            return relative
        return None

    def _resolve(self, owner, endpoint):
        path = self._occurrence_path(owner, endpoint)
        keys = self.occurrence_bodies.get(path, [])
        return keys[0] if keys else None

    def _connections(self, component, owner, context):
        for collection in (component.joints, component.asBuiltJoints):
            for native in collection:
                label = (owner + '/' if owner else '') + native.name
                try:
                    if native.isSuppressed:
                        continue
                    base = self._resolve(owner, native.occurrenceTwo)
                    follower = self._resolve(owner, native.occurrenceOne)
                    if not base or not follower:
                        self.diagnostics.append(dict(name=label, status='excluded',
                                                     reason='Endpoint has no visible solid body (reference-only, hidden, or empty).'))
                        continue
                    joint = native.createForAssemblyContext(context) if context else native
                    kind = JOINT_TYPES.get(joint.jointMotion.jointType)
                    if kind is None:
                        raise ValueError('Unsupported joint type: ' + label)
                    # Proxy geometry includes solved flexible-subassembly placement.
                    frame = self._joint_frame(joint)
                    if kind in ('Revolute', 'Prismatic'):
                        motion = joint.jointMotion
                        direction = motion.rotationAxis if kind == 'Revolute' else motion.slideDirection
                        if direction == adsk.fusion.JointDirections.CustomJointDirection:
                            native_motion = native.jointMotion
                            vector = (native_motion.rotationAxisVector if kind == 'Revolute'
                                      else native_motion.slideDirectionVector)
                            if vector is None:
                                raise ValueError('Missing custom joint axis: ' + label)
                            frame = motion_frame(frame, direction, vector.asArray(), self._joint_frame(native))
                        else:
                            frame = motion_frame(frame, direction)
                    self._add_joint(label, kind, base, follower, frame)
                except RuntimeError as error:
                    raise ValueError('Cannot read joint ' + label + ': ' + str(error)) from error
        self._collect_rigid_groups(component, owner)

    def _collect_rigid_groups(self, component, owner):
        for index in range(component.rigidGroups.count):
            group = component.rigidGroups.item(index)
            if group.isSuppressed:
                continue
            label = (owner + '/' if owner else '') + group.name
            members = []
            for occurrence in group.occurrences:
                path = self._occurrence_path(owner, occurrence)
                bodies = self.occurrence_bodies.get(path, [])
                if bodies:
                    members.extend(bodies)
                else:
                    self.diagnostics.append(dict(name=label, status='excluded',
                                                 member=occurrence.fullPathName,
                                                 reason='Rigid-group member has no visible solid body.'))
            self.rigid_groups.append(dict(name=label, members=sorted(set(members))))

    @staticmethod
    def _joint_frame(joint):
        geometry = joint.geometryTwoTransform if isinstance(joint, adsk.fusion.Joint) else joint.transform
        return metres(geometry.asArray())

    def _add_joint(self, name, kind, base, follower, frame):
        self.joints.append(dict(name=name, type=kind, base=base, follower=follower, frame=frame))
