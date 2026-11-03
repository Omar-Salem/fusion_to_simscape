"""Read-only inventory of the active Fusion design for exporter diagnostics."""
import json
from pathlib import Path
import adsk.core
import adsk.fusion


def inspect(output):
    app = adsk.core.Application.get()
    design = adsk.fusion.Design.cast(app.activeProduct)
    root = design.rootComponent
    def occurrence(o):
        return dict(path=o.fullPathName, name=o.name, component=o.component.name,
                    transform=o.transform2.asArray(), grounded=o.isGrounded,
                    visible=o.isVisible, bodies=o.bRepBodies.count, meshes=o.component.meshBodies.count, ground_parent=o.isGroundToParent,
                    children=o.childOccurrences.count)
    def joint(j):
        data = dict(name=j.name)
        for key in ('isSuppressed', 'healthState', 'errorOrWarningMessage', 'occurrenceOne', 'occurrenceTwo'):
            try:
                value = getattr(j, key)
                data[key] = value.fullPathName if hasattr(value, 'fullPathName') else str(value)
            except RuntimeError as error:
                data[key] = str(error)
        data['type'] = j.jointMotion.jointType
        for key in ('geometryOrOriginOne', 'geometryOrOriginTwo', 'geometry', 'geometryOneTransform', 'geometryTwoTransform'):
            try:
                value = getattr(j, key)
                data[key] = value.asArray() if hasattr(value, 'asArray') else str(value)
            except (AttributeError, RuntimeError):
                pass
        return data
    result = dict(document=app.activeDocument.name, root=root.name,
                  occurrences=[occurrence(o) for o in root.allOccurrences],
                  joints=[joint(j) for j in root.allJoints],
                  component_joints=[dict(component=c.name, joints=[joint(j) for j in c.joints], as_built=[joint(j) for j in c.asBuiltJoints]) for c in design.allComponents],
                  as_built=[joint(j) for j in root.allAsBuiltJoints],
                  rigid_groups=[dict(name=g.name, occurrences=[o.fullPathName for o in g.occurrences]) for g in root.allRigidGroups])
    path = Path(output)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(result, indent=2))
    print('Inventory written: ' + str(path))


if __name__ == '__main__':
    inspect(Path(__file__).resolve().parents[1] / 'exports' / 'inventory.json')
