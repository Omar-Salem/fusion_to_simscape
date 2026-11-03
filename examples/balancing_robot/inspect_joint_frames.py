"""Compare native and occurrence-context frame conventions."""
import json
from pathlib import Path
import adsk.core
import adsk.fusion

root = adsk.fusion.Design.cast(adsk.core.Application.get().activeProduct).rootComponent
records = []
for owner in root.occurrences:
    for joint in owner.component.joints:
        if joint.name != 'Revolute 8':
            continue
        record = dict(owner=owner.fullPathName, owner_transform=owner.transform2.asArray())
        for kind, item in [('native', joint), ('proxy', joint.createForAssemblyContext(owner))]:
            data = dict(one=item.occurrenceOne.fullPathName, two=item.occurrenceTwo.fullPathName,
                        one_transform=item.occurrenceOne.transform2.asArray(),
                        two_transform=item.occurrenceTwo.transform2.asArray(),
                        geometry_one=item.geometryOneTransform.asArray(),
                        geometry_two=item.geometryTwoTransform.asArray())
            for side in ('One', 'Two'):
                geometry = getattr(item, 'geometryOrOrigin' + side)
                info = dict(type=geometry.objectType)
                for prop in ('origin', 'primaryAxisVector', 'secondaryAxisVector', 'thirdAxisVector'):
                    try:
                        info[prop] = getattr(geometry, prop).asArray()
                    except (AttributeError, RuntimeError):
                        pass
                data['geometry' + side] = info
            record[kind] = data
        records.append(record)
(Path(__file__).resolve().parents[2] / 'exports/assembled/joint_frame_audit.json').write_text(json.dumps(records, indent=2))
print('Joint frame audit saved.')
