"""Independent source measurements for validating the exported snapshot."""
import json
from pathlib import Path
import adsk.core
import adsk.fusion

design = adsk.fusion.Design.cast(adsk.core.Application.get().activeProduct)
root = design.rootComponent
records = []
for occurrence in root.allOccurrences:
    if not occurrence.isVisible:
        continue
    for body in occurrence.bRepBodies:
        props = body.getPhysicalProperties(adsk.fusion.CalculationAccuracy.HighCalculationAccuracy)
        records.append(dict(path=occurrence.fullPathName, body=body.name,
                            center=[v*0.01 for v in props.centerOfMass.asArray()],
                            minimum=[v*0.01 for v in body.boundingBox.minPoint.asArray()],
                            maximum=[v*0.01 for v in body.boundingBox.maxPoint.asArray()]))
axes = []
for occurrence in root.occurrences:
    for joint in occurrence.component.joints:
        if joint.name == 'Revolute 8':
            motion = joint.jointMotion
            axes.append(dict(path=occurrence.fullPathName, axis=motion.rotationAxisVector.asArray(),
                             direction=motion.rotationAxis, value=motion.rotationValue,
                             frame=joint.geometryTwoTransform.asArray()))
(Path(__file__).resolve().parents[2] / 'exports/assembled/source_audit.json').write_text(
    json.dumps(dict(bodies=records, axes=axes), indent=2))
print('Source audit saved.')
