# Fusion to Simscape Multibody

A Fusion add-in that exports the visible solid assembly to Simscape Multibody
Import XML v2, with STL geometry in metres and CAD mass properties in SI units.
MATLAB imports the CAD assembly; an optional, separate setup function configures
gravity, grounding, a floating base, and floor contact.

## Export and import

1. In Fusion, open **Utilities → Add-ins → Scripts and Add-ins**.
2. Add the `FusionToSimscape` folder and run the add-in.
3. Choose **Export to Simscape Multibody** and select a parent folder. The add-in
   creates a subfolder named after the active model, containing all export files.
   For example, selecting `/exports` for `assembled v14` writes to
   `/exports/assembled v14/`. Characters invalid in folder names are replaced with
   underscores. Re-exporting the same model uses its existing subfolder.
4. In MATLAB with Simscape and Simscape Multibody installed:

```matlab
cd('/exports/assembled v14')
model = import_fusion_model;
```

Exports include `assembly.xml`, a versioned `assembly.json`, `geometry/*.stl`,
and the MATLAB helpers with their `private/` folder. Keep these files together.
Import preserves CAD grounding by default. Model names come from the document
name unless `ModelName` is supplied. Existing generated models/data files move
into `history/` before replacement; loaded models must be closed first.

Each import writes `<model>_manifest.json`: body UIDs map to solid blocks and
subsystems; revolute and prismatic joints map to imported blocks using the source identifiers
in MATLAB's import data; CAD world attachments map to their rigid transforms.
Paths in the manifest are relative to the model. Welds are recorded as rigid
connections, without requiring a separate joint block.

## Configure a simulation

Simulation choices are explicit rather than inferred from component names.
Inspect `assembly.json` to select exported body UIDs:

```matlab
folder = "/path/to/export";
snapshot = jsondecode(fileread(fullfile(folder,"assembly.json")));
disp(struct2table(snapshot.bodies))

% Preserve the CAD's geometry, joints, and grounding, with explicit gravity.
model = configure_fusion_model(folder, ModelName="cad_simulation", ...
    Gravity=[0 0 -9.80665]);
```

For a floating assembly, choose a root body, contact bodies, and support bodies.
The identifiers below are placeholders to replace with UIDs from your export:

```matlab
model = configure_fusion_model(folder, ModelName="floor_simulation", ...
    Grounding="floating", RootBody="root_body_uid", ...
    Floor=true, FloorHeight=0, FloorSize=[1 1], ...
    ContactBodies=["root_body_uid"; "other_body_uid"], ...
    PlaceOnFloor=true, SupportBodies="other_body_uid", ...
    RotationAxis="+X", RotationAngle=5, ...
    ContactStiffness=1e4, ContactDamping=100, ...
    ContactTransitionWidth=1e-3, ...
    StaticFriction=0.8, DynamicFriction=0.6, ...
    FrictionCriticalVelocity=0.01, StopTime=2);
```

`Grounding="preserve"` retains CAD grounding. `Grounding="fixed"` grounds only
the selected root body. `Grounding="floating"` replaces that attachment with a
6-DOF joint, with zero initial translational velocity. Fixed/floating root
selection currently requires a connected assembly; disconnected mechanisms must
be exported separately. CAD grounding overrides are made in a temporary XML
copy, leaving the exported XML and Fusion design unchanged.

The floor is an infinite plane normal to world +Z; `FloorSize` controls its
visible dimensions. `PlaceOnFloor` computes vertical placement from the selected
support meshes after initial rotation. `InitialTranslation` supplies an optional
world translation; floor placement calculates its Z coordinate. Initial pose
adjustments require floating grounding. Each selected contact body uses its
convex hull, which can bridge holes and concave regions. Contact parameters are
simulation assumptions to tune for each mechanism, not measured material data.
The chosen settings and calculated translation are stored in the model manifest.
Simscape logging uses `fusionLog` by default; `LogName` selects another output name.

## Translation support and limits

- Visible solid bodies, occurrence placement, CAD mass, centre of mass, inertia,
  and component colours are exported. Multiple bodies in a component are welded.
- Joint frames use solved Fusion assembly-context placements, including flexible
  subassemblies. Hierarchy is flattened; full occurrence paths remain in metadata.
- Rigid, revolute, and slider joints are supported. Sliders become Simscape
  prismatic joints. Revolute and slider X/Y/Z directions are mapped onto the
  exported joint frame's Z axis; custom direction vectors are transported from
  the native frame into the solved assembly frame. Origins and captured poses
  are preserved, including positive motion direction.
- Rigid groups are exported as a minimal weld forest. Overlapping groups,
  multi-body component welds, and existing rigid joints are combined without
  adding redundant rigid loops. Suppressed groups are skipped. Members with no
  visible solid body are excluded with diagnostics; descendants are included
  only when Fusion reports them as group members.
- Relative motion outside a rigid group remains available. A revolute or slider
  joint whose endpoints are rigidly connected is reported as `locked` and omitted
  from the moving-joint list. Redundant explicit welds are reported as `redundant`.
  Group names and exported body membership are retained in `rigid_groups` in
  `assembly.json`.
- Other joint types and mesh bodies still fail explicitly.
- The captured CAD pose becomes the joint reference configuration. Joint limits,
  motion relationships, motors, controllers, and other environments are not
  translated automatically.
- Hidden, empty, and reference-only joint endpoints are excluded with diagnostics.
- Snapshot schema version 1 is emitted. MATLAB also accepts the original
  unversioned snapshots from this repository; unknown versions are rejected.

## Balancing robot example

Robot names, wheel selection, tuned contact settings, and robot-specific
validation live in [`examples/balancing_robot`](examples/balancing_robot/README.md).
The example calls the same configuration API as any other assembly. Generated
robot exports and simulation results remain local under `exports/`, ignored by
Git; small captured JSON fixtures are retained for regression tests.

## Validation

Python checks run without Fusion or MATLAB:

```sh
python3 -m unittest discover -s tests -v
```

Create unrelated cube, hinge, joint-motion, and rigid-group exports, then exercise MATLAB:

```sh
PYTHONPATH=. python3 tests/create_test_exports.py exports/test_cases
xmllint --noout --schema reference/SimscapeMultibodyImportXML.xsd exports/test_cases/single_body/assembly.xml
xmllint --noout --schema reference/SimscapeMultibodyImportXML.xsd exports/test_cases/hinged_pair/assembly.xml
xmllint --noout --schema reference/SimscapeMultibodyImportXML.xsd exports/test_cases/joint_motion_cases/assembly.xml
xmllint --noout --schema reference/SimscapeMultibodyImportXML.xsd exports/test_cases/rigid_groups/assembly.xml
```

```matlab
addpath('tests/matlab')
validate_general_setup(string(pwd))
validate_joint_motion(string(pwd))
validate_rigid_groups(string(pwd))
```

The joint-motion check imports eight joints (X, Y, Z, and custom directions for
both revolute and slider motion). It checks imported world axes and captured
poses, then exercises positive angular and linear velocities to verify sign and
SI-unit scale. Python tests also cover negative axes, solved proxy transport,
and both standard and as-built joint API adapters. Synthetic tests complement
the captured Fusion regression data; they do not replace live Fusion testing.

The rigid-group check combines overlapping groups and duplicate welds, checks
relative frames and total mass after import, and exercises an external hinge
and slider to verify that both remain free. Python checks cover suppression,
excluded members, repeated instances, and joints intentionally locked by a group.

The setup checks cover a single floating body placed on the floor, a two-body
hinge grounded at a different body than in the CAD export, arbitrary component
names, gravity, block mapping, simulation, and invalid body selection.

`tools/preview_export.m` renders any export folder without Simscape.
Fusion developer scripts should be loaded with `runpy.run_path` so their own
file locations resolve correctly; `tools/export_active.py` opens the export
folder picker and `tools/inspect_fusion.py` writes a diagnostic inventory.

## References

- [MathWorks CAD translation](https://www.mathworks.com/help/sm/ug/cad-translation.html)
- [Official XML schemas](https://www.mathworks.com/products/simscape-multibody/download_smxmlschema.html)
- [smimport](https://www.mathworks.com/help/sm/ref/smimport.html)
- [Fusion joint directions](https://help.autodesk.com/cloudhelp/ENU/Fusion-360-API/files/fusion_JointDirections.htm)
- [Fusion rigid groups](https://help.autodesk.com/cloudhelp/ENU/Fusion-360-API/files/fusion_RigidGroup.htm)
- [Fusion slider joint motion](https://help.autodesk.com/cloudhelp/ENU/Fusion-360-API/files/fusion_SliderJointMotion.htm)

Files under `reference/` are unmodified MathWorks reference downloads for schema
validation; they are not part of the add-in runtime.
