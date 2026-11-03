# Balancing robot example

This example uses the local `exports/assembled` capture of `assembled v14`, with
`base_link v46`: 13 solids, 10 rigid connections, and two wheel revolute joints.
The original `assembledV14.slx` preserves CAD grounding. The generated
`assembledV14_falling.slx` has a floating base and floor contact.

From the repository folder in MATLAB:

```matlab
addpath('FusionToSimscape','examples/balancing_robot')
model = build_falling_model;
close_system(model,0)
report = validate_falling_model;
```

Close the falling model before rebuilding. The wrapper identifies this example's
two wheels and CAD-grounded root, then calls `configure_fusion_model` with
explicit UIDs. Wheel-name recognition does not occur in the reusable setup.
All 13 solids participate in floor contact. The initial 5° tilt is applied before
mesh-based placement, so the tyres start touching the floor at world Z=0.

The demonstration uses 10,000 N/m stiffness, 500 N·s/m damping, 0.2 mm transition
width, and 0.8/0.6 static/dynamic friction. These values are tuned assumptions for
this robot. They are not the general setup defaults.

Validation checks tipping, initial floor contact, geometric floor penetration,
chassis rebound below 1 mm, and angular overshoot below 1°. Results are written
to `falling_validation.json` and `fallen_pose.png` beside the export.
`validate_simscape.m` separately checks imported mass properties and independent
wheel rotation. The Python audit scripts capture independent measurements from
Fusion for this example's fixtures.
