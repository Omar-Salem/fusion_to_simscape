function model = build_falling_model(exportFolder)
%BUILD_FALLING_MODEL Configure the captured balancing robot using the general API.
arguments
    exportFolder (1,1) string = string(fullfile(fileparts(fileparts(fileparts(mfilename('fullpath')))), 'exports', 'assembled'))
end
repository = fileparts(fileparts(fileparts(mfilename('fullpath'))));
addpath(fullfile(repository, 'FusionToSimscape'));
snapshot = jsondecode(fileread(fullfile(exportFolder, 'assembly.json')));
root = snapshot.bodies([snapshot.bodies.grounded]);
assert(isscalar(root), 'The balancing robot example expects one CAD-grounded body.');
wheels = snapshot.bodies(contains(string({snapshot.bodies.occurrence}), 'Wheel'));
assert(numel(wheels) == 2, 'The balancing robot example expects two wheels.');
model = configure_fusion_model(exportFolder, ModelName="assembledV14_falling", ...
    Grounding="floating", RootBody=string(root.uid), RotationAngle=5, ...
    Floor=true, FloorSize=[0.8 0.8], ContactBodies=string({snapshot.bodies.uid})', ...
    PlaceOnFloor=true, SupportBodies=string({wheels.uid})', ...
    ContactStiffness=1e4, ContactDamping=500, ContactTransitionWidth=2e-4, ...
    StaticFriction=0.8, DynamicFriction=0.6, FrictionCriticalVelocity=0.05, ...
    LogName="fallLog");
end
