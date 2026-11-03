% Verify the imported model and independently exercise both wheel freedoms.
repository = fileparts(fileparts(fileparts(mfilename('fullpath'))));
folder = fullfile(repository, 'exports', 'assembled');
addpath(folder, fullfile(folder, 'geometry'));
model = 'assembledV14';
load_system(fullfile(folder, [model '.slx']));
joints = find_system(model, 'FollowLinks', 'on', 'LookUnderMasks', 'all', ...
    'RegExp', 'on', 'Name', '^mounted_motor_v13[12]_Revolute_8$');
assert(numel(joints) == 2, 'Expected two independent wheel joints.');
workspace = get_param(model, 'ModelWorkspace');
data = workspace.evalin('smiData');
snapshot = jsondecode(fileread(fullfile(folder, 'assembly.json')));
assert(numel(data.Solid) == numel(snapshot.bodies));
for index = 1:numel(data.Solid)
    solid = data.Solid(index);
    match = find(strcmp({snapshot.bodies.uid}, solid.ID));
    assert(isscalar(match));
    body = snapshot.bodies(match);
    assert(abs(solid.mass - body.mass) < 1e-12);
    assert(max(abs(solid.CoM(:) - body.center_of_mass(:))) < 1e-12);
    assert(max(abs([solid.MoI(:); solid.PoI(:)] - body.inertia(:))) < 1e-12);
end
set_param(model, 'SimscapeLogType', 'all', 'SimscapeLogName', 'wheelLog');
for index = 1:numel(joints)
    set_param(joints{index}, 'VelocityTargetSpecify', 'on', ...
        'VelocityTargetValue', num2str(index), 'VelocityTargetValueUnits', 'rad/s');
end
result = sim(model, 'StopTime', '0.1', 'ReturnWorkspaceOutputs', 'on');
log = result.get('wheelLog');
report = struct('Model', model, 'Solids', numel(data.Solid), ...
    'RevoluteJoints', numel(joints), 'MassKg', sum([data.Solid.mass]), ...
    'SimulationSeconds', 0.1, 'WheelDisplacementRad', zeros(1, numel(joints)));
for index = 1:numel(joints)
    name = get_param(joints{index}, 'Name');
    jointLog = log.(name);
    displacement = values(jointLog.Rz.q.series, Units='rad');
    report.WheelDisplacementRad(index) = displacement(end)-displacement(1);
    assert(abs(report.WheelDisplacementRad(index) - 0.1*index) < 1e-3, ...
        'Wheel joint must rotate at its initial speed about the centred axle.');
end
file = fopen(fullfile(folder, 'validation.json'), 'w');
fileCleanup = onCleanup(@() fclose(file));
fprintf(file, '%s\n', jsonencode(report, PrettyPrint=true));
disp(report);
disp('SIMSCAPE_VALIDATION_PASSED');
close_system(model, 0);
