function report = validate_falling_model(exportFolder, result)
%VALIDATE_FALLING_MODEL Check gravity, tipping, and floor support geometrically.
arguments
    exportFolder (1,1) string = string(fullfile(fileparts(fileparts(fileparts(mfilename('fullpath')))), 'exports', 'assembled'))
    result = []
end
addpath(exportFolder, fullfile(exportFolder, 'geometry'));
model = 'assembledV14_falling';
load_system(fullfile(exportFolder, [model '.slx']));
cleanup = onCleanup(@() close_system(model, 0));
set_param(model, 'SimscapeLogLimitData', 'off');
if isempty(result)
    result = sim(model, 'StopTime', '2');
end
log = result.get('fallLog');
t = time(log.FreeBody.Pz.p.series);
p = [values(log.FreeBody.Px.p.series, Units='m'), ...
    values(log.FreeBody.Py.p.series, Units='m'), values(log.FreeBody.Pz.p.series, Units='m')];
q = values(log.FreeBody.S.Q.series);
assert(abs(t(1)) < 1e-12 && t(end) >= 2);
assert(p(1,3)-min(p(:,3)) > 0.02, 'The chassis did not drop.');
angle = 2*acosd(min(1, abs(q(:, 1))));
assert(max(angle) > 45, 'The initially tilted robot did not tip over.');
sampleTime = (0:0.001:2)';
height = interp1(t, p(:,3), sampleTime);
minima = find(height(2:end-1) < height(1:end-2) & ...
    height(2:end-1) <= height(3:end)) + 1;
impact = minima(find(sampleTime(minima) > 0.1, 1));
assert(~isempty(impact), 'Expected a chassis impact during the fall.');
rebound = max(height(impact:end))-height(impact);
angularOvershoot = max(angle)-angle(end);
assert(rebound < 0.001, 'Chassis rebound exceeds 1 mm.');
assert(angularOvershoot < 1, 'Impact rocking exceeds one degree.');

snapshot = jsondecode(fileread(fullfile(exportFolder, 'assembly.json')));
meshes = cell(numel(snapshot.bodies), 1);
points = meshes;
wheelAngles = zeros(numel(t), numel(meshes));
for index = 1:numel(meshes)
    body = snapshot.bodies(index);
    meshes{index} = stlread(fullfile(exportFolder, body.geometry));
    transform = reshape(body.transform, 4, 4)';
    points{index} = meshes{index}.Points*transform(1:3,1:3)' + transform(1:3,4)';
    if contains(body.occurrence, 'Wheel')
        wheelIndex = 1 + contains(body.occurrence, 'v13:2');
        name = sprintf('mounted_motor_v13%d_Revolute_8', wheelIndex);
        wheelAngles(:, index) = values(log.(name).Rz.q.series, Units='rad');
    end
end
samples = unique(round(linspace(1, numel(t), 201)));
lowest = zeros(size(samples));
finalPoints = cell(size(points));
tilt = axis_rotation([1 0 0], deg2rad(5));
for sample = 1:numel(samples)
    step = samples(sample);
    rotation = quaternion_rotation(q(step,:))*tilt;
    lowest(sample) = inf;
    for index = 1:numel(points)
        vertices = points{index};
        body = snapshot.bodies(index);
        if contains(body.occurrence, 'Wheel')
            joint = snapshot.joints(strcmp({snapshot.joints.follower}, body.uid) & ...
                strcmp({snapshot.joints.type}, 'Revolute'));
            frame = reshape(joint.frame, 4, 4)';
            origin = frame(1:3,4)';
            wheelRotation = axis_rotation(frame(1:3,3)', wheelAngles(step,index));
            vertices = (vertices-origin)*wheelRotation' + origin;
        end
        vertices = vertices*rotation' + p(step,:);
        lowest(sample) = min(lowest(sample), min(vertices(:,3)));
        if sample == numel(samples)
            finalPoints{index} = vertices;
        end
    end
end
initialClearance = lowest(1);
assert(abs(initialClearance) < 1e-6, 'The wheels must initially touch the floor.');
penetration = max(0, -min(lowest));
fprintf('Floor penetration: %.9g m; final clearance: %.9g m; final rotation: %.6g deg.\n', penetration, lowest(end), angle(end));
assert(penetration < 0.01, 'A CAD solid passed through the floor.');
assert(abs(lowest(end)) < 0.003, 'The fallen robot is not supported by the floor.');
report = struct('Model', model, 'DurationSeconds', t(end), ...
    'MaximumChassisDropM', p(1,3)-min(p(:,3)), 'FinalRotationDeg', angle(end), ...
    'MaximumSampledFloorPenetrationM', penetration, 'InitialFloorClearanceM', initialClearance, ...
    'ChassisReboundM', rebound, 'AngularOvershootDeg', angularOvershoot);
file = fopen(fullfile(exportFolder, 'falling_validation.json'), 'w');
fileCleanup = onCleanup(@() fclose(file));
fprintf(file, '%s\n', jsonencode(report, PrettyPrint=true));

fig = figure('Visible','off','Color','w');
ax = axes('Parent',fig,'Color','w','XColor','k','YColor','k','ZColor','k');
hold(ax,'on');
patch('Faces',[1 2 3 4], 'Vertices',[-0.4 -0.4 0;0.4 -0.4 0;0.4 0.4 0;-0.4 0.4 0], ...
    'FaceColor',[0.7 0.75 0.8], 'FaceAlpha',0.6);
for index = 1:numel(points)
    patch('Faces',meshes{index}.ConnectivityList,'Vertices',finalPoints{index}, ...
        'FaceColor',snapshot.bodies(index).color','EdgeColor','none','FaceLighting','gouraud');
end
axis equal; xlim([-0.2 0.2]); ylim([-0.25 0.25]); zlim([-0.02 0.2]);
view(125,25); grid on; camlight('headlight');
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('Robot after falling onto the floor','Color','k');
exportgraphics(fig,fullfile(exportFolder,'fallen_pose.png'),'Resolution',160);
close(fig);
disp(report);
disp('FALLING_MODEL_VALIDATION_PASSED');
end

function rotation = quaternion_rotation(q)
q = q/norm(q);
w=q(1); x=q(2); y=q(3); z=q(4);
rotation = [1-2*(y*y+z*z),2*(x*y-w*z),2*(x*z+w*y); ...
    2*(x*y+w*z),1-2*(x*x+z*z),2*(y*z-w*x); ...
    2*(x*z-w*y),2*(y*z+w*x),1-2*(x*x+y*y)];
end

function rotation = axis_rotation(axis, angle)
axis = axis/norm(axis);
cross = [0 -axis(3) axis(2); axis(3) 0 -axis(1); -axis(2) axis(1) 0];
rotation = eye(3)*cos(angle) + (1-cos(angle))*(axis'*axis) + sin(angle)*cross;
end
