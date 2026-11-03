function minimum = fusion_support_minimum(snapshot, folder, options)
%FUSION_SUPPORT_MINIMUM Find the lowest selected support-mesh vertex after initial rotation.
vector = double(options.RotationAxis == ["+X", "+Y", "+Z"])';
skew = [0 -vector(3) vector(2); vector(3) 0 -vector(1); -vector(2) vector(1) 0];
angle = deg2rad(options.RotationAngle);
rotation = eye(3)*cos(angle) + (1-cos(angle))*(vector*vector') + sin(angle)*skew;
minimum = inf;
for body = snapshot.bodies'
    if ~ismember(string(body.uid), options.SupportBodies)
        continue;
    end
    mesh = stlread(fullfile(folder, body.geometry));
    transform = reshape(body.transform,4,4)';
    points = (mesh.Points*transform(1:3,1:3)' + transform(1:3,4)')*rotation';
    minimum = min(minimum, min(points(:,3)));
end
assert(isfinite(minimum), 'FusionToSimscape:SupportGeometry', 'Support geometry has no finite vertices.');
end

