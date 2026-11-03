% Render exported geometry independently of Simscape for placement verification.
function preview_export(folder)
arguments
    folder (1,1) string
end
snapshot = jsondecode(fileread(fullfile(folder, 'assembly.json')));
fig = figure('Visible', 'off', 'Color', [0.96 0.97 0.98]);
axes('Parent', fig);
hold on;
for index = 1:numel(snapshot.bodies)
    body = snapshot.bodies(index);
    mesh = stlread(fullfile(folder, body.geometry));
    transform = reshape(body.transform, 4, 4)';
    vertices = mesh.Points * transform(1:3, 1:3)' + transform(1:3, 4)';
    patch('Faces', mesh.ConnectivityList, 'Vertices', vertices, ...
        'FaceColor', body.color', 'EdgeColor', 'none', ...
        'FaceLighting', 'gouraud', 'AmbientStrength', 0.35);
end
axis equal;
grid on;
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
view(135, 25);
camlight('headlight');
title(sprintf('%s — exported geometry', snapshot.document), 'Interpreter', 'none');
exportgraphics(fig, fullfile(folder, 'geometry_preview.png'), 'Resolution', 160);
close(fig);

end
