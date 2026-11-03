function fusion_attach_floating_base(model, manifest, world, translation, options)
%FUSION_ATTACH_FLOATING_BASE Insert an assembly-wide free-body pose before the mapped root attachment.
assert(isscalar(manifest.GroundAttachments) && ...
    manifest.GroundAttachments.BodyUid == options.RootBody, ...
    'FusionToSimscape:GroundMapping', 'Expected one mapped attachment for the selected root body.');
root = model + "/" + manifest.GroundAttachments.TransformBlock;
ports = get_param(root,'PortHandles');
delete_line(get_param(ports.LConn(1),'Line'));
free = model + "/FreeBody";
add_block('sm_lib/Joints/6-DOF Joint', free, 'Position',[130 660 180 730]);
axes = ["Px", "Py", "Pz"];
for index = 1:3
    axis = axes(index);
    set_param(free, axis+"PositionTargetSpecify",'on', axis+"PositionTargetPriority",'High', ...
        axis+"PositionTargetValue",num2str(translation(index),17), ...
        axis+"PositionTargetValueUnits",'m', axis+"VelocityTargetSpecify",'on', ...
        axis+"VelocityTargetValue",'0');
end
pose = model + "/InitialPose";
add_block(fusion_library_block('Rigid\s+Transform'),pose,'Position',[210 680 250 720]);
set_param(pose,'RotationMethod','StandardAxis','RotationStandardAxis',options.RotationAxis, ...
    'RotationAngle',num2str(options.RotationAngle,17),'RotationAngleUnits','deg');
worldPorts=get_param(world,'PortHandles'); freePorts=get_param(free,'PortHandles');
posePorts=get_param(pose,'PortHandles');
add_line(model,worldPorts.RConn(1),freePorts.LConn(1),'autorouting','on');
add_line(model,freePorts.RConn(1),posePorts.LConn(1),'autorouting','on');
add_line(model,posePorts.RConn(1),ports.LConn(1),'autorouting','on');
end

