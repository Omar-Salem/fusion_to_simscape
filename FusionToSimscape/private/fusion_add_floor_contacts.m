function fusion_add_floor_contacts(model, manifest, world, options)
%FUSION_ADD_FLOOR_CONTACTS Connect explicitly selected CAD collision hulls to a world-Z floor.
frame = model + "/FloorFrame";
add_block(fusion_library_block('Rigid\s+Transform'),frame,'Position',[100 820 140 860]);
set_param(frame,'TranslationMethod','Cartesian','TranslationCartesianOffset', ...
    mat2str([0 0 options.FloorHeight],17),'TranslationCartesianOffsetUnits','m');
plane = model + "/Floor";
add_block('sm_lib/Curves and Surfaces/Infinite Plane',plane,'Position',[200 820 250 870]);
set_param(plane,'GraphicPlaneWidth',num2str(options.FloorSize(1),17), ...
    'GraphicPlaneHeight',num2str(options.FloorSize(2),17),'GraphicDiffuseColor','[0.35 0.4 0.45]');
worldPorts=get_param(world,'PortHandles'); framePorts=get_param(frame,'PortHandles');
planePorts=get_param(plane,'PortHandles');
add_line(model,worldPorts.RConn(1),framePorts.LConn(1),'autorouting','on');
add_line(model,framePorts.RConn(1),planePorts.LConn(1),'autorouting','on');
for index=1:numel(options.ContactBodies)
    body=manifest.Bodies(string({manifest.Bodies.Uid}) == options.ContactBodies(index));
    solid=model+"/"+body.SolidBlock; subsystem=model+"/"+body.Subsystem;
    set_param(solid,'ExportConvexHullGeometry','on');
    existing=find_system(subsystem,'SearchDepth',1,'BlockType','PMIOPort');
    portIndex=max(str2double(get_param(existing,'Port')))+1;
    output=subsystem+"/ContactGeometry";
    add_block('built-in/PMIOPort',output,'Side','Right','Port',num2str(portIndex), ...
        'Position',[510 250 540 270]);
    solidPorts=get_param(solid,'PortHandles'); outputPorts=get_param(output,'PortHandles');
    internal=[outputPorts.LConn outputPorts.RConn];
    assert(isscalar(internal),'FusionToSimscape:GeometryPort','Expected one geometry connection.');
    add_line(subsystem,solidPorts.LConn(1),internal,'autorouting','on');
    contact=model+"/FloorContact_"+index;
    add_block('sm_lib/Forces and Torques/Spatial Contact Force',contact, ...
        'Position',[650+80*mod(index-1,4) 760+90*floor((index-1)/4) 700+80*mod(index-1,4) 810+90*floor((index-1)/4)]);
    set_param(contact,'NormalStiffness',num2str(options.ContactStiffness,17), ...
        'NormalDamping',num2str(options.ContactDamping,17), ...
        'NormalTransitionRegionWidth',num2str(options.ContactTransitionWidth,17), ...
        'FrictionalCriticalVelocity',num2str(options.FrictionCriticalVelocity,17), ...
        'FrictionType','SmoothStickSlip','CoefficientOfStaticFriction',num2str(options.StaticFriction,17), ...
        'CoefficientOfDynamicFriction',num2str(options.DynamicFriction,17));
    contactPorts=get_param(contact,'PortHandles'); geometryPorts=get_param(subsystem,'PortHandles');
    add_line(model,planePorts.RConn(1),contactPorts.LConn(1),'autorouting','on');
    add_line(model,geometryPorts.RConn(end),contactPorts.RConn(1),'autorouting','on');
end
end

