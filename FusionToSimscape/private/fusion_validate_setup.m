function fusion_validate_setup(snapshot, options)
%FUSION_VALIDATE_SETUP Reject invalid scene choices before modifying imported model files.
uids = string({snapshot.bodies.uid})';
assert(isvarname(options.LogName), 'FusionToSimscape:LogName', ...
    'LogName must be a valid MATLAB identifier.');
assert(all(ismember([options.ContactBodies; options.SupportBodies], uids)), ...
    'FusionToSimscape:BodySelection', 'Contact and support selections must use exported body UIDs.');
assert(numel(unique(options.ContactBodies)) == numel(options.ContactBodies), ...
    'FusionToSimscape:BodySelection', 'Contact body selections must be unique.');
assert(options.StaticFriction >= options.DynamicFriction, ...
    'FusionToSimscape:Friction', 'Static friction must be at least dynamic friction.');
assert(options.Floor || (isempty(options.ContactBodies) && ~options.PlaceOnFloor), ...
    'FusionToSimscape:FloorRequired', 'Enable Floor before selecting contact or placement.');
assert(~options.Floor || ~isempty(options.ContactBodies), ...
    'FusionToSimscape:ContactSelection', 'Select ContactBodies explicitly when enabling the floor.');
assert(~options.PlaceOnFloor || (options.Grounding == "floating" && ...
    ~isempty(options.SupportBodies) && all(ismember(options.SupportBodies, options.ContactBodies))), ...
    'FusionToSimscape:SupportSelection', 'Floor placement requires floating grounding and support bodies included in ContactBodies.');
assert(options.Grounding == "floating" || ...
    (all(options.InitialTranslation == 0) && options.RotationAngle == 0), ...
    'FusionToSimscape:InitialPose', 'Initial pose adjustments require floating grounding.');
if options.Grounding ~= "preserve"
    assert(ismember(options.RootBody, uids), 'FusionToSimscape:RootBody', 'Select a valid RootBody UID.');
    visited = options.RootBody;
    while true
        previous = numel(visited);
        for joint = snapshot.joints'
            endpoints = string({joint.base, joint.follower})';
            if any(ismember(endpoints, visited))
                visited = unique([visited; endpoints]);
            end
        end
        if numel(visited) == previous
            break;
        end
    end
    assert(numel(visited) == numel(uids), 'FusionToSimscape:DisconnectedAssembly', ...
        'Fixed/floating root setup currently requires one connected assembly. Export disconnected mechanisms separately.');
end
end

