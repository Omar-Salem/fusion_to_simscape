function manifest = fusion_model_manifest(model, exportFolder)
%FUSION_MODEL_MANIFEST Map assembly identifiers to actual imported block paths.
arguments
    model (1,1) string
    exportFolder (1,1) string
end
snapshot = fusion_snapshot(exportFolder);
% smimport's generated data script carries source identifiers independently of labels.
workspace = get_param(model, 'ModelWorkspace');
smiData = workspace.evalin('smiData');
blocks = find_system(model, 'FollowLinks', 'on', 'LookUnderMasks', 'all', 'Type', 'Block');
isSolid = cellfun(@(block) any(strcmp(fieldnames(get_param(block, ...
    'ObjectParameters')), 'ExtGeomFileName')), blocks);
solids = blocks(isSolid);
bodies = repmat(struct('Uid', "", 'Name', "", 'SolidBlock', "", 'Subsystem', ""), ...
    numel(snapshot.bodies), 1);
for index = 1:numel(snapshot.bodies)
    body = snapshot.bodies(index);
    matches = false(size(solids));
    for candidate = 1:numel(solids)
        [~, stem] = fileparts(get_param(solids{candidate}, 'ExtGeomFileName'));
        matches(candidate) = strcmp(stem, body.uid);
    end
    assert(nnz(matches) == 1, 'FusionToSimscape:BodyMapping', ...
        'Cannot uniquely map body %s to an imported File Solid.', body.name);
    solid = string(solids{matches});
    bodies(index) = struct('Uid', string(body.uid), 'Name', string(body.name), ...
        'SolidBlock', relative_path(solid, model), ...
        'Subsystem', relative_path(string(get_param(solid, 'Parent')), model));
end
transforms = find_system(model, 'SearchDepth', 1, ...
    'ReferenceBlock', fusion_library_block('Rigid\s+Transform'));
roots = repmat(struct('BodyUid', "", 'TransformBlock', ""), 0, 1);
for index = 1:numel(transforms)
    expression = get_param(transforms{index}, 'TranslationCartesianOffset');
    token = regexp(expression, 'RigidTransform\((\d+)\)', 'tokens', 'once');
    if isempty(token)
        continue;
    end
    identifier = string(smiData.RigidTransform(str2double(token{1})).ID);
    uid = regexp(identifier, '^RootGround\[(.+)\]$', 'tokens', 'once');
    if ~isempty(uid)
        roots(end+1) = struct('BodyUid', string(uid{1}), ...
            'TransformBlock', relative_path(string(transforms{index}), model)); %#ok<AGROW>
    end
end
jointBlocks = [find_system(model, 'SearchDepth', 1, ...
    'ReferenceBlock', 'sm_lib/Joints/Revolute Joint'); ...
    find_system(model, 'SearchDepth', 1, ...
    'ReferenceBlock', 'sm_lib/Joints/Prismatic Joint')];
joints = repmat(struct('Name', "", 'BaseUid', "", 'FollowerUid', "", ...
    'Type', "", 'Block', ""), numel(snapshot.joints), 1);
for index = 1:numel(snapshot.joints)
    joint = snapshot.joints(index);
    block = "";
    if any(strcmp(joint.type, {'Revolute', 'Prismatic'}))
        expected = "[" + string(joint.base) + ":-:" + string(joint.follower) + "]";
        for candidate = 1:numel(jointBlocks)
            expression = get_param(jointBlocks{candidate}, 'PositionTargetValue');
            pattern = [joint.type 'Joint\((\d+)\)'];
            token = regexp(expression, pattern, 'tokens', 'once');
            dataType = [joint.type 'Joint'];
            if ~isempty(token) && string(smiData.(dataType)(str2double(token{1})).ID) == expected
                assert(block == "", 'FusionToSimscape:JointMapping', ...
                    'Ambiguous imported joint mapping for %s.', joint.name);
                block = relative_path(string(jointBlocks{candidate}), model);
            end
        end
        assert(block ~= "", 'FusionToSimscape:JointMapping', ...
            'Cannot map imported joint %s.', joint.name);
    end
    joints(index) = struct('Name', string(joint.name), 'BaseUid', string(joint.base), ...
        'FollowerUid', string(joint.follower), 'Type', string(joint.type), 'Block', block);
end
manifest = struct('SchemaVersion', 1, 'Model', model, 'Bodies', bodies, ...
    'Joints', joints, 'GroundAttachments', roots);
writelines(string(jsonencode(manifest, PrettyPrint=true)), ...
    fullfile(exportFolder, model + "_manifest.json"));
end

function path = relative_path(block, model)
path = extractAfter(block, strlength(model)+1);
end
