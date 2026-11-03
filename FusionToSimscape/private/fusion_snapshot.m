function snapshot = fusion_snapshot(exportFolder)
%FUSION_SNAPSHOT Read and validate the supported assembly interchange format.
snapshot = jsondecode(fileread(fullfile(exportFolder, 'assembly.json')));
if any(strcmp(fieldnames(snapshot), 'schemaVersion'))
    assert(snapshot.schemaVersion == 1, 'FusionToSimscape:SnapshotVersion', ...
        'Unsupported assembly snapshot version. Re-export with a compatible add-in.');
end
assert(~isempty(snapshot.bodies), 'FusionToSimscape:InvalidBodies', ...
    'The export contains no visible solid bodies.');
uids = string({snapshot.bodies.uid});
assert(~isempty(uids) && numel(unique(uids)) == numel(uids), ...
    'FusionToSimscape:InvalidBodies', 'Expected nonempty, unique body identifiers.');
for joint = snapshot.joints'
    assert(all(ismember(string({joint.base, joint.follower}), uids)), ...
        'FusionToSimscape:InvalidJoint', 'Joint %s references a missing body.', joint.name);
end
end
