function report = validate_rigid_groups(repository)
%VALIDATE_RIGID_GROUPS Check rigid pose preservation and external joint freedoms.
arguments
    repository (1,1) string
end
addpath(fullfile(repository,'FusionToSimscape'));
folder=fullfile(repository,'exports','test_cases','rigid_groups');
snapshot=jsondecode(fileread(fullfile(folder,'assembly.json')));
assert(nnz(string({snapshot.joints.type})=="Weld")==3);
model=configure_fusion_model(folder,ModelName="generic_rigid_groups", ...
    Grounding="fixed",RootBody="a",Gravity=[0 0 0],StopTime=0.1);
cleanup=onCleanup(@() close_system(model,0));
manifest=jsondecode(fileread(fullfile(folder,model+"_manifest.json")));
assert(numel(manifest.Bodies)==6);
assert(isempty(find_system(model,'SearchDepth',1,'ReferenceBlock','sm_lib/Joints/6-DOF Joint')), ...
    'A rigid-group member unexpectedly became a free body.');
workspace=get_param(model,'ModelWorkspace');
data=workspace.evalin('smiData');
assert(abs(sum([data.Solid.mass])-sum([snapshot.bodies.mass]))<1e-12);
maximumPoseError=0;
for joint=snapshot.joints'
    if ~strcmp(joint.type,'Weld')
        continue;
    end
    base=snapshot.bodies(strcmp({snapshot.bodies.uid},joint.base));
    follower=snapshot.bodies(strcmp({snapshot.bodies.uid},joint.follower));
    pair="["+string(joint.base)+":-:"+string(joint.follower)+"]";
    identifiers=string({data.RigidTransform.ID});
    baseLocal=imported_transform(data.RigidTransform(identifiers=="B"+pair));
    followerLocal=imported_transform(data.RigidTransform(identifiers=="F"+pair));
    basePose=reshape(base.transform,4,4)'; followerPose=reshape(follower.transform,4,4)';
    error=max(abs(basePose*baseLocal-followerPose*followerLocal),[],'all');
    maximumPoseError=max(maximumPoseError,error);
end
assert(maximumPoseError<1e-10,'Rigid conversion changed a captured relative pose.');
moving=manifest.Joints(string({manifest.Joints.Type})~="Weld");
assert(numel(moving)==2,'The external hinge and slider must remain independent.');
for joint=moving'
    if string(joint.Type)=="Revolute"
        speed='1'; units='rad/s';
    else
        speed='0.1'; units='m/s';
    end
    set_param(model+"/"+string(joint.Block),'VelocityTargetSpecify','on', ...
        'VelocityTargetValue',speed,'VelocityTargetValueUnits',units);
end
result=sim(model,'StopTime','0.1');
log=result.get('fusionLog');
for joint=moving'
    name=get_param(model+"/"+string(joint.Block),'Name');
    node=log.(name);
    if string(joint.Type)=="Revolute"
        position=values(node.Rz.q.series,Units='rad'); expected=0.1;
    else
        position=values(node.Pz.p.series,Units='m'); expected=0.01;
    end
    assert(abs(position(end)-position(1)-expected)<1e-6, ...
        'Rigid group incorrectly constrained external joint %s.',joint.Name);
end
report=struct('Bodies',numel(manifest.Bodies),'Welds',3,'MovingJoints',numel(moving), ...
    'MaximumRigidPoseError',maximumPoseError,'MassKg',sum([data.Solid.mass]));
writelines(string(jsonencode(report,PrettyPrint=true)),fullfile(folder,'rigid_validation.json'));
disp(report);
fprintf('RIGID_GROUP_VALIDATION_PASSED\n');
end

