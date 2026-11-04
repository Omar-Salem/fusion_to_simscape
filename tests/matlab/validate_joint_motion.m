function report = validate_joint_motion(repository)
%VALIDATE_JOINT_MOTION Check imported axes, captured poses, and signed motion.
arguments
    repository (1,1) string
end
addpath(fullfile(repository,'FusionToSimscape'));
folder=fullfile(repository,'exports','test_cases','joint_motion_cases');
snapshot=jsondecode(fileread(fullfile(folder,'assembly.json')));
model=configure_fusion_model(folder,ModelName="generic_joint_motion", ...
    Grounding="fixed",RootBody="fixed_mount",Gravity=[0 0 0],StopTime=0.1);
cleanup=onCleanup(@() close_system(model,0));
manifest=jsondecode(fileread(fullfile(folder,model+"_manifest.json")));
assert(nnz(string({manifest.Joints.Type})=="Revolute")==4);
assert(nnz(string({manifest.Joints.Type})=="Prismatic")==4);
workspace=get_param(model,'ModelWorkspace');
data=workspace.evalin('smiData');
maximumAxisError=0;
maximumPoseError=0;
for index=1:numel(manifest.Joints)
    joint=manifest.Joints(index);
    block=model+"/"+string(joint.Block);
    if string(joint.Type)=="Revolute"
        speed=1; units='rad/s';
    else
        speed=0.1; units='m/s';
    end
    set_param(block,'VelocityTargetSpecify','on','VelocityTargetPriority','High', ...
        'VelocityTargetValue',num2str(speed),'VelocityTargetValueUnits',units);
    source=snapshot.joints(strcmp({snapshot.joints.name},joint.Name));
    base=snapshot.bodies(strcmp({snapshot.bodies.uid},source.base));
    follower=snapshot.bodies(strcmp({snapshot.bodies.uid},source.follower));
    basePose=reshape(base.transform,4,4)';
    followerPose=reshape(follower.transform,4,4)';
    pair="["+string(source.base)+":-:"+string(source.follower)+"]";
    identifiers=string({data.RigidTransform.ID});
    baseLocal=imported_transform(data.RigidTransform(identifiers=="B"+pair));
    followerLocal=imported_transform(data.RigidTransform(identifiers=="F"+pair));
    baseFrame=basePose*baseLocal;
    followerFrame=followerPose*followerLocal;
    capturedFrame=reshape(source.frame,4,4)';
    axisError=max(abs(baseFrame(1:3,3)-source.expected_motion_axis(:)));
    poseError=max([abs(baseFrame-capturedFrame),abs(followerFrame-capturedFrame)],[],'all');
    maximumAxisError=max(maximumAxisError,axisError);
    maximumPoseError=max(maximumPoseError,poseError);
end
assert(maximumAxisError<1e-10,'Imported motion axes differ from the source directions.');
assert(maximumPoseError<1e-10,'Joint conversion changed the captured assembly pose.');
result=sim(model,'StopTime','0.1');
log=result.get('fusionLog');
motion=repmat(struct('Name',"",'Type',"",'Displacement',0,'Expected',0),numel(manifest.Joints),1);
for index=1:numel(manifest.Joints)
    joint=manifest.Joints(index);
    name=get_param(model+"/"+string(joint.Block),'Name');
    node=log.(name);
    if string(joint.Type)=="Revolute"
        position=values(node.Rz.q.series,Units='rad'); expected=0.1;
    else
        position=values(node.Pz.p.series,Units='m'); expected=0.01;
    end
    displacement=position(end)-position(1);
    assert(abs(displacement-expected)<1e-6,'Joint %s moved with an incorrect sign or scale.',joint.Name);
    motion(index)=struct('Name',string(joint.Name),'Type',string(joint.Type), ...
        'Displacement',displacement,'Expected',expected);
end
report=struct('MaximumAxisError',maximumAxisError,'MaximumPoseError',maximumPoseError,'Motion',motion);
writelines(string(jsonencode(report,PrettyPrint=true)),fullfile(folder,'motion_validation.json'));
disp(report);
fprintf('JOINT_MOTION_VALIDATION_PASSED\n');
end

