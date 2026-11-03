function validate_general_setup(repository)
%VALIDATE_GENERAL_SETUP Exercise setup with unrelated, synthetic assemblies.
arguments
    repository (1,1) string
end
addpath(fullfile(repository,'FusionToSimscape'));
folder=fullfile(repository,'exports','test_cases','single_body');
model=configure_fusion_model(folder,ModelName="generic_cube_floor", ...
    Grounding="floating",RootBody="cube_a",Floor=true,ContactBodies="cube_a", ...
    PlaceOnFloor=true,SupportBodies="cube_a",ContactDamping=200, ...
    ContactTransitionWidth=2e-4,StopTime=0.1);
manifest=jsondecode(fileread(fullfile(folder,model+"_manifest.json")));
assert(isscalar(manifest.Bodies) && manifest.Bodies.Uid=="cube_a");
assert(abs(manifest.Scene.InitialTranslation(3)+0.075)<1e-7);
result=sim(model,'StopTime','0.1');
log=result.get('fusionLog');
height=values(log.FreeBody.Pz.p.series,Units='m');
assert(abs(height(end)-height(1))<0.002,'Cube should settle on the floor.');
close_system(model,0);

folder=fullfile(repository,'exports','test_cases','hinged_pair');
model=configure_fusion_model(folder,ModelName="generic_hinge_fixed", ...
    Grounding="fixed",RootBody="arm_q",Gravity=[0 -9.80665 0],StopTime=0.05,LogName="jointLog");
manifest=jsondecode(fileread(fullfile(folder,model+"_manifest.json")));
assert(numel(manifest.Bodies)==2 && isscalar(manifest.Joints));
assert(manifest.GroundAttachments.BodyUid=="arm_q");
assert(manifest.Joints.Name=="Pivot / unusual label" && manifest.Joints.Block~="");
assert(isempty(find_system(model,'SearchDepth',1,'Name','Floor')));
result=sim(model,'StopTime','0.05');
assert(result.tout(end)>=0.05);
assert(~isempty(result.get('jointLog')));
close_system(model,0);

% Reject invalid selections before importing or modifying a model.
try
    configure_fusion_model(folder,Grounding="floating",RootBody="missing");
    error('Test:ExpectedFailure','Missing body UID must fail.');
catch exception
    assert(exception.identifier=="FusionToSimscape:RootBody");
end
validate_preserved_import(repository);
fprintf('GENERAL_SETUP_VALIDATION_PASSED\n');
end
