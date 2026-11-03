function validate_preserved_import(repository)
%VALIDATE_PRESERVED_IMPORT Check default naming and unchanged CAD grounding.
arguments
    repository (1,1) string
end
addpath(fullfile(repository,'FusionToSimscape'));
folder=fullfile(repository,'exports','test_cases','hinged_pair');
original=fileread(fullfile(folder,'assembly.xml'));
model=configure_fusion_model(folder,StopTime=0.05);
cleanup=onCleanup(@() close_system(model,0));
assert(model=="BenchHingeTest_simulation");
manifest=jsondecode(fileread(fullfile(folder,model+"_manifest.json")));
assert(string(manifest.GroundAttachments.BodyUid)=="support_z");
assert(strcmp(original,fileread(fullfile(folder,'assembly.xml'))), ...
    'Simulation setup must not modify the CAD interchange file.');
fprintf('PRESERVED_IMPORT_VALIDATION_PASSED\n');
end
