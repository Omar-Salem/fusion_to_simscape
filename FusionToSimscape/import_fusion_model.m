function model = import_fusion_model(exportFolder, options)
%IMPORT_FUSION_MODEL Import a FusionToSimscape export and save its Simulink model.
arguments
    exportFolder (1,1) string = string(fileparts(mfilename('fullpath')))
    options.ModelName (1,1) string = ""
    options.Grounding (1,1) string {mustBeMember(options.Grounding, ["preserve", "fixed"])} = "preserve"
    options.RootBody (1,1) string = ""
end
assert(exist('smimport', 'file') == 2, 'FusionToSimscape:MissingMultibody', ...
    'Install Simscape and Simscape Multibody for this MATLAB release.');
previousFolder = pwd;
cleanup = onCleanup(@() cd(previousFolder));
cd(exportFolder);
exportFolder = string(pwd);
addpath(exportFolder, fullfile(exportFolder, 'geometry'));
snapshot = fusion_snapshot(exportFolder);
model = options.ModelName;
if model == ""
    model = string(matlab.lang.makeValidName(snapshot.document));
end
assert(isvarname(model), 'FusionToSimscape:ModelName', 'ModelName must be a valid MATLAB identifier.');
if options.Grounding ~= "preserve"
    assert(any(string({snapshot.bodies.uid}) == options.RootBody), ...
        'FusionToSimscape:RootBody', 'Select a valid RootBody UID for fixed grounding.');
end
assert(~bdIsLoaded(model), 'FusionToSimscape:ModelAlreadyLoaded', ...
    'Close the existing %s model before importing again.', model);
backupFolder = fullfile(exportFolder, 'history', string(datetime('now', Format='yyyyMMdd_HHmmss_SSS')));
for filename = [model+".slx", model+"_DataFile.m", model+"_manifest.json"]
    if isfile(filename)
        if ~isfolder(backupFolder)
            mkdir(backupFolder);
        end
        movefile(filename, fullfile(backupFolder, filename));
    end
end
xmlPath = fusion_import_xml(exportFolder, options.Grounding, options.RootBody);
xmlCleanup = onCleanup(@() delete(xmlPath));
smimport(xmlPath, 'ModelName', model, 'DataFileName', model + "_DataFile");
set_param(model, 'StopTime', '0.1');
set_param(model, 'SimulationCommand', 'update');
save_system(model, fullfile(exportFolder, model + ".slx"));
fusion_model_manifest(model, exportFolder);
fprintf('Imported and saved %s.slx\n', model);
end
