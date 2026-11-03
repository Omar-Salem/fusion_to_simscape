function model = configure_fusion_model(exportFolder, options)
%CONFIGURE_FUSION_MODEL Import CAD and optionally add a floating base and floor.
% Body selections use assembly.json UIDs. Floor and placement use world Z.
arguments
    exportFolder (1,1) string
    options.ModelName (1,1) string = ""
    options.Grounding (1,1) string {mustBeMember(options.Grounding, ["preserve", "fixed", "floating"])} = "preserve"
    options.RootBody (1,1) string = ""
    options.Gravity (1,3) double {mustBeFinite} = [0 0 -9.80665]
    options.InitialTranslation (1,3) double {mustBeFinite} = [0 0 0]
    options.RotationAxis (1,1) string {mustBeMember(options.RotationAxis, ["+X", "+Y", "+Z"])} = "+X"
    options.RotationAngle (1,1) double {mustBeFinite} = 0
    options.Floor (1,1) logical = false
    options.FloorHeight (1,1) double {mustBeFinite} = 0
    options.FloorSize (1,2) double {mustBePositive, mustBeFinite} = [1 1]
    options.ContactBodies (:,1) string = strings(0,1)
    options.PlaceOnFloor (1,1) logical = false
    options.SupportBodies (:,1) string = strings(0,1)
    options.ContactStiffness (1,1) double {mustBePositive, mustBeFinite} = 1e4
    options.ContactDamping (1,1) double {mustBeNonnegative, mustBeFinite} = 100
    options.ContactTransitionWidth (1,1) double {mustBePositive, mustBeFinite} = 1e-3
    options.StaticFriction (1,1) double {mustBeNonnegative, mustBeFinite} = 0.8
    options.DynamicFriction (1,1) double {mustBeNonnegative, mustBeFinite} = 0.6
    options.FrictionCriticalVelocity (1,1) double {mustBePositive, mustBeFinite} = 0.01
    options.StopTime (1,1) double {mustBePositive, mustBeFinite} = 2
    options.LogName (1,1) string = "fusionLog"
    options.MaxStep (1,1) double {mustBePositive, mustBeFinite} = 5e-4
end
previousFolder = pwd;
folderCleanup = onCleanup(@() cd(previousFolder));
cd(exportFolder);
exportFolder = string(pwd);
snapshot = fusion_snapshot(exportFolder);
fusion_validate_setup(snapshot, options);
name = options.ModelName;
if name == ""
    name = string(matlab.lang.makeValidName(string(snapshot.document) + "_simulation"));
end
importGrounding = options.Grounding;
if importGrounding == "floating"
    importGrounding = "fixed";
end
model = import_fusion_model(exportFolder, ModelName=name, ...
    Grounding=importGrounding, RootBody=options.RootBody);
try
    manifest = fusion_model_manifest(model, exportFolder);
    translation = options.InitialTranslation;
    if options.PlaceOnFloor
        translation(3) = options.FloorHeight - fusion_support_minimum(snapshot, exportFolder, options);
    end
    world = fusion_library_instance(model, 'World\s+Frame');
    configuration = fusion_library_instance(model, 'Mechanism\s+Configuration');
    set_param(configuration, 'UniformGravity', 'Constant', 'GravityVector', mat2str(options.Gravity,17));
    if options.Grounding == "floating"
        fusion_attach_floating_base(model, manifest, world, translation, options);
    end
    if options.Floor
        fusion_add_floor_contacts(model, manifest, world, options);
    end
    set_param(model, 'StopTime', num2str(options.StopTime,17), 'Solver', 'ode23t', ...
        'MaxStep', num2str(options.MaxStep,17), 'RelTol', '1e-3', ...
        'SimscapeLogType', 'all', 'SimscapeLogName', options.LogName, ...
        'SimscapeLogLimitData', 'off', 'SimMechanicsOpenEditorOnUpdate', 'on');
    set_param(model, 'SimulationCommand', 'update');
    save_system(model, fullfile(exportFolder, model + ".slx"));
    manifest.Scene = options;
    manifest.Scene.InitialTranslation = translation;
    writelines(string(jsonencode(manifest, PrettyPrint=true)), ...
        fullfile(exportFolder, model + "_manifest.json"));
    fprintf('Configured %s.slx (%s grounding, %d contact bodies).\n', ...
        model, options.Grounding, numel(options.ContactBodies));

catch exception
    close_system(model,0);
    rethrow(exception);
end
end
