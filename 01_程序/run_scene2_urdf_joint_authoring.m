function run_scene2_urdf_joint_authoring()
%RUN_SCENE2_URDF_JOINT_AUTHORING Visual editor for scene2 joint origins/axes.
%
% Workflow:
%   1. Pick a joint.
%   2. Edit origin xyz, origin rpy, and axis xyz.
%   3. Move the joint-value slider to check the motion.
%   4. Save overrides to regenerate exported/stamp_arm.urdf.

clc; close all;

codeDir = fileparts(mfilename('fullpath'));
sceneDir = fileparts(codeDir);
projectDir = fileparts(sceneDir);
exportDir = fullfile(projectDir, 'exported');
overrideFile = fullfile(exportDir, 'scene2_urdf_joint_overrides.csv');
previewFile = fullfile(exportDir, 'scene2_authoring_preview.urdf');
urdfFile = fullfile(exportDir, 'stamp_arm.urdf');
simscapeFile = fullfile(exportDir, 'stamp_arm_simscape.xml');

cfgDefault = scene2_urdf_joint_config();
cfg = apply_scene2_urdf_joint_overrides(cfgDefault, overrideFile);
movableIdx = find(~strcmp({cfg.joints.type}, 'fixed'));
selectedListIndex = 1;
selectedJointIdx = movableIdx(selectedListIndex);
currentQ = 0;
robot = [];
robotJointNames = {};

fig = figure('Name', 'scene2 URDF joint authoring', ...
    'Color', 'white', ...
    'Units', 'normalized', ...
    'Position', [0.05 0.07 0.90 0.84]);

ax = axes('Parent', fig, 'Units', 'normalized', ...
    'Position', [0.04 0.23 0.68 0.73]);

jointNames = {cfg.joints(movableIdx).name};
uicontrol(fig, 'Style', 'text', 'String', 'Joint', ...
    'Units', 'normalized', 'Position', [0.75 0.91 0.08 0.03], ...
    'BackgroundColor', 'white', 'HorizontalAlignment', 'left');
jointPopup = uicontrol(fig, 'Style', 'popupmenu', 'String', jointNames, ...
    'Units', 'normalized', 'Position', [0.82 0.91 0.15 0.035], ...
    'Callback', @joint_changed);

uicontrol(fig, 'Style', 'text', 'String', 'origin xyz / m', ...
    'Units', 'normalized', 'Position', [0.75 0.83 0.20 0.03], ...
    'BackgroundColor', 'white', 'HorizontalAlignment', 'left');
originEdits = make_edit_row(0.79, @edit_changed);

uicontrol(fig, 'Style', 'text', 'String', 'origin rpy / rad', ...
    'Units', 'normalized', 'Position', [0.75 0.70 0.20 0.03], ...
    'BackgroundColor', 'white', 'HorizontalAlignment', 'left');
rpyEdits = make_edit_row(0.66, @edit_changed);

uicontrol(fig, 'Style', 'text', 'String', 'axis xyz', ...
    'Units', 'normalized', 'Position', [0.75 0.57 0.20 0.03], ...
    'BackgroundColor', 'white', 'HorizontalAlignment', 'left');
axisEdits = make_edit_row(0.53, @edit_changed);

uicontrol(fig, 'Style', 'text', 'String', 'joint value', ...
    'Units', 'normalized', 'Position', [0.75 0.455 0.12 0.03], ...
    'BackgroundColor', 'white', 'HorizontalAlignment', 'left');
qSlider = uicontrol(fig, 'Style', 'slider', ...
    'Units', 'normalized', 'Position', [0.75 0.425 0.22 0.035], ...
    'Callback', @q_changed);
qText = uicontrol(fig, 'Style', 'text', 'String', '', ...
    'Units', 'normalized', 'Position', [0.75 0.39 0.22 0.03], ...
    'BackgroundColor', 'white', 'HorizontalAlignment', 'left');
coupledPreviewCheck = uicontrol(fig, 'Style', 'checkbox', ...
    'String', 'Coupled q preview', ...
    'Units', 'normalized', 'Position', [0.75 0.36 0.22 0.03], ...
    'BackgroundColor', 'white', 'Value', 1, ...
    'Callback', @q_changed);

uicontrol(fig, 'Style', 'pushbutton', 'String', 'Update preview', ...
    'Units', 'normalized', 'Position', [0.75 0.305 0.22 0.045], ...
    'Callback', @update_preview);
uicontrol(fig, 'Style', 'pushbutton', 'String', 'Reset selected joint', ...
    'Units', 'normalized', 'Position', [0.75 0.25 0.22 0.045], ...
    'Callback', @reset_selected_joint);
uicontrol(fig, 'Style', 'pushbutton', 'String', 'Save overrides and URDF', ...
    'Units', 'normalized', 'Position', [0.75 0.195 0.22 0.045], ...
    'Callback', @save_overrides_and_urdf);
uicontrol(fig, 'Style', 'pushbutton', 'String', 'Print selected joint', ...
    'Units', 'normalized', 'Position', [0.75 0.14 0.22 0.045], ...
    'Callback', @print_selected_joint);

note = ['Model/DH axes: X radial, Y lateral, Z vertical. ', ...
    'origin xyz is the joint center in the parent link frame. ', ...
    'axis xyz is the joint motion axis in that same DH-aligned frame. ', ...
    'The STL mesh transform is handled separately from these joint frames.'];
uicontrol(fig, 'Style', 'text', 'String', note, ...
    'Units', 'normalized', 'Position', [0.04 0.035 0.68 0.13], ...
    'BackgroundColor', 'white', 'HorizontalAlignment', 'left');

load_selected_joint_to_controls();
regenerate_preview_robot();
render_preview();

function edits = make_edit_row(y, callback)
    edits = gobjects(1, 3);
    labels = {'x', 'y', 'z'};
    for k = 1:3
        uicontrol(fig, 'Style', 'text', 'String', labels{k}, ...
            'Units', 'normalized', ...
            'Position', [0.75 + (k-1)*0.075, y + 0.04, 0.02, 0.025], ...
            'BackgroundColor', 'white');
        edits(k) = uicontrol(fig, 'Style', 'edit', ...
            'Units', 'normalized', ...
            'Position', [0.75 + (k-1)*0.075, y, 0.065, 0.04], ...
            'Callback', callback);
    end
end

function joint_changed(~, ~)
    selectedListIndex = get(jointPopup, 'Value');
    selectedJointIdx = movableIdx(selectedListIndex);
    currentQ = 0;
    load_selected_joint_to_controls();
    render_preview();
end

function edit_changed(~, ~)
    update_cfg_from_controls();
end

function q_changed(~, ~)
    currentQ = get(qSlider, 'Value');
    set(qText, 'String', sprintf('%.6g', currentQ));
    render_preview();
end

function update_preview(~, ~)
    update_cfg_from_controls();
    regenerate_preview_robot();
    render_preview();
end

function reset_selected_joint(~, ~)
    name = cfg.joints(selectedJointIdx).name;
    defaultIdx = find(strcmp({cfgDefault.joints.name}, name), 1);
    cfg.joints(selectedJointIdx) = cfgDefault.joints(defaultIdx);
    currentQ = 0;
    load_selected_joint_to_controls();
    regenerate_preview_robot();
    render_preview();
end

function save_overrides_and_urdf(~, ~)
    update_cfg_from_controls();
    write_joint_overrides_csv(overrideFile, cfg, movableIdx);
    cfgSaved = apply_scene2_urdf_joint_overrides(scene2_urdf_joint_config(), overrideFile);
    write_scene2_urdf_from_config(cfgSaved, exportDir, urdfFile);
    copyfile(urdfFile, simscapeFile, 'f');
    fprintf('Saved scene2 overrides:\n  %s\n', overrideFile);
    fprintf('Regenerated scene2 URDF:\n  %s\n', urdfFile);
end

function print_selected_joint(~, ~)
    update_cfg_from_controls();
    j = cfg.joints(selectedJointIdx);
    fprintf('\n%s\n', j.name);
    fprintf('  parent -> child: %s -> %s\n', j.parent, j.child);
    fprintf('  origin xyz: [%.6g %.6g %.6g]\n', j.origin);
    fprintf('  origin rpy: [%.6g %.6g %.6g]\n', j.rpy);
    fprintf('  axis xyz:   [%.6g %.6g %.6g]\n', normalize_axis(j.axis));
    if ~isempty(j.limit)
        fprintf('  limit:      [%.6g %.6g]\n', j.limit(1), j.limit(2));
    end
end

function load_selected_joint_to_controls()
    j = cfg.joints(selectedJointIdx);
    for k = 1:3
        set(originEdits(k), 'String', sprintf('%.6g', j.origin(k)));
        set(rpyEdits(k), 'String', sprintf('%.6g', j.rpy(k)));
        set(axisEdits(k), 'String', sprintf('%.6g', j.axis(k)));
    end

    if isempty(j.limit)
        sliderMin = -1;
        sliderMax = 1;
    else
        sliderMin = j.limit(1);
        sliderMax = j.limit(2);
    end
    if sliderMin == sliderMax
        sliderMax = sliderMin + 1;
    end
    set(qSlider, 'Min', sliderMin, 'Max', sliderMax, ...
        'Value', min(max(currentQ, sliderMin), sliderMax));
    currentQ = get(qSlider, 'Value');
    set(qText, 'String', sprintf('%.6g', currentQ));
end

function update_cfg_from_controls()
    origin = read_edit_row(originEdits);
    rpy = read_edit_row(rpyEdits);
    axis = read_edit_row(axisEdits);

    if any(~isfinite(origin))
        error('run_scene2_urdf_joint_authoring:InvalidOrigin', ...
            'Origin values must be finite numbers.');
    end
    if any(~isfinite(rpy))
        error('run_scene2_urdf_joint_authoring:InvalidRpy', ...
            'RPY values must be finite numbers.');
    end
    if any(~isfinite(axis)) || norm(axis) < eps
        error('run_scene2_urdf_joint_authoring:InvalidAxis', ...
            'Axis values must define a nonzero vector.');
    end

    cfg.joints(selectedJointIdx).origin = origin;
    cfg.joints(selectedJointIdx).rpy = rpy;
    cfg.joints(selectedJointIdx).axis = normalize_axis(axis);
    for k = 1:3
        set(axisEdits(k), 'String', ...
            sprintf('%.6g', cfg.joints(selectedJointIdx).axis(k)));
    end
end

function values = read_edit_row(edits)
    values = zeros(1, 3);
    for kk = 1:3
        values(kk) = str2double(get(edits(kk), 'String'));
    end
end

function regenerate_preview_robot()
    write_scene2_urdf_from_config(cfg, exportDir, previewFile);
    robot = importrobot(previewFile);
    configTemplate = homeConfiguration(robot);
    robotJointNames = {configTemplate.JointName};
    robot.DataFormat = 'row';
end

function render_preview()
    if isempty(robot)
        regenerate_preview_robot();
    end

    q = preview_joint_values();

    cla(ax);
    show(robot, q, ...
        'Parent', ax, ...
        'Frames', 'on', ...
        'Visuals', 'on', ...
        'Collisions', 'off', ...
        'PreservePlot', false);
    axis(ax, 'equal');
    grid(ax, 'on');
    view(ax, 42, 26);
    xlim(ax, [-0.10, 0.80]);
    ylim(ax, [-0.40, 0.40]);
    zlim(ax, [-0.10, 0.55]);
    xlabel(ax, 'Model X radial / m');
    ylabel(ax, 'Model Y lateral / m');
    zlabel(ax, 'Model Z vertical / m');
    title(ax, sprintf('%s, q = %.4g', cfg.joints(selectedJointIdx).name, currentQ));
    drawnow;
end

function q = preview_joint_values()
    q = zeros(1, numel(robotJointNames));
    selectedName = cfg.joints(selectedJointIdx).name;

    if get(coupledPreviewCheck, 'Value')
        c = cfg.joints(selectedJointIdx).modelCoefficients;
        nonzero = find(abs(c) > eps);
        if numel(nonzero) == 1
            modelQ = zeros(1, 4);
            modelQ(nonzero) = (currentQ - cfg.joints(selectedJointIdx).modelOffset) / ...
                c(nonzero);
            q = map_model_q_to_robot_order(modelQ);
            return;
        end
    end

    qIndex = find(strcmp(robotJointNames, selectedName), 1);
    if ~isempty(qIndex)
        q(qIndex) = currentQ;
    end
end

function q = map_model_q_to_robot_order(modelQ)
    movableJoints = cfg.joints(~strcmp({cfg.joints.type}, 'fixed'));
    q = zeros(1, numel(robotJointNames));

    for ii = 1:numel(robotJointNames)
        mapIdx = find(strcmp({movableJoints.name}, robotJointNames{ii}), 1);
        if isempty(mapIdx)
            continue;
        end
        q(ii) = modelQ * movableJoints(mapIdx).modelCoefficients(:) + ...
            movableJoints(mapIdx).modelOffset;
    end
end
end

function write_joint_overrides_csv(path, cfg, movableIdx)
fid = fopen(path, 'w');
if fid < 0
    error('run_scene2_urdf_joint_authoring:CannotOpenOverrideFile', ...
        'Cannot write override file: %s', path);
end
cleanup = onCleanup(@() fclose(fid));

fprintf(fid, ['frame_convention,joint,origin_x,origin_y,origin_z,rpy_r,rpy_p,rpy_y,', ...
    'axis_x,axis_y,axis_z,model_offset,lower,upper\n']);
for i = movableIdx(:).'
    j = cfg.joints(i);
    axis = normalize_axis(j.axis);
    if isempty(j.limit)
        lower = 0;
        upper = 0;
    else
        lower = j.limit(1);
        upper = j.limit(2);
    end
    fprintf(fid, ['%s,%s,%.12g,%.12g,%.12g,%.12g,%.12g,%.12g,', ...
        '%.12g,%.12g,%.12g,%.12g,%.12g,%.12g\n'], ...
        cfg.frameConvention, j.name, ...
        j.origin(1), j.origin(2), j.origin(3), ...
        j.rpy(1), j.rpy(2), j.rpy(3), ...
        axis(1), axis(2), axis(3), j.modelOffset, lower, upper);
end
end

function axis = normalize_axis(axis)
if norm(axis) < eps
    axis = [0, 0, 1];
else
    axis = axis ./ norm(axis);
end
end
