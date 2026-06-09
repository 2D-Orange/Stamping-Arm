%RUN_JOINT_PD_TRACKING_URDF_ANIMATION Render PD tracking with SolidWorks URDF.
%
% The CAD/URDF robot shows the actual PD response. Blue overlays show the
% reference stamp path, and red overlays show the actual stamp path.

clear; clc; close all;

codeDir = fileparts(mfilename('fullpath'));
rootDir = fileparts(codeDir);
resultDir = get_output_dir(rootDir, '02_', '02_results');
videoDir = get_output_dir(rootDir, '03_', '03_video');
simFile = fullfile(resultDir, 'joint_pd_tracking_simulation.mat');

if ~exist(simFile, 'file')
    error('run_joint_pd_tracking_urdf_animation:MissingSimulation', ...
        ['PD tracking simulation file not found: %s\nRun this first:\n', ...
        '  run_joint_pd_tracking_simulation'], simFile);
end

if ~exist(videoDir, 'dir')
    mkdir(videoDir);
end

[urdfFile, sourceUrdfFile, packageDir] = prepare_solidworks_urdf(rootDir);
data = load(simFile, 'params', 'targets', 'taskName', 'traj', 'sim', ...
    'desiredStampPath', 'actualStampPath', 'stampError');

robot = importrobot(urdfFile);
configTemplate = homeConfiguration(robot);
robot.DataFormat = 'row';

jointNames = get_robot_joint_names(configTemplate);
actualUrdfQ = map_model_q_to_scene2_urdf_q(data.sim.Q, jointNames);
referenceUrdfQ = map_model_q_to_scene2_urdf_q(data.traj.Q, jointNames);
write_mapping_table(resultDir, data.sim.time, data.traj.Q, data.sim.Q, ...
    referenceUrdfQ, actualUrdfQ, jointNames);

fprintf('Imported scene2 SolidWorks URDF copy for PD tracking:\n  %s\n', urdfFile);
fprintf('Source URDF:\n  %s\n', sourceUrdfFile);
fprintf('Package directory:\n  %s\n', packageDir);
fprintf('URDF joints and model mapping:\n');
for i = 1:numel(jointNames)
    fprintf('  %s\n', jointNames{i});
end

videoBaseFile = fullfile(videoDir, 'scene2_joint_pd_tracking_urdf');
[snapshotFiles, videoFile] = render_urdf_pd_animation(robot, actualUrdfQ, ...
    data, videoBaseFile, resultDir);

fprintf('Saved joint-space PD tracking URDF video to: %s\n', videoFile);
fprintf('Saved snapshots:\n');
for i = 1:numel(snapshotFiles)
    fprintf('  %s\n', snapshotFiles{i});
end

function outputDir = get_output_dir(rootDir, prefix, fallbackName)
items = dir(fullfile(rootDir, [prefix, '*']));
items = items([items.isdir]);

if isempty(items)
    outputDir = fullfile(rootDir, fallbackName);
    return;
end

names = sort({items.name});
outputDir = fullfile(rootDir, names{1});
end

function [importUrdfFile, sourceUrdfFile, packageDir] = prepare_solidworks_urdf(rootDir)
urdfRoots = {
    fullfile(rootDir, 'models', 'urdf')
    fullfile(rootDir, 'urdf')
    };

candidates = [];
searchedRoots = {};
for i = 1:numel(urdfRoots)
    urdfRoot = urdfRoots{i};
    searchedRoots{end+1} = urdfRoot; %#ok<AGROW>
    rootCandidates = dir(fullfile(urdfRoot, '*', 'urdf', '*.urdf'));
    candidates = [candidates; rootCandidates]; %#ok<AGROW>
end

if isempty(candidates)
    error('run_joint_pd_tracking_urdf_animation:MissingScene2Urdf', ...
        'No SolidWorks URDF was found under:\n  %s', ...
        strjoin(searchedRoots, sprintf('\n  ')));
end

[~, newestIndex] = max([candidates.datenum]);
sourceUrdfFile = fullfile(candidates(newestIndex).folder, candidates(newestIndex).name);
packageDir = fileparts(fileparts(sourceUrdfFile));
meshDir = fullfile(packageDir, 'meshes');

if ~exist(meshDir, 'dir')
    error('run_joint_pd_tracking_urdf_animation:MissingMeshDir', ...
        'Mesh directory not found beside the SolidWorks URDF package: %s', meshDir);
end

requiredMeshes = {'base_link.STL', 'link_1.STL', 'link_2.STL', ...
    'link_3.STL', 'link_4.STL', 'link_5.STL'};
for i = 1:numel(requiredMeshes)
    if ~exist(fullfile(meshDir, requiredMeshes{i}), 'file')
        error('run_joint_pd_tracking_urdf_animation:MissingMesh', ...
            'Required mesh not found: %s', fullfile(meshDir, requiredMeshes{i}));
    end
end

importUrdfFile = fullfile(packageDir, 'scene2_solidworks_pd_tracking.urdf');
urdfText = fileread(sourceUrdfFile);
urdfText = regexprep(urdfText, '<robot\s+name="[^"]*"', ...
    '<robot name="scene2_solidworks_pd_tracking"');
urdfText = regexprep(urdfText, 'package://[^/"]+/meshes/', 'meshes/');
urdfText = set_joint_limit(urdfText, 'joint_2', -pi, pi);
urdfText = set_joint_limit(urdfText, 'joint_3', -pi, pi);
urdfText = set_joint_limit(urdfText, 'joint_4', -pi, pi);
urdfText = align_stamp_prismatic_origin(urdfText, stamp_robot_params());
urdfText = recolor_pd_links(urdfText);
write_text_file(importUrdfFile, urdfText);
end

function urdfText = align_stamp_prismatic_origin(urdfText, params)
% Match the SolidWorks joint_5 import frame to the simplified stamp-face
% convention used by forward_kinematics_stamp.
urdfText = set_joint_origin_xyz(urdfText, 'joint_5', ...
    [params.L_axis, -params.H_stamp, 0]);
end

function urdfText = recolor_pd_links(urdfText)
urdfText = set_link_color(urdfText, 'base_link', [0.88, 0.88, 0.84, 1.0]);

movingLinks = {'link_1', 'link_2', 'link_3', 'link_4', 'link_5'};
for i = 1:numel(movingLinks)
    urdfText = set_link_color(urdfText, movingLinks{i}, [0.86, 0.24, 0.18, 1.0]);
end
end

function urdfText = set_link_color(urdfText, linkName, rgba)
linkPattern = ['(<link\s+name="', regexptranslate('escape', linkName), ...
    '"[\s\S]*?</link>)'];
[linkBlock, startIndex, endIndex] = regexp(urdfText, linkPattern, ...
    'match', 'start', 'end', 'once');

if isempty(linkBlock)
    warning('run_joint_pd_tracking_urdf_animation:LinkNotFound', ...
        'Link "%s" was not found while recoloring the URDF.', linkName);
    return;
end

newColorTag = ['<color rgba="', vec_string(rgba), '" />'];
newLinkBlock = regexprep(linkBlock, '<material\s+name="[^"]*">', ...
    ['<material name="', linkName, '_pd_material">']);
newLinkBlock = regexprep(newLinkBlock, '<color\s+rgba="[^"]*"\s*/>', newColorTag);
urdfText = [urdfText(1:startIndex-1), newLinkBlock, urdfText(endIndex+1:end)];
end

function urdfText = set_joint_limit(urdfText, jointName, lowerLimit, upperLimit)
jointPattern = ['(<joint\s+name="', regexptranslate('escape', jointName), ...
    '"[\s\S]*?</joint>)'];
[jointBlock, startIndex, endIndex] = regexp(urdfText, jointPattern, ...
    'match', 'start', 'end', 'once');

if isempty(jointBlock)
    warning('run_joint_pd_tracking_urdf_animation:JointNotFound', ...
        'Joint "%s" was not found while relaxing visualization limits.', jointName);
    return;
end

newJointBlock = regexprep(jointBlock, 'lower="[^"]*"', ...
    ['lower="', sprintf('%.15g', lowerLimit), '"'], 'once');
newJointBlock = regexprep(newJointBlock, 'upper="[^"]*"', ...
    ['upper="', sprintf('%.15g', upperLimit), '"'], 'once');
urdfText = [urdfText(1:startIndex-1), newJointBlock, urdfText(endIndex+1:end)];
end

function urdfText = set_joint_origin_xyz(urdfText, jointName, xyz)
jointPattern = ['(<joint\s+name="', regexptranslate('escape', jointName), ...
    '"[\s\S]*?</joint>)'];
[jointBlock, startIndex, endIndex] = regexp(urdfText, jointPattern, ...
    'match', 'start', 'end', 'once');

if isempty(jointBlock)
    warning('run_joint_pd_tracking_urdf_animation:JointNotFound', ...
        'Joint "%s" was not found while setting origin.', jointName);
    return;
end

newJointBlock = regexprep(jointBlock, 'xyz="[^"]*"', ...
    ['xyz="', vec_string(xyz), '"'], 'once');
urdfText = [urdfText(1:startIndex-1), newJointBlock, urdfText(endIndex+1:end)];
end

function text = vec_string(values)
text = strjoin(arrayfun(@num_string, values(:).', 'UniformOutput', false), ' ');
end

function text = num_string(value)
text = sprintf('%.9g', value);
if ~contains(text, '.') && ~contains(lower(text), 'e')
    text = [text, '.0'];
end
end

function write_text_file(fileName, textValue)
fid = fopen(fileName, 'w', 'n', 'UTF-8');
if fid < 0
    error('run_joint_pd_tracking_urdf_animation:CannotWriteUrdf', ...
        'Cannot write URDF import copy: %s', fileName);
end
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid, '%s', textValue);
end

function jointNames = get_robot_joint_names(configTemplate)
if isstruct(configTemplate)
    jointNames = {configTemplate.JointName};
else
    error('run_joint_pd_tracking_urdf_animation:InvalidConfigurationFormat', ...
        'homeConfiguration(robot) must return a struct array before DataFormat is set to row.');
end
end

function urdfQ = map_model_q_to_scene2_urdf_q(modelQ, jointNames)
urdfQ = zeros(size(modelQ, 1), numel(jointNames));

for j = 1:numel(jointNames)
    switch jointNames{j}
        case {'base_yaw', 'joint_1'}
            urdfQ(:,j) = modelQ(:,1);
        case 'shoulder_pitch'
            urdfQ(:,j) = modelQ(:,2);
        case 'joint_2'
            urdfQ(:,j) = -modelQ(:,2);
        case 'elbow_pitch'
            urdfQ(:,j) = modelQ(:,3);
        case 'joint_3'
            urdfQ(:,j) = -modelQ(:,3);
        case {'wrist_level', 'joint_4'}
            urdfQ(:,j) = modelQ(:,2) + modelQ(:,3);
        case {'stamp_prismatic', 'joint_5'}
            urdfQ(:,j) = modelQ(:,4);
        otherwise
            error('run_joint_pd_tracking_urdf_animation:UnknownJoint', ...
                'No model-to-URDF mapping is defined for joint: %s', jointNames{j});
    end
end
end

function write_mapping_table(resultDir, time, referenceQ, actualQ, ...
    referenceUrdfQ, actualUrdfQ, jointNames)
mappingTable = table((1:numel(time)).', time, ...
    'VariableNames', {'sample', 'time_s'});
refModelTable = array2table(referenceQ(:,1:4), ...
    'VariableNames', {'ref_q1', 'ref_q2', 'ref_q3', 'ref_q4'});
actualModelTable = array2table(actualQ(:,1:4), ...
    'VariableNames', {'actual_q1', 'actual_q2', 'actual_q3', 'actual_q4'});

safeJointNames = matlab.lang.makeValidName(jointNames);
refUrdfTable = array2table(referenceUrdfQ, ...
    'VariableNames', strcat('ref_', safeJointNames));
actualUrdfTable = array2table(actualUrdfQ, ...
    'VariableNames', strcat('actual_', safeJointNames));
mappingTable = [mappingTable, refModelTable, actualModelTable, ...
    refUrdfTable, actualUrdfTable];

writetable(mappingTable, ...
    fullfile(resultDir, 'scene2_joint_pd_urdf_mapping.csv'));
end

function [snapshotFiles, videoFile] = render_urdf_pd_animation(robot, actualUrdfQ, ...
    data, videoBaseFile, resultDir)
videoFps = 30;
frameStride = 5;
frameIdx = 1:frameStride:size(actualUrdfQ, 1);
if frameIdx(end) ~= size(actualUrdfQ, 1)
    frameIdx = [frameIdx, size(actualUrdfQ, 1)];
end

fig = figure('Visible','off','Name','scene2 SolidWorks URDF PD tracking', ...
    'Position',[100 100 1280 900]);
set(fig, 'Color', 'white', 'Renderer', 'opengl');
ax = axes('Parent', fig);

[video, videoFile] = create_video_writer(videoBaseFile);
video.FrameRate = videoFps;
open(video);

snapshotIdx = unique([1, round(size(actualUrdfQ,1)/2), size(actualUrdfQ,1)]);
snapshotNames = {'fig_joint_pd_urdf_start.png', ...
    'fig_joint_pd_urdf_middle.png', ...
    'fig_joint_pd_urdf_end.png'};
snapshotFiles = cell(numel(snapshotIdx), 1);
snapshotCounter = 1;

for n = 1:numel(frameIdx)
    k = frameIdx(n);
    cla(ax);
    show(robot, actualUrdfQ(k,:), ...
        'Parent', ax, ...
        'Frames','off', ...
        'Visuals','on', ...
        'Collisions','off', ...
        'PreservePlot',false);

    hold(ax, 'on');
    draw_work_pads(ax, data.targets, data.taskName);
    draw_task_points(ax, data.targets, data.taskName);
    plot_pd_paths(ax, data, k);
    configure_axes(ax);
    apply_frame_lighting(ax);
    title(ax, sprintf('scene2 SolidWorks URDF PD tracking  t = %.2f s', ...
        data.sim.time(k)));
    draw_status_text(ax, data, k);
    drawnow;

    if snapshotCounter <= numel(snapshotIdx) && k >= snapshotIdx(snapshotCounter)
        snapshotFiles{snapshotCounter} = fullfile(resultDir, snapshotNames{snapshotCounter});
        exportgraphics(fig, snapshotFiles{snapshotCounter}, 'Resolution', 180);
        snapshotCounter = snapshotCounter + 1;
    end

    writeVideo(video, getframe(fig));
end

close(video);
close(fig);
end

function [video, videoFile] = create_video_writer(videoBaseFile)
try
    videoFile = [videoBaseFile, '.mp4'];
    video = VideoWriter(videoFile, 'MPEG-4');
catch
    videoFile = [videoBaseFile, '.avi'];
    video = VideoWriter(videoFile, 'Motion JPEG AVI');
end

if isprop(video, 'Quality')
    video.Quality = 95;
end
end

function plot_pd_paths(ax, data, k)
refPath = data.desiredStampPath;
actualPath = data.actualStampPath;

plot3(ax, refPath(1:k,1), refPath(1:k,2), refPath(1:k,3), ...
    'Color', [0.10 0.36 0.82], 'LineWidth', 1.7);
plot3(ax, actualPath(1:k,1), actualPath(1:k,2), actualPath(1:k,3), ...
    'Color', [0.86 0.20 0.16], 'LineWidth', 2.1);
plot3(ax, refPath(k,1), refPath(k,2), refPath(k,3), 'o', ...
    'MarkerSize', 7, 'MarkerFaceColor', [0.10 0.36 0.82], ...
    'MarkerEdgeColor', [0.05 0.18 0.45]);
plot3(ax, actualPath(k,1), actualPath(k,2), actualPath(k,3), 'o', ...
    'MarkerSize', 7, 'MarkerFaceColor', [0.86 0.20 0.16], ...
    'MarkerEdgeColor', [0.50 0.08 0.06]);
plot3(ax, [refPath(k,1), actualPath(k,1)], ...
    [refPath(k,2), actualPath(k,2)], ...
    [refPath(k,3), actualPath(k,3)], ...
    ':', 'Color', [0.15 0.15 0.15], 'LineWidth', 1.1);
end

function draw_work_pads(ax, targets, taskName)
for i = 1:size(targets, 1)
    name = taskName{i};
    if contains(name, 'ink_press')
        draw_pad(ax, targets(i,1:2), [0.12 0.26 0.46], 0.065);
    elseif contains(name, 'paper_press')
        draw_pad(ax, targets(i,1:2), [0.82 0.78 0.64], 0.075);
    end
end
end

function draw_pad(ax, centerXY, color, sideLength)
halfSide = sideLength / 2;
x = centerXY(1) + [-halfSide halfSide halfSide -halfSide];
y = centerXY(2) + [-halfSide -halfSide halfSide halfSide];
z = 0.002 * ones(1, 4);
patch(ax, x, y, z, color, ...
    'EdgeColor', [0.25 0.25 0.25], ...
    'FaceAlpha', 0.85);
end

function draw_task_points(ax, targets, taskName)
for i = 1:size(targets, 1)
    if contains(taskName{i}, 'press')
        marker = 's';
        color = [0.85 0.20 0.20];
    elseif contains(taskName{i}, 'above')
        marker = '^';
        color = [0.20 0.50 0.85];
    else
        marker = 'o';
        color = [0.30 0.30 0.30];
    end
    plot3(ax, targets(i,1), targets(i,2), targets(i,3), marker, ...
        'MarkerSize', 7, 'MarkerFaceColor', color, 'MarkerEdgeColor', color);
end
end

function draw_status_text(ax, data, k)
qRef = data.traj.Q(k,:);
qActual = data.sim.Q(k,:);
text(ax, 0.02, -0.38, 0.47, sprintf( ...
    ['blue: reference path   red: actual URDF response\n', ...
    'q4 ref %.3f m   q4 actual %.3f m   stamp error %.3f mm'], ...
    qRef(4), qActual(4), 1000 * data.stampError.norm(k)), ...
    'FontSize', 10, 'BackgroundColor', 'white', 'Margin', 3);
end

function apply_frame_lighting(ax)
delete(findall(ax, 'Type', 'light'));

style_lit_meshes(findall(ax, 'Type', 'patch'));
style_lit_meshes(findall(ax, 'Type', 'surface'));

camlight(ax, 'headlight');
camlight(ax, 'right');
lighting(ax, 'gouraud');
material(ax, 'dull');
end

function style_lit_meshes(meshHandles)
for i = 1:numel(meshHandles)
    set(meshHandles(i), ...
        'FaceLighting', 'gouraud', ...
        'AmbientStrength', 0.25, ...
        'DiffuseStrength', 0.78, ...
        'SpecularStrength', 0.18, ...
        'SpecularExponent', 18, ...
        'BackFaceLighting', 'reverselit');
end
end

function configure_axes(ax)
axis(ax, 'equal');
grid(ax, 'on');
view(ax, 42, 26);
xlim(ax, [-0.12, 0.78]);
ylim(ax, [-0.42, 0.42]);
zlim(ax, [-0.16, 0.52]);
set(ax, 'Color', 'white', ...
    'XColor', 'black', 'YColor', 'black', 'ZColor', 'black', ...
    'GridColor', [0.75 0.75 0.75]);
xlabel(ax, 'X / m');
ylabel(ax, 'Y / m');
zlabel(ax, 'Z / m');
end
