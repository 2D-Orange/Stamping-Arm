%RUN_SCENE2_URDF_KINEMATICS_VISUALIZATION Visualize trajectory with the
%newly exported scene2 SolidWorks URDF.

clear; clc; close all;

codeDir = fileparts(mfilename('fullpath'));
rootDir = fileparts(codeDir);
resultDir = get_output_dir(rootDir, '02_', '02_results');
videoDir = get_output_dir(rootDir, '03_', '03_video');
trajFile = fullfile(resultDir, 'trajectory_planning.mat');
sampleCsvFile = fullfile(resultDir, 'trajectory_samples.csv');

if ~exist(trajFile, 'file') && ~exist(sampleCsvFile, 'file')
    error('run_scene2_urdf_kinematics_visualization:MissingTrajectory', ...
        ['Trajectory files not found:\n  %s\n  %s\nRun this first:\n', ...
        '  run_trajectory_planning'], trajFile, sampleCsvFile);
end

if ~exist(videoDir, 'dir')
    mkdir(videoDir);
end

[urdfFile, sourceUrdfFile, packageDir] = prepare_solidworks_urdf(rootDir);
[traj, params, stampPath] = load_trajectory_data(trajFile, sampleCsvFile);

robot = importrobot(urdfFile);
configTemplate = homeConfiguration(robot);
robot.DataFormat = 'row';

jointNames = get_robot_joint_names(configTemplate);
urdfQ = map_model_q_to_scene2_urdf_q(traj.Q, jointNames);
write_mapping_table(resultDir, traj, urdfQ, jointNames);
validationTable = validate_urdf_mapping(robot, urdfQ, traj.Q, params, resultDir);

fprintf('Imported scene2 SolidWorks URDF copy:\n  %s\n', urdfFile);
fprintf('Source URDF:\n  %s\n', sourceUrdfFile);
fprintf('Package directory:\n  %s\n', packageDir);
fprintf('URDF joints and model mapping:\n');
for i = 1:numel(jointNames)
    fprintf('  %s\n', jointNames{i});
end
fprintf('Max simplified FK vs URDF link_5 stamp-face check error: %.3e m\n', ...
    max(validationTable.position_error_m));

videoBaseFile = fullfile(videoDir, 'scene2_urdf_kinematics');
[snapshotFiles, videoFile] = render_urdf_animation(robot, urdfQ, traj.time, ...
    stampPath, resultDir, videoBaseFile);

fprintf('Saved scene2 URDF kinematics video to: %s\n', videoFile);
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
    error('run_scene2_urdf_kinematics_visualization:MissingScene2Urdf', ...
        'No SolidWorks URDF was found under:\n  %s', ...
        strjoin(searchedRoots, sprintf('\n  ')));
end

[~, newestIndex] = max([candidates.datenum]);
sourceUrdfFile = fullfile(candidates(newestIndex).folder, candidates(newestIndex).name);
packageDir = fileparts(fileparts(sourceUrdfFile));
meshDir = fullfile(packageDir, 'meshes');

if ~exist(meshDir, 'dir')
    error('run_scene2_urdf_kinematics_visualization:MissingMeshDir', ...
        'Mesh directory not found beside the SolidWorks URDF package: %s', meshDir);
end

requiredMeshes = {'base_link.STL', 'link_1.STL', 'link_2.STL', ...
    'link_3.STL', 'link_4.STL', 'link_5.STL'};
for i = 1:numel(requiredMeshes)
    if ~exist(fullfile(meshDir, requiredMeshes{i}), 'file')
        error('run_scene2_urdf_kinematics_visualization:MissingMesh', ...
            'Required mesh not found: %s', fullfile(meshDir, requiredMeshes{i}));
    end
end

importUrdfFile = fullfile(packageDir, 'scene2_solidworks_import.urdf');
urdfText = fileread(sourceUrdfFile);
urdfText = regexprep(urdfText, '<robot\s+name="[^"]*"', ...
    '<robot name="scene2_solidworks_stamp_arm"');
urdfText = regexprep(urdfText, 'package://[^/"]+/meshes/', 'meshes/');
urdfText = set_joint_limit(urdfText, 'joint_2', -pi, pi);
urdfText = set_joint_limit(urdfText, 'joint_3', -pi, pi);
urdfText = set_joint_limit(urdfText, 'joint_4', -pi, pi);
urdfText = recolor_solidworks_links(urdfText);
write_text_file(importUrdfFile, urdfText);
end

function urdfText = recolor_solidworks_links(urdfText)
% Keep the table/background light and make the moving robot links gray.
urdfText = set_link_color(urdfText, 'base_link', [0.88, 0.88, 0.84, 1.0]);

grayLinks = {'link_1', 'link_2', 'link_3', 'link_4', 'link_5'};
for i = 1:numel(grayLinks)
    urdfText = set_link_color(urdfText, grayLinks{i}, [0.42, 0.42, 0.42, 1.0]);
end
end

function urdfText = set_link_color(urdfText, linkName, rgba)
linkPattern = ['(<link\s+name="', regexptranslate('escape', linkName), ...
    '"[\s\S]*?</link>)'];
[linkBlock, startIndex, endIndex] = regexp(urdfText, linkPattern, ...
    'match', 'start', 'end', 'once');

if isempty(linkBlock)
    warning('run_scene2_urdf_kinematics_visualization:LinkNotFound', ...
        'Link "%s" was not found while recoloring the URDF.', linkName);
    return;
end

newColorTag = ['<color rgba="', vec_string(rgba), '" />'];
newLinkBlock = regexprep(linkBlock, '<material\s+name="[^"]*">', ...
    ['<material name="', linkName, '_material">']);
newLinkBlock = regexprep(newLinkBlock, '<color\s+rgba="[^"]*"\s*/>', newColorTag);
urdfText = [urdfText(1:startIndex-1), newLinkBlock, urdfText(endIndex+1:end)];
end

function urdfText = set_joint_limit(urdfText, jointName, lowerLimit, upperLimit)
jointPattern = ['(<joint\s+name="', regexptranslate('escape', jointName), ...
    '"[\s\S]*?</joint>)'];
[jointBlock, startIndex, endIndex] = regexp(urdfText, jointPattern, ...
    'match', 'start', 'end', 'once');

if isempty(jointBlock)
    warning('run_scene2_urdf_kinematics_visualization:JointNotFound', ...
        'Joint "%s" was not found while relaxing visualization limits.', jointName);
    return;
end

newJointBlock = regexprep(jointBlock, 'lower="[^"]*"', ...
    ['lower="', sprintf('%.15g', lowerLimit), '"'], 'once');
newJointBlock = regexprep(newJointBlock, 'upper="[^"]*"', ...
    ['upper="', sprintf('%.15g', upperLimit), '"'], 'once');
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
    error('run_scene2_urdf_kinematics_visualization:CannotWriteUrdf', ...
        'Cannot write URDF import copy: %s', fileName);
end
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid, '%s', textValue);
end

function stampPath = compute_stamp_path(traj, params)
stampPath = zeros(size(traj.Q, 1), 3);
for k = 1:size(traj.Q, 1)
    kin = forward_kinematics_stamp(traj.Q(k,:), params);
    stampPath(k,:) = kin.p_stamp.';
end
end

function [traj, params, stampPath] = load_trajectory_data(trajFile, sampleCsvFile)
useCsv = false;
if exist(sampleCsvFile, 'file')
    if ~exist(trajFile, 'file')
        useCsv = true;
    else
        matInfo = dir(trajFile);
        csvInfo = dir(sampleCsvFile);
        useCsv = csvInfo.datenum > matInfo.datenum;
    end
end

if useCsv
    params = stamp_robot_params();
    sampleTable = readtable(sampleCsvFile);
    traj.time = sampleTable.time_s;
    traj.segmentIndex = sampleTable.segment_index;
    traj.Q = [sampleTable.q1_rad, sampleTable.q2_rad, ...
        sampleTable.q3_rad, sampleTable.q4_m];
    traj.Qdot = [sampleTable.q1dot, sampleTable.q2dot, ...
        sampleTable.q3dot, sampleTable.q4dot];
    traj.Qddot = [sampleTable.q1ddot, sampleTable.q2ddot, ...
        sampleTable.q3ddot, sampleTable.q4ddot];
    stampPath = [sampleTable.stamp_x_m, sampleTable.stamp_y_m, ...
        sampleTable.stamp_z_m];
else
    data = load(trajFile, 'traj', 'params', 'stampPath');
    traj = data.traj;
    params = data.params;
    if isfield(data, 'stampPath')
        stampPath = data.stampPath;
    else
        stampPath = compute_stamp_path(traj, params);
    end
end
end

function jointNames = get_robot_joint_names(configTemplate)
if isstruct(configTemplate)
    jointNames = {configTemplate.JointName};
else
    error('run_scene2_urdf_kinematics_visualization:InvalidConfigurationFormat', ...
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
            error('run_scene2_urdf_kinematics_visualization:UnknownJoint', ...
                'No model-to-URDF mapping is defined for joint: %s', jointNames{j});
    end
end
end

function write_mapping_table(resultDir, traj, urdfQ, jointNames)
mappingTable = table((1:numel(traj.time)).', traj.time, ...
    'VariableNames', {'sample', 'time_s'});
modelTable = array2table(traj.Q(:,1:4), ...
    'VariableNames', {'model_q1', 'model_q2', 'model_q3', 'model_q4'});
urdfTable = array2table(urdfQ, ...
    'VariableNames', matlab.lang.makeValidName(jointNames));
mappingTable = [mappingTable, modelTable, urdfTable];

writetable(mappingTable, fullfile(resultDir, 'scene2_urdf_visualization_mapping.csv'));
end

function validationTable = validate_urdf_mapping(robot, urdfQ, modelQ, params, resultDir)
sampleIdx = unique([1, round(size(urdfQ,1)/2), size(urdfQ,1)]);
modelStamp = zeros(numel(sampleIdx), 3);
urdfStamp = zeros(numel(sampleIdx), 3);
positionError = zeros(numel(sampleIdx), 1);

for i = 1:numel(sampleIdx)
    k = sampleIdx(i);
    kin = forward_kinematics_stamp(modelQ(k,:), params);
    T = getTransform(robot, urdfQ(k,:), 'link_5');
    modelStamp(i,:) = kin.p_stamp.';
    urdfStamp(i,:) = (T(1:3,4) + [0; 0; -params.H_stamp]).';
    positionError(i) = norm(urdfStamp(i,:) - modelStamp(i,:));
end

validationTable = table(sampleIdx(:), modelStamp(:,1), modelStamp(:,2), ...
    modelStamp(:,3), urdfStamp(:,1), urdfStamp(:,2), urdfStamp(:,3), ...
    positionError, ...
    'VariableNames', {'sample', 'model_stamp_x_m', 'model_stamp_y_m', ...
    'model_stamp_z_m', 'urdf_stamp_x_m', 'urdf_stamp_y_m', ...
    'urdf_stamp_z_m', 'position_error_m'});
writetable(validationTable, ...
    fullfile(resultDir, 'scene2_solidworks_urdf_validation.csv'));
end

function [snapshotFiles, videoFile] = render_urdf_animation(robot, urdfQ, time, ...
    stampPath, resultDir, videoBaseFile)
videoFps = 10;
frameStride = 5;
frameIdx = 1:frameStride:size(urdfQ, 1);
if frameIdx(end) ~= size(urdfQ, 1)
    frameIdx = [frameIdx, size(urdfQ, 1)];
end

fig = figure('Visible','off','Name','scene2 SolidWorks URDF kinematics', ...
    'Position',[100 100 1280 900]);
set(fig, 'Color', 'white', 'Renderer', 'opengl');
ax = axes('Parent', fig);

try
    videoFile = [videoBaseFile, '.mp4'];
    video = VideoWriter(videoFile, 'MPEG-4');
catch
    videoFile = [videoBaseFile, '.avi'];
    video = VideoWriter(videoFile, 'Motion JPEG AVI');
end
video.FrameRate = videoFps;
open(video);

snapshotIdx = unique([1, round(size(urdfQ,1)/2), size(urdfQ,1)]);
snapshotNames = {'fig_scene2_urdf_start.png', ...
    'fig_scene2_urdf_middle.png', ...
    'fig_scene2_urdf_end.png'};
snapshotFiles = cell(numel(snapshotIdx), 1);
snapshotCounter = 1;

for n = 1:numel(frameIdx)
    k = frameIdx(n);
    cla(ax);
    show(robot, urdfQ(k,:), ...
        'Parent', ax, ...
        'Frames','off', ...
        'Visuals','on', ...
        'Collisions','off', ...
        'PreservePlot',false);

    hold(ax, 'on');
    plot3(ax, stampPath(1:k,1), stampPath(1:k,2), stampPath(1:k,3), ...
        'Color', [0.85 0.20 0.18], 'LineWidth', 1.7);
    plot3(ax, stampPath(k,1), stampPath(k,2), stampPath(k,3), 'o', ...
        'MarkerSize', 6, 'MarkerFaceColor', [0.85 0.20 0.18], ...
        'MarkerEdgeColor', [0.60 0.10 0.08]);
    configure_axes(ax);
    apply_frame_lighting(ax);
    title(ax, sprintf('scene2 SolidWorks URDF kinematics  t = %.2f s', time(k)));
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
