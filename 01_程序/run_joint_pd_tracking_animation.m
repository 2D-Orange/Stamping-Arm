%RUN_JOINT_PD_TRACKING_ANIMATION Render the PD tracking simulation as video.
%
% Blue shows the reference motion from the quintic trajectory. Red shows the
% actual joint response under the PD controller.

clear; clc; close all;

codeDir = fileparts(mfilename('fullpath'));
rootDir = fileparts(codeDir);
resultDir = get_output_dir(rootDir, '02_', '02_results');
videoDir = get_output_dir(rootDir, '03_', '03_video');
simFile = fullfile(resultDir, 'joint_pd_tracking_simulation.mat');

if ~exist(simFile, 'file')
    error('run_joint_pd_tracking_animation:MissingSimulation', ...
        ['PD tracking simulation file not found: %s\nRun this first:\n', ...
        '  run_joint_pd_tracking_simulation'], simFile);
end

if ~exist(videoDir, 'dir')
    mkdir(videoDir);
end

data = load(simFile, 'params', 'targets', 'taskName', 'traj', 'sim', ...
    'desiredStampPath', 'actualStampPath', 'stampError');

videoBaseFile = fullfile(videoDir, 'scene2_joint_pd_tracking');
[snapshotFiles, videoFile] = render_pd_tracking_animation(data, ...
    videoBaseFile, resultDir);

fprintf('Saved joint-space PD tracking video to: %s\n', videoFile);
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

function [snapshotFiles, videoFile] = render_pd_tracking_animation(data, ...
    videoBaseFile, resultDir)
traj = data.traj;
sim = data.sim;
params = data.params;
targets = data.targets;
taskName = data.taskName;
desiredStampPath = data.desiredStampPath;
actualStampPath = data.actualStampPath;
stampError = data.stampError;

videoFps = 15;
frameStride = 5;
frameIdx = 1:frameStride:size(sim.Q, 1);
if frameIdx(end) ~= size(sim.Q, 1)
    frameIdx = [frameIdx, size(sim.Q, 1)];
end

fig = figure('Visible', 'off', 'Name', 'scene2 joint PD tracking', ...
    'Position', [100 100 1280 900], 'Color', 'white');
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

snapshotIdx = unique([1, round(size(sim.Q,1)/2), size(sim.Q,1)]);
snapshotNames = {'fig_joint_pd_tracking_start.png', ...
    'fig_joint_pd_tracking_middle.png', ...
    'fig_joint_pd_tracking_end.png'};
snapshotFiles = cell(numel(snapshotIdx), 1);
snapshotCounter = 1;

for n = 1:numel(frameIdx)
    k = frameIdx(n);
    cla(ax);

    refKin = forward_kinematics_stamp(traj.Q(k,:), params);
    actualKin = forward_kinematics_stamp(sim.Q(k,:), params);
    draw_scene(ax, refKin, actualKin, data, k);
    drawnow;

    if snapshotCounter <= numel(snapshotIdx) && k >= snapshotIdx(snapshotCounter)
        snapshotFiles{snapshotCounter} = fullfile(resultDir, ...
            snapshotNames{snapshotCounter});
        exportgraphics(fig, snapshotFiles{snapshotCounter}, 'Resolution', 180);
        snapshotCounter = snapshotCounter + 1;
    end

    writeVideo(video, getframe(fig));
end

close(video);
close(fig);
end

function draw_scene(ax, refKin, actualKin, data, k)
traj = data.traj;
sim = data.sim;
params = data.params;
targets = data.targets;
taskName = data.taskName;
desiredStampPath = data.desiredStampPath;
actualStampPath = data.actualStampPath;
stampError = data.stampError;

hold(ax, 'on');
draw_table(ax);
draw_work_pads(ax, targets, taskName);
draw_task_points(ax, targets, taskName);

plot3(ax, desiredStampPath(1:k,1), desiredStampPath(1:k,2), ...
    desiredStampPath(1:k,3), 'Color', [0.10 0.36 0.82], ...
    'LineWidth', 1.6);
plot3(ax, actualStampPath(1:k,1), actualStampPath(1:k,2), ...
    actualStampPath(1:k,3), 'Color', [0.86 0.20 0.16], ...
    'LineWidth', 2.0);

draw_reference_model(ax, refKin);
draw_actual_model(ax, actualKin, params);

plot3(ax, desiredStampPath(k,1), desiredStampPath(k,2), ...
    desiredStampPath(k,3), 'o', 'MarkerSize', 7, ...
    'MarkerFaceColor', [0.10 0.36 0.82], ...
    'MarkerEdgeColor', [0.06 0.18 0.45]);
plot3(ax, actualStampPath(k,1), actualStampPath(k,2), ...
    actualStampPath(k,3), 'o', 'MarkerSize', 7, ...
    'MarkerFaceColor', [0.86 0.20 0.16], ...
    'MarkerEdgeColor', [0.50 0.08 0.06]);

axis(ax, 'equal');
grid(ax, 'on');
view(ax, 42, 26);
xlim(ax, [-0.10 0.75]);
ylim(ax, [-0.35 0.35]);
zlim(ax, [-0.04 0.45]);
xlabel(ax, 'X / m');
ylabel(ax, 'Y / m');
zlabel(ax, 'Z / m');
set(ax, 'Color', 'white', ...
    'XColor', 'black', 'YColor', 'black', 'ZColor', 'black', ...
    'GridColor', [0.75 0.75 0.75]);

camlight(ax, 'headlight');
lighting(ax, 'gouraud');
material(ax, 'dull');

title(ax, sprintf('scene2 joint-space PD tracking  t = %.2f s', sim.time(k)));

qRef = traj.Q(k,:);
qAct = sim.Q(k,:);
text(ax, 0.02, -0.32, 0.42, sprintf( ...
    ['blue: reference   red: actual\n', ...
    'q4 ref %.3f m   q4 actual %.3f m\n', ...
    'stamp error %.3f mm'], ...
    qRef(4), qAct(4), 1000 * stampError.norm(k)), ...
    'FontSize', 10, 'BackgroundColor', 'white', 'Margin', 3);
end

function draw_table(ax)
patch(ax, ...
    'XData', [-0.08 0.70 0.70 -0.08], ...
    'YData', [-0.30 -0.30 0.30 0.30], ...
    'ZData', [0 0 0 0], ...
    'FaceColor', [0.88 0.88 0.86], ...
    'EdgeColor', [0.50 0.50 0.50], ...
    'FaceAlpha', 0.55);
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

function draw_reference_model(ax, kin)
points = kin.points.';
plot3(ax, points(:,1), points(:,2), points(:,3), '--o', ...
    'Color', [0.10 0.36 0.82], ...
    'LineWidth', 1.3, ...
    'MarkerSize', 5, ...
    'MarkerFaceColor', [0.75 0.84 1.00], ...
    'MarkerEdgeColor', [0.10 0.36 0.82]);
end

function draw_actual_model(ax, kin, params)
pBase = kin.p_base;
pShoulder = kin.p_shoulder;
pElbow = kin.p_elbow;
pWrist = kin.p_wrist;
pAxis = kin.p_axis;
pStamp = kin.p_stamp;

armColor = [0.86 0.20 0.16];
darkColor = [0.20 0.12 0.12];

draw_cylinder_between(ax, pBase + [0;0;-0.012], pBase + [0;0;0.006], ...
    0.075, [0.34 0.34 0.36], 1.00);
draw_cylinder_between(ax, pBase, pShoulder, 0.040, [0.54 0.54 0.56], 1.00);
draw_cylinder_between(ax, pShoulder, pElbow, 0.020, armColor, 1.00);
draw_cylinder_between(ax, pElbow, pWrist, 0.017, armColor, 1.00);
draw_cylinder_between(ax, pWrist, pAxis, 0.013, armColor, 1.00);

guideBottom = pAxis - [0;0;params.H_stamp + params.q4_max + 0.025];
draw_cylinder_between(ax, pAxis, guideBottom, 0.018, [0.54 0.54 0.56], 0.22);
draw_cylinder_between(ax, pAxis, pStamp, 0.011, armColor, 1.00);
draw_cylinder_between(ax, pStamp, pStamp + [0;0;0.026], ...
    0.027, armColor, 1.00);

draw_sphere_at(ax, pShoulder, 0.030, darkColor, 1.00);
draw_sphere_at(ax, pElbow, 0.026, darkColor, 1.00);
draw_sphere_at(ax, pWrist, 0.024, darkColor, 1.00);
draw_sphere_at(ax, pAxis, 0.020, darkColor, 1.00);
draw_sphere_at(ax, pStamp, 0.014, darkColor, 1.00);
end

function draw_cylinder_between(ax, p0, p1, radius, color, alphaValue)
if nargin < 6
    alphaValue = 1.0;
end

p0 = p0(:);
p1 = p1(:);
v = p1 - p0;
lengthValue = norm(v);
if lengthValue < 1e-12
    return;
end

[xc, yc, zc] = cylinder(radius, 28);
zc = zc * lengthValue;

e3 = v / lengthValue;
tmp = [0; 0; 1];
if abs(dot(e3, tmp)) > 0.95
    tmp = [0; 1; 0];
end
e1 = cross(tmp, e3);
e1 = e1 / norm(e1);
e2 = cross(e3, e1);

x = p0(1) + e1(1) * xc + e2(1) * yc + e3(1) * zc;
y = p0(2) + e1(2) * xc + e2(2) * yc + e3(2) * zc;
z = p0(3) + e1(3) * xc + e2(3) * yc + e3(3) * zc;

surf(ax, x, y, z, ...
    'FaceColor', color, ...
    'EdgeColor', 'none', ...
    'FaceAlpha', alphaValue, ...
    'FaceLighting', 'gouraud');
patch(ax, x(1,:), y(1,:), z(1,:), color, ...
    'EdgeColor', 'none', ...
    'FaceAlpha', alphaValue, ...
    'FaceLighting', 'gouraud');
patch(ax, x(2,:), y(2,:), z(2,:), color, ...
    'EdgeColor', 'none', ...
    'FaceAlpha', alphaValue, ...
    'FaceLighting', 'gouraud');
end

function draw_sphere_at(ax, center, radius, color, alphaValue)
if nargin < 5
    alphaValue = 1.0;
end

center = center(:);
[xs, ys, zs] = sphere(18);
surf(ax, center(1) + radius * xs, ...
    center(2) + radius * ys, ...
    center(3) + radius * zs, ...
    'FaceColor', color, ...
    'EdgeColor', 'none', ...
    'FaceAlpha', alphaValue, ...
    'FaceLighting', 'gouraud');
end
