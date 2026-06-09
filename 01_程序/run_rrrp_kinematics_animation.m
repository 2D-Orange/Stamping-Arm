%RUN_RRRP_KINEMATICS_ANIMATION Render primitive no-CAD scene2 RRRP kinematics.

clear; clc; close all;

codeDir = fileparts(mfilename('fullpath'));
rootDir = fileparts(codeDir);
resultDir = get_output_dir(rootDir, '02_', '02_results');
videoDir = get_output_dir(rootDir, '03_', '03_video');
trajFile = fullfile(resultDir, 'trajectory_planning.mat');

if ~exist(trajFile, 'file')
    error('run_rrrp_kinematics_animation:MissingTrajectory', ...
        ['Trajectory file not found: %s\nRun this first:\n', ...
        '  run_trajectory_planning'], trajFile);
end

if ~exist(videoDir, 'dir')
    mkdir(videoDir);
end

data = load(trajFile, 'traj', 'params', 'taskName', 'targets');
traj = data.traj;
params = data.params;
targets = data.targets;
taskName = data.taskName;

videoFile = fullfile(videoDir, 'scene2_rrrp_kinematics.avi');
snapshotFiles = render_animation(traj, params, targets, taskName, videoFile, resultDir);

fprintf('Saved scene2 primitive RRRP animation to: %s\n', videoFile);
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

function snapshotFiles = render_animation(traj, params, targets, taskName, videoFile, resultDir)
videoFps = 15;
frameStride = 5;
frameIdx = 1:frameStride:size(traj.Q, 1);
if frameIdx(end) ~= size(traj.Q, 1)
    frameIdx = [frameIdx, size(traj.Q, 1)];
end

fig = figure('Visible', 'off', 'Name', 'scene2 primitive RRRP kinematics', ...
    'Position', [100 100 1280 900], 'Color', 'white');
ax = axes('Parent', fig);

video = VideoWriter(videoFile, 'Motion JPEG AVI');
video.FrameRate = videoFps;
open(video);

snapshotIdx = unique([1, round(size(traj.Q,1)/2), size(traj.Q,1)]);
snapshotNames = {'fig_scene2_rrrp_start.png', ...
    'fig_scene2_rrrp_middle.png', ...
    'fig_scene2_rrrp_end.png'};
snapshotFiles = cell(numel(snapshotIdx), 1);
snapshotCounter = 1;

stampPath = zeros(size(traj.Q,1), 3);
for k = 1:size(traj.Q,1)
    kin = forward_kinematics_stamp(traj.Q(k,:), params);
    stampPath(k,:) = kin.p_stamp.';
end

for n = 1:numel(frameIdx)
    k = frameIdx(n);
    cla(ax);
    kin = forward_kinematics_stamp(traj.Q(k,:), params);
    draw_scene(ax, kin, traj, stampPath, targets, taskName, params, k);
    title(ax, sprintf('scene2 primitive RRRP kinematics  t = %.2f s', traj.time(k)));
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

function draw_scene(ax, kin, traj, stampPath, targets, taskName, params, k)
hold(ax, 'on');

draw_table(ax);
draw_work_pads(ax, targets, taskName);
draw_task_points(ax, targets, taskName);

plot3(ax, stampPath(1:k,1), stampPath(1:k,2), stampPath(1:k,3), ...
    'Color', [0.85 0.20 0.18], 'LineWidth', 1.8);

draw_rrrp_model(ax, kin, params);
draw_joint_axes(ax, kin);
draw_joint_labels(ax, kin.points.');

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

q = traj.Q(k,:);
text(ax, 0.02, -0.32, 0.42, sprintf( ...
    'q1 %.1f deg   q2 %.1f deg   q3 %.1f deg   q4 %.3f m', ...
    rad2deg(q(1)), rad2deg(q(2)), rad2deg(q(3)), q(4)), ...
    'FontSize', 10, 'BackgroundColor', 'white', 'Margin', 2);
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

function draw_rrrp_model(ax, kin, params)
pBase = kin.p_base;
pShoulder = kin.p_shoulder;
pElbow = kin.p_elbow;
pWrist = kin.p_wrist;
pAxis = kin.p_axis;
pStamp = kin.p_stamp;

armGray = [0.42 0.42 0.42];
darkGray = [0.18 0.18 0.20];

draw_cylinder_between(ax, pBase + [0;0;-0.012], pBase + [0;0;0.006], ...
    0.075, [0.34 0.34 0.36], 1.00);
draw_cylinder_between(ax, pBase, pShoulder, 0.042, armGray, 1.00);

draw_cylinder_between(ax, pShoulder, pElbow, 0.020, armGray, 1.00);
draw_cylinder_between(ax, pElbow, pWrist, 0.017, armGray, 1.00);
draw_cylinder_between(ax, pWrist, pAxis, 0.013, armGray, 1.00);

guideBottom = pAxis - [0;0;params.H_stamp + params.q4_max + 0.025];
draw_cylinder_between(ax, pAxis, guideBottom, 0.018, armGray, 0.22);
draw_cylinder_between(ax, pAxis, pStamp, 0.011, armGray, 1.00);

draw_cylinder_between(ax, pStamp, pStamp + [0;0;0.026], ...
    0.027, armGray, 1.00);

draw_sphere_at(ax, pShoulder, 0.030, darkGray, 1.00);
draw_sphere_at(ax, pElbow, 0.026, darkGray, 1.00);
draw_sphere_at(ax, pWrist, 0.024, darkGray, 1.00);
draw_sphere_at(ax, pAxis, 0.020, darkGray, 1.00);
draw_sphere_at(ax, pStamp, 0.014, darkGray, 1.00);
end

function draw_joint_axes(ax, kin)
axisLength = 0.065;
q1 = kin.q(1);
baseAxis = [0; 0; 1];
armAxis = [-sin(q1); cos(q1); 0];
sliderAxis = [0; 0; -1];

draw_axis_arrow(ax, kin.p_base, baseAxis, axisLength, [0.78 0.16 0.16], 'q1');
draw_axis_arrow(ax, kin.p_shoulder, armAxis, axisLength, [0.78 0.48 0.05], 'q2');
draw_axis_arrow(ax, kin.p_elbow, armAxis, axisLength, [0.78 0.48 0.05], 'q3');
draw_axis_arrow(ax, kin.p_axis, sliderAxis, axisLength, [0.34 0.16 0.70], 'q4');
end

function draw_axis_arrow(ax, origin, direction, axisLength, color, labelText)
origin = origin(:);
direction = direction(:) / norm(direction);
v = direction * axisLength;
quiver3(ax, origin(1), origin(2), origin(3), v(1), v(2), v(3), ...
    0, 'Color', color, 'LineWidth', 1.5, 'MaxHeadSize', 0.7);
tip = origin + v;
text(ax, tip(1), tip(2), tip(3), [' ', labelText], ...
    'FontSize', 8, 'Color', color, 'FontWeight', 'bold');
end

function draw_joint_labels(ax, points)
names = {'base', 'shoulder', 'elbow', 'wrist', 'axis', 'stamp'};
for i = 1:numel(names)
    text(ax, points(i,1), points(i,2), points(i,3), ...
        ['  ', names{i}], 'FontSize', 8, 'Color', [0.12 0.12 0.12]);
end
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
