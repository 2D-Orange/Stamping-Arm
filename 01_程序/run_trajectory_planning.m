%RUN_TRAJECTORY_PLANNING Plan the joint-space trajectory for all task points.

clear; clc; close all;

params = stamp_robot_params();

taskName = params.taskNames(:);
targets = params.taskTargets;
validate_task_config(taskName, targets);

nTask = size(targets, 1);
Q_points = zeros(nTask, 4);

for i = 1:nTask
    Q_points(i,:) = inverse_kinematics_stamp(targets(i,:), params, params.elbowMode);
end

traj = plan_joint_trajectory(Q_points, params);

nSample = numel(traj.time);
stampPath = zeros(nSample, 3);

for k = 1:nSample
    kin = forward_kinematics_stamp(traj.Q(k,:), params);
    stampPath(k,:) = kin.p_stamp.';
end

resultDir = get_result_dir();

waypointTable = table( ...
    taskName, ...
    targets(:,1), targets(:,2), targets(:,3), targets(:,4), ...
    Q_points(:,1), Q_points(:,2), Q_points(:,3), Q_points(:,4), ...
    rad2deg(Q_points(:,1)), rad2deg(Q_points(:,2)), rad2deg(Q_points(:,3)), ...
    'VariableNames', { ...
    'point', ...
    'target_x_m', 'target_y_m', 'target_z_m', 'press_flag', ...
    'q1_rad', 'q2_rad', 'q3_rad', 'q4_m', ...
    'q1_deg', 'q2_deg', 'q3_deg' ...
    });

sampleTable = table( ...
    (1:nSample).', ...
    traj.time, ...
    traj.segmentIndex, ...
    traj.Q(:,1), traj.Q(:,2), traj.Q(:,3), traj.Q(:,4), ...
    traj.Qdot(:,1), traj.Qdot(:,2), traj.Qdot(:,3), traj.Qdot(:,4), ...
    traj.Qddot(:,1), traj.Qddot(:,2), traj.Qddot(:,3), traj.Qddot(:,4), ...
    stampPath(:,1), stampPath(:,2), stampPath(:,3), ...
    'VariableNames', { ...
    'sample', 'time_s', 'segment_index', ...
    'q1_rad', 'q2_rad', 'q3_rad', 'q4_m', ...
    'q1dot', 'q2dot', 'q3dot', 'q4dot', ...
    'q1ddot', 'q2ddot', 'q3ddot', 'q4ddot', ...
    'stamp_x_m', 'stamp_y_m', 'stamp_z_m' ...
    });

writetable(waypointTable, fullfile(resultDir, 'trajectory_waypoints.csv'));
writetable(sampleTable, fullfile(resultDir, 'trajectory_samples.csv'));

save(fullfile(resultDir, 'trajectory_planning.mat'), ...
    'params', 'taskName', 'targets', 'Q_points', 'traj', 'stampPath');

plot_trajectory_results(traj, stampPath, Q_points, resultDir);

continuity = check_trajectory_continuity(traj);

disp('Joint-space waypoints:');
disp(waypointTable);

fprintf('\nTrajectory samples: %d\n', nSample);
fprintf('Total trajectory time: %.3f s\n', traj.time(end));
fprintf('Max joint waypoint position jump error: %.3e\n', continuity.maxPositionError);
fprintf('Max segment boundary velocity norm: %.3e\n', continuity.maxBoundaryVelocityNorm);
fprintf('Max segment boundary acceleration norm: %.3e\n', continuity.maxBoundaryAccelerationNorm);
fprintf('Saved trajectory results to: %s\n', resultDir);

function validate_task_config(taskName, targets)
if size(targets, 2) ~= 4
    error('run_trajectory_planning:InvalidTaskTargets', ...
        'params.taskTargets must be an N-by-4 matrix: [x y z press].');
end

if size(targets, 1) < 2
    error('run_trajectory_planning:TooFewTaskTargets', ...
        'At least two task targets are required for trajectory planning.');
end

if numel(taskName) ~= size(targets, 1)
    error('run_trajectory_planning:TaskSizeMismatch', ...
        ['params.taskNames contains %d names, but params.taskTargets ', ...
        'contains %d target rows. These counts must match.'], ...
        numel(taskName), size(targets, 1));
end
end

function resultDir = get_result_dir()
paths = stamp_project_paths();
resultDir = paths.resultDir;
if ~exist(resultDir, 'dir')
    mkdir(resultDir);
end
end

function plot_trajectory_results(traj, stampPath, Q_points, resultDir)
fig = figure('Visible','off','Name','Joint Position Trajectory');
jointLabels = {'q1 / rad', 'q2 / rad', 'q3 / rad', 'q4 / m'};
for j = 1:4
    subplot(4,1,j);
    plot(traj.time, traj.Q(:,j), 'LineWidth', 1.5);
    hold on;
    plot_segment_boundaries(traj);
    grid on;
    ylabel(jointLabels{j});
end
xlabel('time / s');
saveas(fig, fullfile(resultDir, 'fig_joint_position_trajectory.png'));
close(fig);

fig = figure('Visible','off','Name','Joint Velocity Trajectory');
for j = 1:4
    subplot(4,1,j);
    plot(traj.time, traj.Qdot(:,j), 'LineWidth', 1.5);
    hold on;
    plot_segment_boundaries(traj);
    grid on;
    ylabel(['q', num2str(j), 'dot']);
end
xlabel('time / s');
saveas(fig, fullfile(resultDir, 'fig_joint_velocity_trajectory.png'));
close(fig);

fig = figure('Visible','off','Name','Joint Acceleration Trajectory');
for j = 1:4
    subplot(4,1,j);
    plot(traj.time, traj.Qddot(:,j), 'LineWidth', 1.5);
    hold on;
    plot_segment_boundaries(traj);
    grid on;
    ylabel(['q', num2str(j), 'ddot']);
end
xlabel('time / s');
saveas(fig, fullfile(resultDir, 'fig_joint_acceleration_trajectory.png'));
close(fig);

fig = figure('Visible','off','Name','Stamp Working-Face Path');
plot3(stampPath(:,1), stampPath(:,2), stampPath(:,3), 'LineWidth', 2);
hold on;
plot3(stampPath(1,1), stampPath(1,2), stampPath(1,3), 'o', 'MarkerSize', 7);
plot3(stampPath(end,1), stampPath(end,2), stampPath(end,3), 'x', 'MarkerSize', 8);
grid on; axis equal;
xlabel('X / m');
ylabel('Y / m');
zlabel('Z / m');
title('Stamp working-face trajectory');
saveas(fig, fullfile(resultDir, 'fig_stamp_path_trajectory.png'));
close(fig);

fig = figure('Visible','off','Name','Joint Waypoints');
plot(Q_points, '-o', 'LineWidth', 1.5);
grid on;
xlabel('waypoint index');
ylabel('joint value');
legend({'q1 / rad','q2 / rad','q3 / rad','q4 / m'}, 'Location','best');
saveas(fig, fullfile(resultDir, 'fig_joint_waypoints.png'));
close(fig);
end

function plot_segment_boundaries(traj)
boundaryTimes = [0; cumsum(traj.segmentTimes)];
yl = ylim;
for i = 1:numel(boundaryTimes)
    plot([boundaryTimes(i), boundaryTimes(i)], yl, ':', 'Color', [0.55 0.55 0.55]);
end
ylim(yl);
end

function continuity = check_trajectory_continuity(traj)
boundaryTimes = cumsum(traj.segmentTimes);
maxPositionError = 0;
maxVelocityNorm = 0;
maxAccelerationNorm = 0;

for i = 1:numel(boundaryTimes)
    idx = find(abs(traj.time - boundaryTimes(i)) <= 10*eps(boundaryTimes(i) + 1), 1);
    if isempty(idx)
        [~, idx] = min(abs(traj.time - boundaryTimes(i)));
    end

    waypoint = traj.Q_waypoints(i+1,:);
    maxPositionError = max(maxPositionError, norm(traj.Q(idx,:) - waypoint));
    maxVelocityNorm = max(maxVelocityNorm, norm(traj.Qdot(idx,:)));
    maxAccelerationNorm = max(maxAccelerationNorm, norm(traj.Qddot(idx,:)));
end

continuity.maxPositionError = maxPositionError;
continuity.maxBoundaryVelocityNorm = maxVelocityNorm;
continuity.maxBoundaryAccelerationNorm = maxAccelerationNorm;
end
