%RUN_JOINT_PD_TRACKING_SIMULATION Simulate joint-space PD trajectory tracking.
%
% The desired trajectory is rebuilt from stamp_robot_params().taskTargets each
% time this script runs. To change the stamping work points, edit taskNames and
% taskTargets in stamp_robot_params.m, then run this script again.

clear; clc; close all;

params = stamp_robot_params();
taskName = params.taskNames(:);
targets = params.taskTargets;
validate_task_config(taskName, targets);

controller = get_pd_controller_params(params);

nTask = size(targets, 1);
Q_points = zeros(nTask, 4);

for i = 1:nTask
    Q_points(i,:) = inverse_kinematics_stamp(targets(i,:), params, params.elbowMode);
end

traj = plan_joint_trajectory(Q_points, params);
traj = append_settling_hold(traj, params);
sim = simulate_joint_pd_tracking(traj, params, controller);

[desiredStampPath, actualStampPath, stampError] = compute_stamp_paths( ...
    traj.Q, sim.Q, params);

resultDir = get_result_dir();
sampleTable = build_sample_table(traj, sim, desiredStampPath, ...
    actualStampPath, stampError);
summaryTable = build_summary_table(sim);

writetable(sampleTable, fullfile(resultDir, 'joint_pd_tracking_samples.csv'));
writetable(summaryTable, fullfile(resultDir, 'joint_pd_tracking_summary.csv'));

save(fullfile(resultDir, 'joint_pd_tracking_simulation.mat'), ...
    'params', 'taskName', 'targets', 'Q_points', 'traj', 'controller', ...
    'sim', 'desiredStampPath', 'actualStampPath', 'stampError');

plot_pd_tracking_results(traj, sim, desiredStampPath, actualStampPath, ...
    stampError, resultDir);

disp('Joint-space PD tracking summary:');
disp(summaryTable);
fprintf('\nTotal trajectory time: %.3f s\n', traj.time(end));
fprintf('Final settling hold time: %.3f s\n', get_scalar_param(params, ...
    'pdSettlingTime', 0));
fprintf('PD gains Kp: [%s]\n', num2str(controller.Kp, ' %.3g'));
fprintf('PD gains Kd: [%s]\n', num2str(controller.Kd, ' %.3g'));
fprintf('Max stamp position error: %.6e m\n', max(stampError.norm));
fprintf('RMS stamp position error: %.6e m\n', rms_local(stampError.norm));
fprintf('Saved PD tracking results to: %s\n', resultDir);

function validate_task_config(taskName, targets)
if size(targets, 2) ~= 4
    error('run_joint_pd_tracking_simulation:InvalidTaskTargets', ...
        'params.taskTargets must be an N-by-4 matrix: [x y z press].');
end

if size(targets, 1) < 2
    error('run_joint_pd_tracking_simulation:TooFewTaskTargets', ...
        'At least two task targets are required for PD tracking simulation.');
end

if numel(taskName) ~= size(targets, 1)
    error('run_joint_pd_tracking_simulation:TaskSizeMismatch', ...
        ['params.taskNames contains %d names, but params.taskTargets ', ...
        'contains %d target rows. These counts must match.'], ...
        numel(taskName), size(targets, 1));
end
end

function traj = append_settling_hold(traj, params)
settlingTime = get_scalar_param(params, 'pdSettlingTime', 0);

if settlingTime < 0
    error('run_joint_pd_tracking_simulation:InvalidSettlingTime', ...
        'pdSettlingTime must be nonnegative.');
end

if settlingTime == 0
    return;
end

if isfield(traj, 'dt') && ~isempty(traj.dt)
    dt = traj.dt;
else
    dt = params.trajDt;
end

holdTime = dt:dt:settlingTime;
if isempty(holdTime) || abs(holdTime(end) - settlingTime) > eps(settlingTime + 1)
    holdTime = [holdTime, settlingTime];
end

nHold = numel(holdTime);
qFinal = traj.Q(end,:);
nJoint = size(traj.Q, 2);

traj.Q = [traj.Q; repmat(qFinal, nHold, 1)];
traj.Qdot = [traj.Qdot; zeros(nHold, nJoint)];
traj.Qddot = [traj.Qddot; zeros(nHold, nJoint)];
traj.time = [traj.time; traj.time(end) + holdTime(:)];
traj.segmentIndex = [traj.segmentIndex; ...
    (max(traj.segmentIndex) + 1) * ones(nHold, 1)];
traj.segmentTimes = [traj.segmentTimes(:); settlingTime];
traj.Q_waypoints = [traj.Q_waypoints; qFinal];
end

function controller = get_pd_controller_params(params)
wn = get_row_param(params, 'pdNaturalFrequency', [10, 10, 10, 14]);
zeta = get_row_param(params, 'pdDampingRatio', [1, 1, 1, 1]);

if any(wn <= 0)
    error('run_joint_pd_tracking_simulation:InvalidNaturalFrequency', ...
        'All pdNaturalFrequency values must be positive.');
end

if any(zeta <= 0)
    error('run_joint_pd_tracking_simulation:InvalidDampingRatio', ...
        'All pdDampingRatio values must be positive.');
end

controller.Kp = wn .^ 2;
controller.Kd = 2 .* zeta .* wn;
controller.naturalFrequency = wn;
controller.dampingRatio = zeta;
controller.initialPositionError = get_row_param(params, ...
    'pdInitialPositionError', [0, 0, 0, 0]);
controller.initialVelocityError = get_row_param(params, ...
    'pdInitialVelocityError', [0, 0, 0, 0]);
controller.maxAcceleration = get_row_param(params, ...
    'pdMaxJointAcceleration', [30, 30, 30, 3]);
end

function value = get_scalar_param(params, fieldName, defaultValue)
if isfield(params, fieldName)
    value = params.(fieldName);
else
    value = defaultValue;
end

if ~isscalar(value)
    error('run_joint_pd_tracking_simulation:InvalidScalarParam', ...
        '%s must be a scalar value.', fieldName);
end
end

function value = get_row_param(params, fieldName, defaultValue)
if isfield(params, fieldName)
    value = params.(fieldName);
else
    value = defaultValue;
end

value = value(:).';
if numel(value) ~= 4
    error('run_joint_pd_tracking_simulation:InvalidControllerParam', ...
        '%s must contain exactly four values.', fieldName);
end
end

function sim = simulate_joint_pd_tracking(traj, params, controller)
nSample = numel(traj.time);
nJoint = size(traj.Q, 2);

if nJoint ~= 4
    error('run_joint_pd_tracking_simulation:InvalidTrajectory', ...
        'The trajectory must contain four joint coordinates.');
end

Q = zeros(nSample, nJoint);
Qdot = zeros(nSample, nJoint);
QddotCommand = zeros(nSample, nJoint);
errorQ = zeros(nSample, nJoint);
errorQdot = zeros(nSample, nJoint);

Q(1,:) = traj.Q(1,:) + controller.initialPositionError;
Qdot(1,:) = traj.Qdot(1,:) + controller.initialVelocityError;
[Q(1,:), Qdot(1,:)] = apply_joint_limits(Q(1,:), Qdot(1,:), params);

for k = 1:nSample
    errorQ(k,:) = traj.Q(k,:) - Q(k,:);
    errorQdot(k,:) = traj.Qdot(k,:) - Qdot(k,:);
    QddotCommand(k,:) = controller.Kp .* errorQ(k,:) + ...
        controller.Kd .* errorQdot(k,:);
    QddotCommand(k,:) = clamp_row(QddotCommand(k,:), ...
        -controller.maxAcceleration, controller.maxAcceleration);

    if k < nSample
        dt = traj.time(k+1) - traj.time(k);
        Qdot(k+1,:) = Qdot(k,:) + QddotCommand(k,:) * dt;
        Q(k+1,:) = Q(k,:) + Qdot(k+1,:) * dt;
        [Q(k+1,:), Qdot(k+1,:)] = apply_joint_limits( ...
            Q(k+1,:), Qdot(k+1,:), params);
    end
end

sim.time = traj.time;
sim.Q = Q;
sim.Qdot = Qdot;
sim.QddotCommand = QddotCommand;
sim.error = errorQ;
sim.velocityError = errorQdot;
sim.Kp = controller.Kp;
sim.Kd = controller.Kd;
end

function value = clamp_row(value, lower, upper)
value = min(max(value, lower), upper);
end

function [q, qdot] = apply_joint_limits(q, qdot, params)
if q(4) < params.q4_min
    q(4) = params.q4_min;
    qdot(4) = max(qdot(4), 0);
elseif q(4) > params.q4_max
    q(4) = params.q4_max;
    qdot(4) = min(qdot(4), 0);
end
end

function [desiredStampPath, actualStampPath, stampError] = compute_stamp_paths( ...
    desiredQ, actualQ, params)
nSample = size(desiredQ, 1);
desiredStampPath = zeros(nSample, 3);
actualStampPath = zeros(nSample, 3);

for k = 1:nSample
    desiredKin = forward_kinematics_stamp(desiredQ(k,:), params);
    actualKin = forward_kinematics_stamp(actualQ(k,:), params);
    desiredStampPath(k,:) = desiredKin.p_stamp.';
    actualStampPath(k,:) = actualKin.p_stamp.';
end

stampError.xyz = desiredStampPath - actualStampPath;
stampError.norm = sqrt(sum(stampError.xyz .^ 2, 2));
end

function sampleTable = build_sample_table(traj, sim, desiredStampPath, ...
    actualStampPath, stampError)
nSample = numel(traj.time);
sampleTable = table( ...
    (1:nSample).', traj.time, traj.segmentIndex, ...
    traj.Q(:,1), traj.Q(:,2), traj.Q(:,3), traj.Q(:,4), ...
    sim.Q(:,1), sim.Q(:,2), sim.Q(:,3), sim.Q(:,4), ...
    sim.error(:,1), sim.error(:,2), sim.error(:,3), sim.error(:,4), ...
    traj.Qdot(:,1), traj.Qdot(:,2), traj.Qdot(:,3), traj.Qdot(:,4), ...
    sim.Qdot(:,1), sim.Qdot(:,2), sim.Qdot(:,3), sim.Qdot(:,4), ...
    sim.velocityError(:,1), sim.velocityError(:,2), ...
    sim.velocityError(:,3), sim.velocityError(:,4), ...
    sim.QddotCommand(:,1), sim.QddotCommand(:,2), ...
    sim.QddotCommand(:,3), sim.QddotCommand(:,4), ...
    desiredStampPath(:,1), desiredStampPath(:,2), desiredStampPath(:,3), ...
    actualStampPath(:,1), actualStampPath(:,2), actualStampPath(:,3), ...
    stampError.xyz(:,1), stampError.xyz(:,2), stampError.xyz(:,3), ...
    stampError.norm, ...
    'VariableNames', { ...
    'sample', 'time_s', 'segment_index', ...
    'q1_ref_rad', 'q2_ref_rad', 'q3_ref_rad', 'q4_ref_m', ...
    'q1_actual_rad', 'q2_actual_rad', 'q3_actual_rad', 'q4_actual_m', ...
    'q1_error_rad', 'q2_error_rad', 'q3_error_rad', 'q4_error_m', ...
    'q1dot_ref', 'q2dot_ref', 'q3dot_ref', 'q4dot_ref', ...
    'q1dot_actual', 'q2dot_actual', 'q3dot_actual', 'q4dot_actual', ...
    'q1dot_error', 'q2dot_error', 'q3dot_error', 'q4dot_error', ...
    'q1ddot_cmd', 'q2ddot_cmd', 'q3ddot_cmd', 'q4ddot_cmd', ...
    'stamp_ref_x_m', 'stamp_ref_y_m', 'stamp_ref_z_m', ...
    'stamp_actual_x_m', 'stamp_actual_y_m', 'stamp_actual_z_m', ...
    'stamp_error_x_m', 'stamp_error_y_m', 'stamp_error_z_m', ...
    'stamp_error_norm_m' ...
    });
end

function summaryTable = build_summary_table(sim)
joint = {'q1'; 'q2'; 'q3'; 'q4'};
nativeUnit = {'rad'; 'rad'; 'rad'; 'm'};
reportUnit = {'deg'; 'deg'; 'deg'; 'mm'};

maxAbsError = max(abs(sim.error), [], 1).';
rmsError = sqrt(mean(sim.error .^ 2, 1)).';
finalAbsError = abs(sim.error(end,:)).';
maxAbsVelocityError = max(abs(sim.velocityError), [], 1).';
maxAbsAccelCommand = max(abs(sim.QddotCommand), [], 1).';

maxAbsErrorReport = [rad2deg(maxAbsError(1:3)); maxAbsError(4) * 1000];
rmsErrorReport = [rad2deg(rmsError(1:3)); rmsError(4) * 1000];
finalAbsErrorReport = [rad2deg(finalAbsError(1:3)); finalAbsError(4) * 1000];

summaryTable = table(joint, nativeUnit, maxAbsError, rmsError, ...
    finalAbsError, maxAbsVelocityError, maxAbsAccelCommand, reportUnit, ...
    maxAbsErrorReport, rmsErrorReport, finalAbsErrorReport, ...
    'VariableNames', { ...
    'joint', 'native_unit', 'max_abs_error', 'rms_error', ...
    'final_abs_error', 'max_abs_velocity_error', 'max_abs_accel_command', ...
    'report_unit', 'max_abs_error_report', 'rms_error_report', ...
    'final_abs_error_report' ...
    });
end

function plot_pd_tracking_results(traj, sim, desiredStampPath, ...
    actualStampPath, stampError, resultDir)
fig = figure('Visible','off','Name','Joint PD Tracking Position');
jointLabels = {'q1 / rad', 'q2 / rad', 'q3 / rad', 'q4 / m'};
for j = 1:4
    subplot(4,1,j);
    plot(traj.time, traj.Q(:,j), 'LineWidth', 1.5);
    hold on;
    plot(sim.time, sim.Q(:,j), '--', 'LineWidth', 1.2);
    grid on;
    ylabel(jointLabels{j});
    if j == 1
        legend({'reference','actual'}, 'Location','best');
    end
end
xlabel('time / s');
saveas(fig, fullfile(resultDir, 'fig_joint_pd_tracking_position.png'));
close(fig);

fig = figure('Visible','off','Name','Joint PD Tracking Error');
for j = 1:4
    subplot(4,1,j);
    plot(sim.time, sim.error(:,j), 'LineWidth', 1.5);
    grid on;
    ylabel(['e_', num2str(j)]);
end
xlabel('time / s');
saveas(fig, fullfile(resultDir, 'fig_joint_pd_tracking_error.png'));
close(fig);

fig = figure('Visible','off','Name','Stamp PD Tracking Path');
plot3(desiredStampPath(:,1), desiredStampPath(:,2), ...
    desiredStampPath(:,3), 'LineWidth', 2);
hold on;
plot3(actualStampPath(:,1), actualStampPath(:,2), ...
    actualStampPath(:,3), '--', 'LineWidth', 1.4);
plot3(desiredStampPath(1,1), desiredStampPath(1,2), ...
    desiredStampPath(1,3), 'o', 'MarkerSize', 7);
plot3(desiredStampPath(end,1), desiredStampPath(end,2), ...
    desiredStampPath(end,3), 'x', 'MarkerSize', 8);
grid on; axis equal;
xlabel('X / m');
ylabel('Y / m');
zlabel('Z / m');
legend({'reference','actual','start','end'}, 'Location','best');
title('Stamp working-face PD tracking path');
saveas(fig, fullfile(resultDir, 'fig_stamp_pd_tracking_path.png'));
close(fig);

fig = figure('Visible','off','Name','Stamp PD Tracking Error');
plot(sim.time, stampError.xyz(:,1), 'LineWidth', 1.2);
hold on;
plot(sim.time, stampError.xyz(:,2), 'LineWidth', 1.2);
plot(sim.time, stampError.xyz(:,3), 'LineWidth', 1.2);
plot(sim.time, stampError.norm, 'k', 'LineWidth', 1.5);
grid on;
xlabel('time / s');
ylabel('position error / m');
legend({'x','y','z','norm'}, 'Location','best');
saveas(fig, fullfile(resultDir, 'fig_stamp_pd_tracking_error.png'));
close(fig);
end

function resultDir = get_result_dir()
paths = stamp_project_paths();
resultDir = paths.resultDir;
if ~exist(resultDir, 'dir')
    mkdir(resultDir);
end
end

function value = rms_local(value)
value = sqrt(mean(value .^ 2));
end
