%RUN_KINEMATICS_SOLUTION Solve and verify FK/IK for the stamp arm.

clear; clc;

params = stamp_robot_params();

taskName = params.taskNames(:);
targets = params.taskTargets;

if size(targets, 2) ~= 4
    error('run_kinematics_solution:InvalidTaskTargets', ...
        'params.taskTargets must be an N-by-4 matrix: [x y z press].');
end

if isempty(targets)
    error('run_kinematics_solution:EmptyTaskTargets', ...
        'params.taskTargets must contain at least one task point.');
end

if numel(taskName) ~= size(targets, 1)
    error('run_kinematics_solution:TaskSizeMismatch', ...
        ['params.taskNames contains %d names, but params.taskTargets ', ...
        'contains %d target rows. These counts must match.'], ...
        numel(taskName), size(targets, 1));
end

nTask = size(targets, 1);
Q = zeros(nTask, 4);
Q_deg = zeros(nTask, 3);
fkStamp = zeros(nTask, 3);
posErr = zeros(nTask, 3);
posErrNorm = zeros(nTask, 1);
D_value = zeros(nTask, 1);
rWrist = zeros(nTask, 1);
zWrist = zeros(nTask, 1);
elbowZ = zeros(nTask, 1);

fprintf('Stamp arm geometry:\n');
fprintf('  L1 = %.3f m, L2 = %.3f m, L3 = %.3f m\n', ...
    params.L1, params.L2, params.L3);
fprintf('  L_axis = %.3f m, H_stamp = %.3f m, q4 range = [%.3f, %.3f] m\n', ...
    params.L_axis, params.H_stamp, params.q4_min, params.q4_max);
fprintf('  inverse kinematics elbow mode = %s\n\n', params.elbowMode);

for i = 1:nTask
    [Q(i,:), info] = inverse_kinematics_stamp(targets(i,:), params, params.elbowMode);
    kin = forward_kinematics_stamp(Q(i,:), params);

    Q_deg(i,:) = rad2deg(Q(i,1:3));
    fkStamp(i,:) = kin.p_stamp.';
    posErr(i,:) = info.positionError.';
    posErrNorm(i) = info.positionErrorNorm;
    D_value(i) = info.D;
    rWrist(i) = info.r_wrist;
    zWrist(i) = info.z_wrist;
    elbowZ(i) = info.elbowZ;
end

results = table( ...
    taskName, ...
    targets(:,1), targets(:,2), targets(:,3), targets(:,4), ...
    Q(:,1), Q(:,2), Q(:,3), Q(:,4), ...
    Q_deg(:,1), Q_deg(:,2), Q_deg(:,3), ...
    fkStamp(:,1), fkStamp(:,2), fkStamp(:,3), ...
    posErrNorm, rWrist, zWrist, elbowZ, D_value, ...
    'VariableNames', { ...
    'point', ...
    'target_x_m', 'target_y_m', 'target_z_m', 'press_flag', ...
    'q1_rad', 'q2_rad', 'q3_rad', 'q4_m', ...
    'q1_deg', 'q2_deg', 'q3_deg', ...
    'fk_x_m', 'fk_y_m', 'fk_z_m', ...
    'position_error_m', 'r_wrist_m', 'z_wrist_m', 'elbow_z_m', 'D' ...
    });

disp(results);

maxError = max(posErrNorm);
fprintf('\nMax FK/IK stamp-face position error: %.3e m\n', maxError);

if maxError > 1e-9
    error('run_kinematics_solution:VerificationFailed', ...
        'Forward/inverse kinematics verification failed.');
end

codeDir = fileparts(mfilename('fullpath'));
rootDir = fileparts(codeDir);
resultDir = get_output_dir(rootDir, '02_', '02_results');
if ~exist(resultDir, 'dir')
    mkdir(resultDir);
end

outCsv = fullfile(resultDir, 'kinematics_solution.csv');
writetable(results, outCsv);
fprintf('Saved result table to: %s\n', outCsv);

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
