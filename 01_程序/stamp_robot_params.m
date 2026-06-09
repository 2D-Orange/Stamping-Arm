function params = stamp_robot_params()
%STAMP_ROBOT_PARAMS Geometry parameters for scene2 RRRP stamping arm.
% Units: meter and radian.

params.L1 = 0.15;        % Base height.
params.L2 = 0.25;        % Upper arm length.
params.L3 = 0.30;        % Forearm length.
params.L_axis = 0.11;    % Horizontal offset from wrist joint axis to stamp axis.
params.H_stamp = 0.01;   % Stamp face height offset when q4 = 0.

params.q4_min = 0.0;
params.q4_max = 0.12;
params.q4_press = 0.06;

% +1 means the stamp slider axis is radially outward from the wrist.
% Use -1 if the actual scene2 assembly places it radially inward.
params.axisSign = 1;

params.safeStampZ = 0.08;
params.tolerance = 1e-10;
params.elbowMode = 'elbow_up';

% Trajectory planning settings.
% trajSegmentTime can be a scalar or a vector with one value per segment.
params.trajSegmentTime = 2.0;
params.trajDt = 0.02;
params.pdSettlingTime = 1.0;

% Joint-space PD tracking simulation settings.
% Kp = pdNaturalFrequency.^2, Kd = 2*pdDampingRatio.*pdNaturalFrequency.
% q1-q3 use radian units; q4 uses meter units.
params.pdNaturalFrequency = [10, 10, 10, 14];
params.pdDampingRatio = [1, 1, 1, 1];
params.pdInitialPositionError = [0, 0, 0, 0];
params.pdInitialVelocityError = [0, 0, 0, 0];
params.pdMaxJointAcceleration = [30, 30, 30, 3];

% Task points for inverse kinematics.
% Format of each target row: [x, y, z, press]
% z is the desired stamp working-face height.
% press = 0 means q4_min, press = 1 means q4_press.
% You may also put a direct q4 value in meters, such as 0.060.
% You can add, delete, or reorder rows here; keep taskNames the same length.
params.taskNames = {
    'home'
    'ink_above'
    'ink_press'
    'ink_above'
    'paper_above_1'
    'paper_press_1'
    'paper_above_1'
    'paper_above_2'
    'paper_press_2'
    'paper_above_2'
    'home'
    };

params.taskTargets = [
    0.38,  0.00, 0.28, 0
    0.38,  0.18, 0.08, 0
    0.38,  0.18, 0.02, 0.06 % 0.12 1
    0.38,  0.18, 0.08, 0
    0.28, -0.02, 0.06, 0
    0.28, -0.02, 0.00, 0.06
    0.28, -0.02, 0.06, 0
    0.42, -0.18, 0.06, 0
    0.42, -0.18, 0.00, 0.06
    0.42, -0.18, 0.06, 0
    0.38,  0.00, 0.28, 0
    ];
end
