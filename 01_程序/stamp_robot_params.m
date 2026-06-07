function params = stamp_robot_params()
%STAMP_ROBOT_PARAMS Geometry parameters for the desktop stamping arm.
% Units: meter and radian.

params.L1 = 0.105;       % Base height.
params.L2 = 0.25;        % Upper arm length.
params.L3 = 0.30;        % Forearm length.
params.L_axis = 0.025;   % Horizontal offset from wrist joint axis to stamp slider axis.
params.H_stamp = 0.015;  % Stamp face height offset when q4 = 0.

params.q4_min = 0.0;
params.q4_max = 0.07;
params.q4_press = 0.07;

% +1 means the stamp slider axis is radially outward from the wrist.
% Use -1 if the actual assembly places it radially inward.
params.axisSign = 1;

params.safeStampZ = 0.12;
params.tolerance = 1e-10;

% Task points for inverse kinematics.
% Format of each target row: [x, y, z, press]
% z is the desired stamp working-face height.
% press = 0 means q4_min, press = 1 means q4_press.
% You may also put a direct q4 value in meters, such as 0.035.
% You can add, delete, or reorder rows here; keep taskNames the same length.
params.taskNames = {
    'home'
    'ink_above'
    'ink_press'
    'paper_above'
    'paper_press'
    };

params.taskTargets = [
    0.28,  0.00, 0.18, 0
    0.20,  0.15, 0.12, 0
    0.20,  0.15, 0.02, 1
    0.25, -0.15, 0.12, 0
    0.25, -0.15, 0.02, 1
    ];
end
