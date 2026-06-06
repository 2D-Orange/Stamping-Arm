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
end
