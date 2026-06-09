function [q, info] = inverse_kinematics_stamp(target, params, elbowMode)
%INVERSE_KINEMATICS_STAMP Geometric inverse kinematics for stamp targets.
%
% target = [x y z press]
% x, y, z define the desired stamp working-face position in base frame.
% press can be:
%   0 or 1  -> use params.q4_min or params.q4_press
%   other   -> use the value directly as q4, in meters
%
% elbowMode:
%   'up' / 'z_up'     -> solution with the elbow convex toward +Z, default
%   'down' / 'z_down' -> solution with the lower elbow height

if nargin < 2 || isempty(params)
    params = stamp_robot_params();
end
if nargin < 3 || isempty(elbowMode)
    elbowMode = 'up';
end

target = target(:).';
if numel(target) < 3
    error('inverse_kinematics_stamp:InvalidTarget', ...
        'target must contain at least [x y z].');
end

x = target(1);
y = target(2);
z = target(3);

if numel(target) >= 4
    pressValue = target(4);
else
    pressValue = 0;
end

if abs(pressValue) < params.tolerance
    q4 = params.q4_min;
elseif abs(pressValue - 1) < params.tolerance
    q4 = params.q4_press;
else
    q4 = pressValue;
end

if q4 < params.q4_min - params.tolerance || q4 > params.q4_max + params.tolerance
    error('inverse_kinematics_stamp:Q4OutOfRange', ...
        'q4 = %.6f m is outside the allowed range [%.6f, %.6f] m.', ...
        q4, params.q4_min, params.q4_max);
end

q1 = atan2(y, x);

r_stamp = hypot(x, y);
r_wrist = r_stamp - params.axisSign * params.L_axis;
z_wrist = z + params.H_stamp + q4;
zp = z_wrist - params.L1;

if r_wrist <= params.tolerance
    error('inverse_kinematics_stamp:InvalidRadius', ...
        'The wrist radius %.6f m is not valid for this simplified geometry.', ...
        r_wrist);
end

D_raw = (r_wrist^2 + zp^2 - params.L2^2 - params.L3^2) / ...
    (2 * params.L2 * params.L3);

if abs(D_raw) > 1 + params.tolerance
    error('inverse_kinematics_stamp:UnreachableTarget', ...
        ['Target [%.6f %.6f %.6f] is outside the L2-L3 workspace. ', ...
        'Computed D = %.12f.'], x, y, z, D_raw);
end

D = min(max(D_raw, -1), 1);
rootTermAbs = sqrt(max(0, 1 - D^2));

[candidateQ, candidateElbowZ] = solve_elbow_candidates(q1, q4, D, ...
    rootTermAbs, zp, r_wrist, params);

if any(strcmpi(elbowMode, {'up', 'z_up', 'elbow_up'}))
    [~, selectedIndex] = max(candidateElbowZ);
elseif any(strcmpi(elbowMode, {'down', 'z_down', 'elbow_down'}))
    [~, selectedIndex] = min(candidateElbowZ);
else
    error('inverse_kinematics_stamp:InvalidElbowMode', ...
        'elbowMode must be ''up'', ''z_up'', ''down'', or ''z_down''.');
end

q = candidateQ(selectedIndex,:);

kin = forward_kinematics_stamp(q, params);
positionError = kin.p_stamp - [x; y; z];

info.reachable = true;
info.elbowMode = elbowMode;
info.q4 = q4;
info.r_stamp = r_stamp;
info.r_wrist = r_wrist;
info.z_wrist = z_wrist;
info.D = D_raw;
info.elbowZ = kin.p_elbow(3);
info.candidateElbowZ = candidateElbowZ;
info.forwardPosition = kin.p_stamp;
info.positionError = positionError;
info.positionErrorNorm = norm(positionError);
end

function [candidateQ, candidateElbowZ] = solve_elbow_candidates(q1, q4, D, ...
    rootTermAbs, zp, r_wrist, params)
candidateRoots = [rootTermAbs, -rootTermAbs];
candidateQ = zeros(2, 4);
candidateElbowZ = zeros(2, 1);

for i = 1:2
    q3 = atan2(candidateRoots(i), D);
    q2 = atan2(zp, r_wrist) - ...
        atan2(params.L3 * sin(q3), params.L2 + params.L3 * cos(q3));

    candidateQ(i,:) = [q1, q2, q3, q4];
    kin = forward_kinematics_stamp(candidateQ(i,:), params);
    candidateElbowZ(i) = kin.p_elbow(3);
end
end
