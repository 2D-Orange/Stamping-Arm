function varargout = forward_kinematics_stamp(q, params)
%FORWARD_KINEMATICS_STAMP Non-standard-DH forward kinematics for the stamp arm.
%
% q = [q1 q2 q3 q4]
% q1: base yaw, rad
% q2: upper arm pitch in the radial vertical plane, rad
% q3: forearm pitch relative to upper arm, rad
% q4: downward stamp extension, m
%
% The stamp face is returned as p_stamp. This implementation uses a
% non-standard DH-style homogeneous transform chain:
%   T01 = Tz(L1) * Rz(q1)
%   T12 = Ry(-q2) * Tx(L2)
%   T23 = Ry(-q3) * Tx(L3)
%   T3a = Ry(q2+q3) * Tx(axisSign*L_axis)
%   Ta4 = Tz(-(H_stamp+q4))
% The compensation frame a keeps the stamp axis vertical.

if nargin < 2 || isempty(params)
    params = stamp_robot_params();
end

q = q(:);
if numel(q) ~= 4
    error('forward_kinematics_stamp:InvalidQ', ...
        'q must contain exactly four elements: [q1 q2 q3 q4].');
end

q1 = q(1);
q2 = q(2);
q3 = q(3);
q4 = q(4);

if q4 < params.q4_min - params.tolerance || q4 > params.q4_max + params.tolerance
    error('forward_kinematics_stamp:Q4OutOfRange', ...
        'q4 = %.6f m is outside the allowed range [%.6f, %.6f] m.', ...
        q4, params.q4_min, params.q4_max);
end

T_0_1 = trans_z(params.L1) * rot_z(q1);
T_1_2 = rot_y(-q2) * trans_x(params.L2);
T_2_3 = rot_y(-q3) * trans_x(params.L3);
T_3_a = rot_y(q2 + q3) * trans_x(params.axisSign * params.L_axis);
T_a_4 = trans_z(-(params.H_stamp + q4));

T_0_2 = T_0_1 * T_1_2;
T_0_3 = T_0_2 * T_2_3;
T_0_a = T_0_3 * T_3_a;
T_0_4 = T_0_a * T_a_4;

p_base = [0; 0; 0];
p_shoulder = T_0_1(1:3, 4);
p_elbow = T_0_2(1:3, 4);
p_wrist = T_0_3(1:3, 4);
p_axis = T_0_a(1:3, 4);
p_stamp = T_0_4(1:3, 4);

kin.q = q;
kin.p_base = p_base;
kin.p_shoulder = p_shoulder;
kin.p_elbow = p_elbow;
kin.p_wrist = p_wrist;
kin.p_axis = p_axis;
kin.p_stamp = p_stamp;
kin.points = [p_base, p_shoulder, p_elbow, p_wrist, p_axis, p_stamp];
kin.T_base = eye(4);
kin.T_shoulder = T_0_1;
kin.T_elbow = T_0_2;
kin.T_wrist = T_0_3;
kin.T_axis = T_0_a;
kin.T_stamp = T_0_4;
kin.T_0_1 = T_0_1;
kin.T_1_2 = T_1_2;
kin.T_2_3 = T_2_3;
kin.T_3_a = T_3_a;
kin.T_a_4 = T_a_4;
kin.T_0_2 = T_0_2;
kin.T_0_3 = T_0_3;
kin.T_0_a = T_0_a;
kin.T_0_4 = T_0_4;

if nargout <= 1
    varargout{1} = kin;
else
    varargout = {p_base, p_shoulder, p_elbow, p_wrist, p_axis, p_stamp, kin};
end
end

function T = trans_x(distance)
T = eye(4);
T(1, 4) = distance;
end

function T = trans_z(distance)
T = eye(4);
T(3, 4) = distance;
end

function T = rot_y(theta)
c = cos(theta);
s = sin(theta);
T = eye(4);
T(1:3, 1:3) = [
    c, 0, s
    0, 1, 0
   -s, 0, c
    ];
end

function T = rot_z(theta)
c = cos(theta);
s = sin(theta);
T = eye(4);
T(1:3, 1:3) = [
    c, -s, 0
    s,  c, 0
    0,  0, 1
    ];
end
