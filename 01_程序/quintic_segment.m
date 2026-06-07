function [q, qdot, qddot, t] = quintic_segment(q0, qf, T, dt)
%QUINTIC_SEGMENT Generate a fifth-order polynomial joint trajectory segment.
%
% Boundary conditions:
%   q(0) = q0, q(T) = qf
%   qdot(0) = qdot(T) = 0
%   qddot(0) = qddot(T) = 0

if T <= 0
    error('quintic_segment:InvalidDuration', ...
        'Segment duration T must be positive.');
end

if dt <= 0
    error('quintic_segment:InvalidStep', ...
        'Time step dt must be positive.');
end

q0 = q0(:).';
qf = qf(:).';

if numel(q0) ~= numel(qf)
    error('quintic_segment:SizeMismatch', ...
        'q0 and qf must have the same number of elements.');
end

t = 0:dt:T;
if abs(t(end) - T) > eps(T)
    t = [t, T];
end

tau = t / T;
s = 10*tau.^3 - 15*tau.^4 + 6*tau.^5;
sdot = (30*tau.^2 - 60*tau.^3 + 30*tau.^4) / T;
sddot = (60*tau - 180*tau.^2 + 120*tau.^3) / T^2;

dq = qf - q0;

q = q0 + s(:) * dq;
qdot = sdot(:) * dq;
qddot = sddot(:) * dq;
end
