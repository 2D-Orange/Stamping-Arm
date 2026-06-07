function traj = plan_joint_trajectory(Q_waypoints, params, segmentTimes)
%PLAN_JOINT_TRAJECTORY Build a full joint trajectory through waypoints.
%
% Q_waypoints is an N-by-4 matrix. The generated trajectory uses one
% quintic segment between each adjacent waypoint.

if nargin < 2 || isempty(params)
    params = stamp_robot_params();
end

if nargin < 3 || isempty(segmentTimes)
    segmentTimes = params.trajSegmentTime;
end

if size(Q_waypoints, 2) ~= 4
    error('plan_joint_trajectory:InvalidWaypoints', ...
        'Q_waypoints must be an N-by-4 matrix.');
end

nPoint = size(Q_waypoints, 1);
if nPoint < 2
    error('plan_joint_trajectory:TooFewWaypoints', ...
        'At least two waypoints are required for trajectory planning.');
end

nSegment = nPoint - 1;
segmentTimes = normalize_segment_times(segmentTimes, nSegment);

Q = [];
Qdot = [];
Qddot = [];
time = [];
segmentIndex = [];

currentTime = 0;

for i = 1:nSegment
    [qSeg, qdotSeg, qddotSeg, tSeg] = quintic_segment( ...
        Q_waypoints(i,:), Q_waypoints(i+1,:), segmentTimes(i), params.trajDt);

    if i > 1
        qSeg = qSeg(2:end,:);
        qdotSeg = qdotSeg(2:end,:);
        qddotSeg = qddotSeg(2:end,:);
        tSeg = tSeg(2:end);
    end

    Q = [Q; qSeg]; %#ok<AGROW>
    Qdot = [Qdot; qdotSeg]; %#ok<AGROW>
    Qddot = [Qddot; qddotSeg]; %#ok<AGROW>
    time = [time; currentTime + tSeg(:)]; %#ok<AGROW>
    segmentIndex = [segmentIndex; i * ones(numel(tSeg), 1)]; %#ok<AGROW>

    currentTime = currentTime + segmentTimes(i);
end

traj.Q = Q;
traj.Qdot = Qdot;
traj.Qddot = Qddot;
traj.time = time;
traj.segmentIndex = segmentIndex;
traj.segmentTimes = segmentTimes(:);
traj.Q_waypoints = Q_waypoints;
traj.dt = params.trajDt;
end

function segmentTimes = normalize_segment_times(segmentTimes, nSegment)
if isscalar(segmentTimes)
    segmentTimes = repmat(segmentTimes, nSegment, 1);
else
    segmentTimes = segmentTimes(:);
end

if numel(segmentTimes) ~= nSegment
    error('plan_joint_trajectory:SegmentTimeMismatch', ...
        'segmentTimes must be a scalar or contain %d values.', nSegment);
end

if any(segmentTimes <= 0)
    error('plan_joint_trajectory:InvalidSegmentTime', ...
        'All segment durations must be positive.');
end
end
