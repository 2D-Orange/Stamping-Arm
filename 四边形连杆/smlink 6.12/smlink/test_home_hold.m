clear;
clc;
close all;

%% ============================================================
%  generate_stamp_joint_press_single.m
%
%  纯关节空间盖章动作生成程序
%
%  不使用逆运动学。
%  不输入空间坐标。
%
%  用户只输入：
%  起始点关节角：[q1_deg q2_deg q3_deg]
%  目标点关节角：[q1_deg q2_deg q3_deg]
%
%  程序自动执行：
%  零位
%  -> 起始点
%  -> 起始点下压
%  -> 起始点升起
%  -> 目标点
%  -> 目标点下压
%  -> 目标点升起
%  -> 返回零位
%
%  运行后生成：
%  q1_ts, q2_ts, q3_ts, q4_ts, T_end
%
%  Simulink 中：
%  From Workspace 分别读取 q1_ts, q2_ts, q3_ts, q4_ts
%% ============================================================


%% ============================================================
% 1. 模型名称
%% ============================================================

modelName = 'stamp_arm';


%% ============================================================
% 2. 已调好的 Simscape 实际零位
%
%  当输入 q_home 时，机械臂保持你的机械零位。
%% ============================================================

q1_home = deg2rad(9);
q2_home = deg2rad(0);
q3_home = deg2rad(180);
q4_home = 0.040;

q_home = [q1_home, q2_home, q3_home, q4_home];


%% ============================================================
% 3. 关节方向修正
%
%  如果某个关节正方向和你希望的方向相反，
%  就把对应项从 1 改成 -1。
%
%  顺序：
%  [q1 基座, q2 大臂, q3 小臂, q4 印章下压]
%% ============================================================

jointSign = [1, -1, 1, -1];


%% ============================================================
% 4. 下压动作参数
%
%  这里单独设置，不需要每次输入。
%% ============================================================

startPressDepth_mm  = 40;     % 起始点下压量，单位 mm
targetPressDepth_mm = 40;     % 目标点下压量，单位 mm

startHoldTime  = 0.5;         % 起始点下压后保持时间，单位 s
targetHoldTime = 0.5;         % 目标点下压后保持时间，单位 s


%% ============================================================
% 5. 各段运动时间
%% ============================================================

T_home_to_start     = 3.0;    % 零位 -> 起始点
T_start_press_down  = 0.8;    % 起始点下压
T_start_press_up    = 0.8;    % 起始点升起

T_start_to_target   = 4.0;    % 起始点 -> 目标点

T_target_press_down = 0.8;    % 目标点下压
T_target_press_up   = 0.8;    % 目标点升起

T_return_home       = 3.0;    % 返回零位

dt = 0.02;


%% ============================================================
% 6. 默认输入
%
%  注意：
%  这里输入的是相对零位的关节角度，单位 deg。
%
%  例如 [20 10 -15] 表示：
%  q1 相对零位转 20°
%  q2 相对零位转 10°
%  q3 相对零位转 -15°
%% ============================================================

defaultStartJointDeg  = [30, 45, 45];
defaultTargetJointDeg = [-30, 45, 45];


%% ============================================================
% 7. 读取用户输入
%% ============================================================

fprintf('\n========== 盖章机器人关节空间盖章动作生成 ==========\n');
fprintf('输入格式：[q1_deg q2_deg q3_deg]\n');
fprintf('三个角度均为相对机械零位的旋转角度，单位 deg。\n\n');

fprintf('当前 Simscape 零位：\n');
fprintf('q_home = [%.3f deg, %.3f deg, %.3f deg, %.3f mm]\n\n', ...
    rad2deg(q_home(1)), rad2deg(q_home(2)), rad2deg(q_home(3)), 1000*q_home(4));

fprintf('起始点下压量：%.3f mm\n', startPressDepth_mm);
fprintf('目标点下压量：%.3f mm\n\n', targetPressDepth_mm);

fprintf('默认起始点关节角：[%g %g %g]\n', defaultStartJointDeg);
startJointDeg = input('请输入起始点关节角 [q1_deg q2_deg q3_deg]，直接回车使用默认值：');

if isempty(startJointDeg)
    startJointDeg = defaultStartJointDeg;
end

fprintf('默认目标点关节角：[%g %g %g]\n', defaultTargetJointDeg);
targetJointDeg = input('请输入目标点关节角 [q1_deg q2_deg q3_deg]，直接回车使用默认值：');

if isempty(targetJointDeg)
    targetJointDeg = defaultTargetJointDeg;
end

if numel(startJointDeg) ~= 3 || numel(targetJointDeg) ~= 3
    error('起始点和目标点都必须是 [q1_deg q2_deg q3_deg] 三个数。');
end

startJointDeg = reshape(startJointDeg, 1, 3);
targetJointDeg = reshape(targetJointDeg, 1, 3);


%% ============================================================
% 8. 构造各任务姿态
%% ============================================================

startUpJoint = [startJointDeg, 0];
startDownJoint = [startJointDeg, startPressDepth_mm];

targetUpJoint = [targetJointDeg, 0];
targetDownJoint = [targetJointDeg, targetPressDepth_mm];

q_start_up = joint_input_to_simscape(startUpJoint, q_home, jointSign);
q_start_down = joint_input_to_simscape(startDownJoint, q_home, jointSign);

q_target_up = joint_input_to_simscape(targetUpJoint, q_home, jointSign);
q_target_down = joint_input_to_simscape(targetDownJoint, q_home, jointSign);

q_final_home = q_home;


%% ============================================================
% 9. 任务点序列
%
%  相邻两个任务点相同时，就是保持动作。
%% ============================================================

Q_points = [
    q_home;
    q_start_up;
    q_start_down;
    q_start_down;
    q_start_up;
    q_target_up;
    q_target_down;
    q_target_down;
    q_target_up;
    q_final_home
];

taskNames = [
    "home";
    "start_up";
    "start_down";
    "start_hold";
    "start_up_again";
    "target_up";
    "target_down";
    "target_hold";
    "target_up_again";
    "home_return"
];

segmentTimes = [
    T_home_to_start;
    T_start_press_down;
    startHoldTime;
    T_start_press_up;
    T_start_to_target;
    T_target_press_down;
    targetHoldTime;
    T_target_press_up;
    T_return_home
];


%% ============================================================
% 10. 打印任务点
%% ============================================================

fprintf('\n========== 用户输入的相对关节任务 ==========\n');
fprintf('%-16s: [%8.3f %8.3f %8.3f %8.3f]\n', ...
    'start_up', startUpJoint(1), startUpJoint(2), startUpJoint(3), startUpJoint(4));
fprintf('%-16s: [%8.3f %8.3f %8.3f %8.3f]\n', ...
    'start_down', startDownJoint(1), startDownJoint(2), startDownJoint(3), startDownJoint(4));
fprintf('%-16s: [%8.3f %8.3f %8.3f %8.3f]\n', ...
    'target_up', targetUpJoint(1), targetUpJoint(2), targetUpJoint(3), targetUpJoint(4));
fprintf('%-16s: [%8.3f %8.3f %8.3f %8.3f]\n', ...
    'target_down', targetDownJoint(1), targetDownJoint(2), targetDownJoint(3), targetDownJoint(4));

fprintf('\n========== 实际送入 Simscape 的任务点 ==========\n');

for i = 1:size(Q_points, 1)
    fprintf('%-16s: q1 = %9.3f deg, q2 = %9.3f deg, q3 = %9.3f deg, q4 = %9.3f mm\n', ...
        taskNames(i), ...
        rad2deg(Q_points(i,1)), ...
        rad2deg(Q_points(i,2)), ...
        rad2deg(Q_points(i,3)), ...
        1000*Q_points(i,4));
end


%% ============================================================
% 11. 生成多段五次多项式轨迹
%% ============================================================

[time, Q, Qd, Qdd] = build_multi_segment_quintic(Q_points, segmentTimes, dt);

T_end = time(end);


%% ============================================================
% 12. 生成 Simulink 需要的 timeseries
%% ============================================================

q1_ts = timeseries(Q(:,1), time);
q2_ts = timeseries(Q(:,2), time);
q3_ts = timeseries(Q(:,3), time);
q4_ts = timeseries(Q(:,4), time);

assignin('base', 'q1_ts', q1_ts);
assignin('base', 'q2_ts', q2_ts);
assignin('base', 'q3_ts', q3_ts);
assignin('base', 'q4_ts', q4_ts);
assignin('base', 'T_end', T_end);

assignin('base', 'time', time);
assignin('base', 'Q', Q);
assignin('base', 'Qd', Qd);
assignin('base', 'Qdd', Qdd);

assignin('base', 'Q_points', Q_points);
assignin('base', 'segmentTimes', segmentTimes);
assignin('base', 'taskNames', taskNames);

assignin('base', 'q_home', q_home);
assignin('base', 'startJointDeg', startJointDeg);
assignin('base', 'targetJointDeg', targetJointDeg);
assignin('base', 'startPressDepth_mm', startPressDepth_mm);
assignin('base', 'targetPressDepth_mm', targetPressDepth_mm);

fprintf('\n已生成 q1_ts, q2_ts, q3_ts, q4_ts, T_end。\n');
fprintf('T_end = %.3f s\n', T_end);
fprintf('现在回到 Simulink，按 Ctrl+D 更新模型，然后点击 Run。\n');


%% ============================================================
% 13. 如果模型已经打开，自动设置 Stop Time
%% ============================================================

if bdIsLoaded(modelName)
    set_param(modelName, 'StopTime', 'T_end');
    set_param(modelName, 'SimulationCommand', 'update');
    fprintf('已自动更新模型 %s，并将 Stop Time 设置为 T_end。\n', modelName);
end


%% ============================================================
% 14. 绘制实际 Simscape 输入轨迹
%% ============================================================

figure('Name','Simscape 实际关节输入轨迹','Color','w');

subplot(4,1,1);
plot(time, rad2deg(Q(:,1)), 'LineWidth', 1.4);
grid on;
ylabel('q_1 / deg');

subplot(4,1,2);
plot(time, rad2deg(Q(:,2)), 'LineWidth', 1.4);
grid on;
ylabel('q_2 / deg');

subplot(4,1,3);
plot(time, rad2deg(Q(:,3)), 'LineWidth', 1.4);
grid on;
ylabel('q_3 / deg');

subplot(4,1,4);
plot(time, 1000*Q(:,4), 'LineWidth', 1.4);
grid on;
ylabel('q_4 / mm');
xlabel('Time / s');

sgtitle('Simscape 的关节轨迹');


%% ============================================================
% 16. 保存数据
%% ============================================================

save('stamp_joint_press_generated.mat', ...
    'q1_ts', 'q2_ts', 'q3_ts', 'q4_ts', 'T_end', ...
    'time', 'Q', 'Qd', 'Qdd', 'Q_relative', ...
    'Q_points', 'segmentTimes', 'taskNames', ...
    'q_home', 'startJointDeg', 'targetJointDeg', ...
    'startPressDepth_mm', 'targetPressDepth_mm', ...
    'startHoldTime', 'targetHoldTime');

disp('已保存 stamp_joint_press_generated.mat。');


%% ============================================================
%  局部函数区
%% ============================================================

function q_cmd = joint_input_to_simscape(jointInput, q_home, jointSign)
%JOINT_INPUT_TO_SIMSCAPE
%
% 用户输入：
% jointInput = [q1_deg q2_deg q3_deg q4_mm]
%
% 输出：
% q_cmd = [q1_rad q2_rad q3_rad q4_m]
%
% q_cmd 是真正送入 Simscape 的主动关节输入。

    q_rel = [
        deg2rad(jointInput(1)), ...
        deg2rad(jointInput(2)), ...
        deg2rad(jointInput(3)), ...
        jointInput(4) / 1000
    ];

    q_cmd = q_home + jointSign .* q_rel;

end


function [timeAll, QAll, QdAll, QddAll] = build_multi_segment_quintic(Q_points, segmentTimes, dt)
%BUILD_MULTI_SEGMENT_QUINTIC
%
% 多段五次多项式关节空间插值。
%
% 每一段满足：
% 起点速度 = 0
% 终点速度 = 0
% 起点加速度 = 0
% 终点加速度 = 0

    if size(Q_points,1) - 1 ~= numel(segmentTimes)
        error('segmentTimes 数量必须等于 Q_points 行数减 1。');
    end

    timeAll = [];
    QAll = [];
    QdAll = [];
    QddAll = [];

    currentTime = 0;

    for k = 1:size(Q_points,1)-1

        q0 = Q_points(k,:);
        qf = Q_points(k+1,:);
        T = segmentTimes(k);

        if T <= 0
            error('第 %d 段持续时间必须大于 0。', k);
        end

        t = (0:dt:T)';

        if t(end) < T
            t = [t; T];
        end

        tau = t / T;

        s = 10*tau.^3 - 15*tau.^4 + 6*tau.^5;
        sd = (30*tau.^2 - 60*tau.^3 + 30*tau.^4) / T;
        sdd = (60*tau - 180*tau.^2 + 120*tau.^3) / (T^2);

        dq = qf - q0;

        Qseg = q0 + s .* dq;
        Qdseg = sd .* dq;
        Qddseg = sdd .* dq;

        if k > 1
            t = t(2:end);
            Qseg = Qseg(2:end,:);
            Qdseg = Qdseg(2:end,:);
            Qddseg = Qddseg(2:end,:);
        end

        timeAll = [timeAll; currentTime + t]; %#ok<AGROW>
        QAll = [QAll; Qseg]; %#ok<AGROW>
        QdAll = [QdAll; Qdseg]; %#ok<AGROW>
        QddAll = [QddAll; Qddseg]; %#ok<AGROW>

        currentTime = currentTime + T;
    end

end