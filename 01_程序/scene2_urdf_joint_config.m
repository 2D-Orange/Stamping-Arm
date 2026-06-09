function cfg = scene2_urdf_joint_config()
%SCENE2_URDF_JOINT_CONFIG Configurable scene2 RRRP URDF assembly model.
%
% Joint frames follow the same non-standard DH convention used by
% forward_kinematics_stamp:
%   T01 = Tz(L1) * Rz(q1)
%   T12 = Ry(-q2) * Tx(L2)
%   T23 = Ry(-q3) * Tx(L3)
%   T3a = Ry(q2+q3) * Tx(L_axis)
%   Ta4 = Tz(-(H_stamp+q4))
%
% Model coordinates:
%   X: radial outward from the base yaw axis at q1 = 0
%   Y: horizontal lateral, completing a right-handed frame
%   Z: vertical upward
%
% SOLIDWORKS scene2 meshes are exported in CAD assembly coordinates:
%   CAD X = model Y, CAD Y = model Z, CAD Z = model X.
% The mesh transform below maps CAD assembly vertices into the DH model
% frame. Joint origins themselves are DH joint centers, not mesh bbox points.

cfg.robotName = 'scene2_stamp_arm';
cfg.meshScale = [0.001, 0.001, 0.001];
cfg.frameConvention = 'scene2_nonstandard_dh_z_up';

cadBaseOrigin = [0.5169, 0.9171, 0.1084];
cfg.cadToModel.R = [
    0, 0, 1
    1, 0, 0
    0, 1, 0
    ];
cfg.cadToModel.t = -cfg.cadToModel.R * cadBaseOrigin(:);

L1 = 0.150;
L2 = 0.250;
L3 = 0.300;
L_axis = 0.110;
H_stamp = 0.010;
q4_max = 0.120;

base = [0, 0, 0];
shoulder = [0, 0, L1];
elbow = [L2, 0, L1];
wrist = [L2 + L3, 0, L1];
stampAxis = [L2 + L3 + L_axis, 0, L1];
stampHome = [L2 + L3 + L_axis, 0, L1 - H_stamp];

tableCenter = cad_to_model([0.5169, 0.7121, 0.3084], cfg.cadToModel);
baseCenter = cad_to_model([0.5169, 1.0071, 0.1084], cfg.cadToModel);
arm1Center = cad_to_model([0.5226, 1.1761, 0.1464], cfg.cadToModel);
arm2Center = cad_to_model([0.5461, 1.2053, 0.3032], cfg.cadToModel);
handCenter = cad_to_model([0.5713, 1.1142, 0.4716], cfg.cadToModel);
stampCenter = cad_to_model([0.5803, 1.0561, 0.5313], cfg.cadToModel);

cfg.links = [
    make_link('table_link', base, 7.0, ...
        tableCenter, cad_size_to_model_size([0.6000, 0.4100, 0.6000]), ...
        {'table'})
    make_link('base_link', base, 1.0, ...
        baseCenter, cad_size_to_model_size([0.1000, 0.2000, 0.1000]), ...
        {'base'})
    make_link('yaw_link', shoulder, 0.01, ...
        shoulder, [0.0100, 0.0100, 0.0100], ...
        {})
    make_link('upper_arm_link', shoulder, 0.45, ...
        arm1Center, cad_size_to_model_size([0.1053, 0.3377, 0.1867]), ...
        {'arm1'})
    make_link('forearm_link', elbow, 0.45, ...
        arm2Center, cad_size_to_model_size([0.0900, 0.2795, 0.3423]), ...
        {'arm2'})
    make_link('hand_link', wrist, 0.25, ...
        handCenter, cad_size_to_model_size([0.1088, 0.1025, 0.2099]), ...
        {'hand'})
    make_link('stamp_link', stampHome, 0.20, ...
        stampCenter, cad_size_to_model_size([0.0600, 0.1519, 0.0603]), ...
        {'stamp'})
    ];

cfg.joints = [
    make_joint('world_to_table', 'fixed', 'world', 'table_link', ...
        [0, 0, 0], [0, 0, 0], [0, 0, 0], [], [0, 0, 0, 0], 0)
    make_joint('table_to_base', 'fixed', 'table_link', 'base_link', ...
        [0, 0, 0], [0, 0, 0], [0, 0, 0], [], [0, 0, 0, 0], 0)
    make_joint('base_yaw', 'revolute', 'base_link', 'yaw_link', ...
        shoulder, [0, 0, 0], [0, 0, 1], [-pi, pi, 40, 2.0], [1, 0, 0, 0], 0)
    make_joint('shoulder_pitch', 'revolute', 'yaw_link', 'upper_arm_link', ...
        [0, 0, 0], [0, 0, 0], [0, -1, 0], [-pi, pi, 30, 2.0], [0, 1, 0, 0], 0)
    make_joint('elbow_pitch', 'revolute', 'upper_arm_link', 'forearm_link', ...
        [L2, 0, 0], [0, 0, 0], [0, -1, 0], [-pi, pi, 25, 2.0], [0, 0, 1, 0], 0)
    make_joint('wrist_level', 'revolute', 'forearm_link', 'hand_link', ...
        [L3, 0, 0], [0, 0, 0], [0, 1, 0], [-2*pi, 2*pi, 10, 2.0], [0, 1, 1, 0], 0)
    make_joint('stamp_prismatic', 'prismatic', 'hand_link', 'stamp_link', ...
        [L_axis, 0, -H_stamp], [0, 0, 0], [0, 0, -1], [0, q4_max, 80, 0.25], [0, 0, 0, 1], 0)
    ];
end

function point = cad_to_model(cadPoint, cadToModel)
point = (cadToModel.R * cadPoint(:) + cadToModel.t).';
end

function sizeModel = cad_size_to_model_size(sizeCad)
sizeModel = [sizeCad(3), sizeCad(1), sizeCad(2)];
end

function link = make_link(name, frame, mass, bboxCenter, bboxSize, meshKeys)
link = struct( ...
    'name', name, ...
    'frame', frame, ...
    'mass', mass, ...
    'bboxCenter', bboxCenter, ...
    'bboxSize', bboxSize, ...
    'meshKeys', {meshKeys});
end

function joint = make_joint(name, type, parent, child, origin, rpy, axis, limit, ...
    modelCoefficients, modelOffset)
joint = struct( ...
    'name', name, ...
    'type', type, ...
    'parent', parent, ...
    'child', child, ...
    'origin', origin, ...
    'rpy', rpy, ...
    'axis', axis, ...
    'limit', limit, ...
    'modelCoefficients', modelCoefficients, ...
    'modelOffset', modelOffset);
end
