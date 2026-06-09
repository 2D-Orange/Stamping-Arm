function cfg = apply_scene2_urdf_joint_overrides(cfg, overrideFile)
%APPLY_SCENE2_URDF_JOINT_OVERRIDES Apply saved scene2 joint authoring values.

if nargin < 2 || isempty(overrideFile) || ~exist(overrideFile, 'file')
    return;
end

overrides = readtable(overrideFile);
if isfield(cfg, 'frameConvention')
    if ~ismember('frame_convention', overrides.Properties.VariableNames)
        warning('apply_scene2_urdf_joint_overrides:LegacyFrameConvention', ...
            ['Ignoring legacy joint override file without frame_convention: %s\n', ...
            'Create a new override file with run_scene2_urdf_joint_authoring.'], ...
            overrideFile);
        return;
    end

    frameConventions = string(overrides.frame_convention);
    if any(frameConventions ~= string(cfg.frameConvention))
        warning('apply_scene2_urdf_joint_overrides:FrameConventionMismatch', ...
            ['Ignoring joint override file with a different frame convention: %s\n', ...
            'Expected: %s'], overrideFile, cfg.frameConvention);
        return;
    end
end

jointNames = {cfg.joints.name};

for i = 1:height(overrides)
    name = table_text(overrides.joint(i));
    idx = find(strcmp(jointNames, name), 1);
    if isempty(idx)
        warning('apply_scene2_urdf_joint_overrides:UnknownJoint', ...
            'Ignoring override for unknown joint: %s', name);
        continue;
    end

    cfg.joints(idx).origin = [
        overrides.origin_x(i), overrides.origin_y(i), overrides.origin_z(i)];

    if all(ismember({'rpy_r', 'rpy_p', 'rpy_y'}, overrides.Properties.VariableNames))
        cfg.joints(idx).rpy = [
            overrides.rpy_r(i), overrides.rpy_p(i), overrides.rpy_y(i)];
    end

    cfg.joints(idx).axis = normalize_axis([
        overrides.axis_x(i), overrides.axis_y(i), overrides.axis_z(i)]);

    if ismember('model_offset', overrides.Properties.VariableNames)
        cfg.joints(idx).modelOffset = overrides.model_offset(i);
    end

    if ismember('lower', overrides.Properties.VariableNames) && ...
            ismember('upper', overrides.Properties.VariableNames) && ...
            ~isempty(cfg.joints(idx).limit)
        cfg.joints(idx).limit(1:2) = [overrides.lower(i), overrides.upper(i)];
    end
end
end

function text = table_text(value)
if iscell(value)
    text = char(value{1});
elseif isstring(value)
    text = char(value);
else
    text = char(value);
end
end

function axis = normalize_axis(axis)
if norm(axis) < eps
    axis = [0, 0, 1];
else
    axis = axis ./ norm(axis);
end
end
