function write_scene2_urdf_from_config(cfg, exportDir, urdfFile)
%WRITE_SCENE2_URDF_FROM_CONFIG Generate scene2 URDF from editable config.

meshDir = fullfile(exportDir, 'meshes');
if ~exist(meshDir, 'dir')
    error('write_scene2_urdf_from_config:MissingMeshDir', ...
        'Mesh directory not found: %s', meshDir);
end

fid = fopen(urdfFile, 'w');
if fid < 0
    error('write_scene2_urdf_from_config:CannotOpenFile', ...
        'Cannot open URDF for writing: %s', urdfFile);
end
cleanup = onCleanup(@() fclose(fid));

fprintf(fid, '<?xml version="1.0"?>\n');
fprintf(fid, '<!-- Configurable scene2 RRRP URDF regenerated from scene2_urdf_joint_config.m. -->\n');
fprintf(fid, '<!-- Joint frames follow scene2 non-standard DH model coordinates. -->\n');
fprintf(fid, '<!-- Meshes are SOLIDWORKS assembly-position STL files transformed into the DH model frame. -->\n');
fprintf(fid, '<robot name="%s">\n', xml_escape(cfg.robotName));
fprintf(fid, '  <link name="world"/>\n');

for i = 1:numel(cfg.links)
    write_link(fid, cfg.links(i), cfg, meshDir);
end

for i = 1:numel(cfg.joints)
    write_joint(fid, cfg.joints(i));
end

fprintf(fid, '</robot>\n');
end

function write_link(fid, link, cfg, meshDir)
fprintf(fid, '  <link name="%s">\n', xml_escape(link.name));
write_inertial(fid, link);

for i = 1:numel(link.meshKeys)
    meshName = find_mesh(meshDir, link.meshKeys{i});
    if isempty(meshName)
        warning('write_scene2_urdf_from_config:MissingMesh', ...
            'No mesh found for key "%s" in link "%s".', ...
            link.meshKeys{i}, link.name);
        continue;
    end

    [meshOrigin, meshRpy] = mesh_transform_for_link(link, cfg);
    write_mesh_element(fid, 'visual', meshName, meshOrigin, meshRpy, cfg.meshScale);
    write_mesh_element(fid, 'collision', meshName, meshOrigin, meshRpy, cfg.meshScale);
end

fprintf(fid, '  </link>\n');
end

function write_inertial(fid, link)
mass = link.mass;
center = link.bboxCenter - link.frame;
size = link.bboxSize;
ixx = mass * (size(2)^2 + size(3)^2) / 12.0;
iyy = mass * (size(1)^2 + size(3)^2) / 12.0;
izz = mass * (size(1)^2 + size(2)^2) / 12.0;

fprintf(fid, '    <inertial>\n');
fprintf(fid, '      <origin xyz="%s" rpy="0.0 0.0 0.0"/>\n', vec_string(center));
fprintf(fid, '      <mass value="%s"/>\n', num_string(mass));
fprintf(fid, ['      <inertia ixx="%s" ixy="0.0" ixz="0.0" ', ...
    'iyy="%s" iyz="0.0" izz="%s"/>\n'], ...
    num_string(ixx), num_string(iyy), num_string(izz));
fprintf(fid, '    </inertial>\n');
end

function [origin, rpy] = mesh_transform_for_link(link, cfg)
if isfield(cfg, 'cadToModel') && isfield(cfg.cadToModel, 'R') && ...
        isfield(cfg.cadToModel, 't')
    T_model_cad = eye(4);
    T_model_cad(1:3,1:3) = cfg.cadToModel.R;
    T_model_cad(1:3,4) = cfg.cadToModel.t(:);

    T_model_link = eye(4);
    T_model_link(1:3,4) = link.frame(:);

    T_link_cad = T_model_link \ T_model_cad;
    origin = T_link_cad(1:3,4).';
    rpy = rotm_to_rpy(T_link_cad(1:3,1:3));
else
    origin = -link.frame;
    rpy = [0, 0, 0];
end
end

function rpy = rotm_to_rpy(R)
pitch = atan2(-R(3,1), hypot(R(1,1), R(2,1)));
if abs(cos(pitch)) > 1e-12
    roll = atan2(R(3,2), R(3,3));
    yaw = atan2(R(2,1), R(1,1));
else
    roll = atan2(-R(2,3), R(2,2));
    yaw = 0;
end
rpy = [roll, pitch, yaw];
end

function write_mesh_element(fid, tagName, meshName, origin, rpy, meshScale)
fprintf(fid, '    <%s>\n', tagName);
fprintf(fid, '      <origin xyz="%s" rpy="%s"/>\n', ...
    vec_string(origin), vec_string(rpy));
fprintf(fid, ['      <geometry><mesh filename="meshes/%s" ', ...
    'scale="%s"/></geometry>\n'], ...
    xml_escape(meshName), vec_string(meshScale));
fprintf(fid, '    </%s>\n', tagName);
end

function write_joint(fid, joint)
fprintf(fid, '  <joint name="%s" type="%s">\n', ...
    xml_escape(joint.name), xml_escape(joint.type));
fprintf(fid, '    <parent link="%s"/>\n', xml_escape(joint.parent));
fprintf(fid, '    <child link="%s"/>\n', xml_escape(joint.child));
fprintf(fid, '    <origin xyz="%s" rpy="%s"/>\n', ...
    vec_string(joint.origin), vec_string(joint.rpy));

if ~strcmp(joint.type, 'fixed')
    fprintf(fid, '    <axis xyz="%s"/>\n', vec_string(normalize_axis(joint.axis)));
    if ~isempty(joint.limit)
        fprintf(fid, ['    <limit lower="%s" upper="%s" effort="%s" ', ...
            'velocity="%s"/>\n'], ...
            num_string(joint.limit(1)), num_string(joint.limit(2)), ...
            num_string(joint.limit(3)), num_string(joint.limit(4)));
        fprintf(fid, '    <dynamics damping="0.05" friction="0.0"/>\n');
    end
end

fprintf(fid, '  </joint>\n');
end

function meshName = find_mesh(meshDir, key)
files = dir(fullfile(meshDir, '*.stl'));
meshName = '';
safeKey = lower(regexprep(char(key), '[^A-Za-z0-9_.]', '_'));

for i = 1:numel(files)
    name = lower(files(i).name);
    stem = regexprep(name, '\.stl$', '');
    if contains(stem, ['___', safeKey, '_']) || ...
            endsWith(stem, ['___', safeKey]) || ...
            strcmp(stem, safeKey) || ...
            (~contains(stem, '___') && startsWith(stem, [safeKey, '_']))
        meshName = files(i).name;
        return;
    end
end
end

function axis = normalize_axis(axis)
if norm(axis) < eps
    axis = [0, 0, 1];
else
    axis = axis ./ norm(axis);
end
end

function text = vec_string(values)
text = strjoin(arrayfun(@num_string, values(:).', 'UniformOutput', false), ' ');
end

function text = num_string(value)
text = sprintf('%.9g', value);
if ~contains(text, '.') && ~contains(lower(text), 'e')
    text = [text, '.0'];
end
end

function text = xml_escape(text)
text = char(text);
text = strrep(text, '&', '&amp;');
text = strrep(text, '"', '&quot;');
text = strrep(text, '<', '&lt;');
text = strrep(text, '>', '&gt;');
end
