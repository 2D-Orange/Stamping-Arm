function run_regenerate_scene2_urdf()
%RUN_REGENERATE_SCENE2_URDF Rebuild exported/stamp_arm.urdf from scene2 config.

clc;

codeDir = fileparts(mfilename('fullpath'));
sceneDir = fileparts(codeDir);
projectDir = fileparts(sceneDir);
exportDir = fullfile(projectDir, 'exported');
overrideFile = fullfile(exportDir, 'scene2_urdf_joint_overrides.csv');
urdfFile = fullfile(exportDir, 'stamp_arm.urdf');
simscapeFile = fullfile(exportDir, 'stamp_arm_simscape.xml');

cfg = scene2_urdf_joint_config();
cfg = apply_scene2_urdf_joint_overrides(cfg, overrideFile);

write_scene2_urdf_from_config(cfg, exportDir, urdfFile);
copyfile(urdfFile, simscapeFile, 'f');

fprintf('Regenerated scene2 URDF:\n  %s\n', urdfFile);
fprintf('Updated Simscape XML copy:\n  %s\n', simscapeFile);
if exist(overrideFile, 'file')
    fprintf('Applied joint overrides:\n  %s\n', overrideFile);
else
    fprintf('No scene2 joint override file found; used default config.\n');
end
end
