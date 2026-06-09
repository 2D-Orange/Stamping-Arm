%RUN_PROJECT One-command entry point for the stamping arm simulation project.

projectRoot = fileparts(mfilename('fullpath'));
codeDir = fullfile(projectRoot, '01_程序');

if ~exist(codeDir, 'dir')
    error('run_project:MissingCodeDir', ...
        'Code directory not found: %s', codeDir);
end

addpath(codeDir);
check_project_environment(projectRoot);
run_scene2_pipeline;
