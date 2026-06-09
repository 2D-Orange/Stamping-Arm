function ok = check_project_environment(projectRoot)
%CHECK_PROJECT_ENVIRONMENT Verify that this project can be reproduced.

if nargin < 1 || isempty(projectRoot)
    projectRoot = fileparts(mfilename('fullpath'));
end

codeDir = fullfile(projectRoot, '01_程序');
if exist(codeDir, 'dir')
    addpath(codeDir);
end

fprintf('Checking stamping arm project environment...\n');
fprintf('  Project root: %s\n', projectRoot);
fprintf('  MATLAB version: %s\n', version);

errors = {};
warnings = {};

requiredFunctions = {
    'VideoWriter', 'MATLAB'
    'exportgraphics', 'MATLAB'
    'readtable', 'MATLAB'
    'writetable', 'MATLAB'
    'importrobot', 'Robotics System Toolbox'
    };

for i = 1:size(requiredFunctions, 1)
    functionName = requiredFunctions{i, 1};
    toolboxName = requiredFunctions{i, 2};
    if ~function_available(functionName)
        errors{end+1} = sprintf('Missing %s function: %s', toolboxName, functionName); %#ok<AGROW>
    end
end

if ~license('test', 'Robotics_System_Toolbox')
    errors{end+1} = 'Robotics System Toolbox license is not available.'; %#ok<AGROW>
end

requiredFiles = {
    fullfile(codeDir, 'run_scene2_pipeline.m')
    fullfile(codeDir, 'run_joint_pd_tracking_urdf_animation.m')
    fullfile(codeDir, 'stamp_robot_params.m')
    };

for i = 1:numel(requiredFiles)
    if ~exist(requiredFiles{i}, 'file')
        errors{end+1} = sprintf('Missing required file: %s', requiredFiles{i}); %#ok<AGROW>
    end
end

try
    paths = stamp_project_paths(projectRoot);
    ensure_writable_dir(paths.resultDir);
    ensure_writable_dir(paths.videoDir);
    fprintf('  Result directory: %s\n', paths.resultDir);
    fprintf('  Video directory: %s\n', paths.videoDir);
catch err
    errors{end+1} = err.message; %#ok<AGROW>
end

try
    urdfInfo = find_scene2_urdf_package(projectRoot);
    fprintf('  Source URDF: %s\n', urdfInfo.sourceUrdfFile);
    fprintf('  Mesh directory: %s\n', urdfInfo.meshDir);
catch err
    errors{end+1} = err.message; %#ok<AGROW>
end

if exist(fullfile(projectRoot, 'models'), 'dir') ~= 7
    warnings{end+1} = 'models directory was not found; CAD source files may be missing.'; %#ok<AGROW>
end

for i = 1:numel(warnings)
    fprintf('WARNING: %s\n', warnings{i});
end

if isempty(errors)
    fprintf('Environment check passed.\n');
    ok = true;
else
    fprintf('Environment check failed:\n');
    for i = 1:numel(errors)
        fprintf('  - %s\n', errors{i});
    end
    ok = false;
    if nargout == 0
        error('check_project_environment:Failed', ...
            'Project environment check failed.');
    end
end
end

function ensure_writable_dir(dirName)
if ~exist(dirName, 'dir')
    mkdir(dirName);
end

probeFile = fullfile(dirName, '.write_test.tmp');
fid = fopen(probeFile, 'w');
if fid < 0
    error('check_project_environment:NotWritable', ...
        'Directory is not writable: %s', dirName);
end
cleanup = onCleanup(@() cleanup_probe(fid, probeFile)); %#ok<NASGU>
fprintf(fid, 'ok\n');
end

function cleanup_probe(fid, probeFile)
fclose(fid);
if exist(probeFile, 'file')
    delete(probeFile);
end
end

function available = function_available(functionName)
available = any(exist(functionName, 'file') == [2, 3, 5, 6]) || ...
    exist(functionName, 'builtin') == 5 || ~isempty(which(functionName));
end
