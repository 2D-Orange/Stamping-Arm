function paths = stamp_project_paths(projectRoot)
%STAMP_PROJECT_PATHS Return project paths for scripts and reproducible runs.

if nargin < 1 || isempty(projectRoot)
    codeDir = fileparts(mfilename('fullpath'));
    projectRoot = fileparts(codeDir);
else
    codeDir = fullfile(projectRoot, '01_程序');
end

paths.rootDir = projectRoot;
paths.codeDir = codeDir;
paths.modelsDir = fullfile(projectRoot, 'models');
paths.urdfRoots = {
    fullfile(projectRoot, 'models', 'urdf')
    fullfile(projectRoot, 'urdf')
    };
paths.resultDir = find_prefixed_output_dir(projectRoot, '02_', '02_results');
paths.videoDir = find_prefixed_output_dir(projectRoot, '03_', '03_video');
end

function outputDir = find_prefixed_output_dir(rootDir, prefix, fallbackName)
items = dir(fullfile(rootDir, [prefix, '*']));
items = items([items.isdir]);

if isempty(items)
    outputDir = fullfile(rootDir, fallbackName);
    return;
end

names = sort({items.name});
outputDir = fullfile(rootDir, names{1});
end
