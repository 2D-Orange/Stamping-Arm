function urdfInfo = find_scene2_urdf_package(projectRoot)
%FIND_SCENE2_URDF_PACKAGE Locate the SolidWorks-exported scene2 URDF package.

paths = stamp_project_paths(projectRoot);
candidates = [];
searchedRoots = {};

for i = 1:numel(paths.urdfRoots)
    urdfRoot = paths.urdfRoots{i};
    searchedRoots{end+1} = urdfRoot; %#ok<AGROW>
    rootCandidates = dir(fullfile(urdfRoot, '*', 'urdf', '*.urdf'));
    candidates = [candidates; rootCandidates]; %#ok<AGROW>
end

if isempty(candidates)
    error('find_scene2_urdf_package:MissingScene2Urdf', ...
        'No SolidWorks URDF was found under:\n  %s', ...
        strjoin(searchedRoots, sprintf('\n  ')));
end

[~, newestIndex] = max([candidates.datenum]);
sourceUrdfFile = fullfile(candidates(newestIndex).folder, ...
    candidates(newestIndex).name);
packageDir = fileparts(fileparts(sourceUrdfFile));
meshDir = fullfile(packageDir, 'meshes');

if ~exist(meshDir, 'dir')
    error('find_scene2_urdf_package:MissingMeshDir', ...
        'Mesh directory not found beside the SolidWorks URDF package: %s', ...
        meshDir);
end

requiredMeshes = {'base_link.STL', 'link_1.STL', 'link_2.STL', ...
    'link_3.STL', 'link_4.STL', 'link_5.STL'};
for i = 1:numel(requiredMeshes)
    meshFile = fullfile(meshDir, requiredMeshes{i});
    if ~exist(meshFile, 'file')
        error('find_scene2_urdf_package:MissingMesh', ...
            'Required mesh not found: %s', meshFile);
    end
end

urdfInfo.sourceUrdfFile = sourceUrdfFile;
urdfInfo.packageDir = packageDir;
urdfInfo.meshDir = meshDir;
urdfInfo.requiredMeshes = requiredMeshes;
urdfInfo.searchedRoots = searchedRoots;
end
