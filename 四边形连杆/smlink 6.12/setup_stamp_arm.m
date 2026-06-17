clear;
clc;

projectDir = fileparts(mfilename('fullpath'));
cd(projectDir);
addpath(genpath(projectDir));

modelName = 'stamp_arm';
dataFileName = 'stamp_arm_DataFile';

if ~isfile(fullfile(projectDir, [dataFileName, '.m']))
    error('缺少文件：%s.m', dataFileName);
end

load_system(modelName);

hws = get_param(modelName, 'ModelWorkspace');
hws.DataSource = 'MATLAB File';
hws.FileName = dataFileName;
hws.reload;

save_system(modelName);
open_system(modelName);

disp('stamp_arm 模型工作区已重新加载。');