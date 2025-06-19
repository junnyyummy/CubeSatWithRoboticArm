% startup.m - 自动添加非包目录路径（例如 resources 或其他非 + 文件夹）

% 如果你确实有非包目录需要 addpath，可以加下面这行（如果有的话）
% addpath(genpath('resources'));  % 示例：非包目录

% 不需要对 +ui, +trajectory, +model 做 addpath
disp('路径初始化完成。');
