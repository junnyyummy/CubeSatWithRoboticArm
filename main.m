% main.m
% 主程序：调用轨迹生成与图形动画模块

clc; clear;
startup;  % 添加路径

% 获取参数
[L1, L2, boxLength, boxDepth, boxHeight, thickness] = model.arm_parameters();

% 从test.m获取目标点序列
[targetsA, targetsB] = test.test();

% 调用动画显示函数，传入目标点序列
ui.display_dual_arm(targetsA, targetsB, L1, L2, boxLength, boxDepth, boxHeight, thickness);