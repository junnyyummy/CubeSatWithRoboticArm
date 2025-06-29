function [targetsA, targetsB] = test()
    % 生成测试目标点序列
    % 返回: 
    %   targetsA - 蓝臂目标点序列 (n×3 矩阵)
    %   targetsB - 红臂目标点序列 (n×3 矩阵)
    
    % 盒子尺寸
    boxLength = 30; boxHeight = 10; boxDepth = 10;
    thickness = 0.2;
    
    % 盒子中心
    boxCenter = [boxLength/2, boxDepth/2, boxHeight/2];
    
    % 基本目标位置（盒子中心正前方35cm）
    baseTarget = boxCenter + [0, -35, 0];  % Y负方向为正前方
    
    % 创建目标点序列
    targets = [];
    % 定义O点
    [L1, L2] = model.arm_parameters();
    boxLength = 30;
    thickness = 0.2;   % 盒子厚度 (2mm)

    % 基座位置
    baseBlue = [2*thickness, 0, 2*thickness];                 % 蓝臂左下角靠里
    baseRed = [boxLength - 2*thickness, 0, boxHeight - 2*thickness];  % 红臂右上角靠里
    Oa = [baseBlue(1)+L1+L2,baseBlue(2),baseBlue(3)];
    Ob = [baseRed(1)+L1+L2,baseRed(2),baseRed(3)];


    % 定义握手动作的点序列
    % 点A: 基本目标位置
    A = baseTarget;
    % 点B: A点上方10cm
    B = baseTarget + [0, 0, 10];
    % 点C: A点下方10cm
    C = baseTarget + [0, 0, -10];
    
    % 创建握手动作序列 [A, B, C, A, C, A, C, A, C]
    targetsA = [Oa; A; B; C; B; C; B; C; B; C; Oa];
    targetsB = [Ob; A; B; C; B; C; B; C; B; C; Ob];
    
    % 可选：添加更多测试序列
    % 例如：targetsA = [targetsA; A + [0, -5, 0]; A + [0, -10, 0]];
    %        targetsB = [targetsB; A + [0, -5, 0]; A + [0, -10, 0]];
end