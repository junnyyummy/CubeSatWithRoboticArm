function [targetsA, targetsB] = test()
    % 获取参数
    [L1, L2, boxLength, boxDepth, boxHeight, thickness] = model.arm_parameters();
    
    % 基座位置
    baseBlue = [2*thickness, 0, 2*thickness];                 % 蓝臂左下角靠里
    baseRed = [boxLength - 2*thickness, 0, boxHeight - 2*thickness];  % 红臂右上角靠里
    Oa = [baseBlue(1)+L1+L2,baseBlue(2),baseBlue(3)];
    Ob = [baseRed(1)+L1+L2,baseRed(2),baseRed(3)];

    % 盒子中心
    boxCenter = [boxLength/2, boxDepth/2, boxHeight/2];
    
    % 基本目标位置（盒子中心正前方35cm）
    baseTarget = boxCenter + [0, -45, 0];  % Y负方向为正前方
    
    % 创建握手动作序列 [A, B, C, A, C, A, C, A, C]
    targetsA = [Oa; baseTarget; baseTarget + [0, 0, 10]; baseTarget + [0, 0, -10]; Oa];
    targetsB = [Ob; baseTarget; baseTarget + [0, 0, 10]; baseTarget + [0, 0, -10]; Ob];
end