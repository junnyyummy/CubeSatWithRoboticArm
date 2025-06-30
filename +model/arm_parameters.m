function [L1, L2, boxLength, boxDepth, boxHeight, thickness] = parameters()
    % 机械臂参数
    L1 = 35;  % 第一段长度
    L2 = 25;  % 第二段长度
    
    % 盒子参数
    boxLength = 36.6; 
    boxDepth = 10; 
    boxHeight = 26.6;
    thickness = 0.2;   % 盒子厚度 (2mm)
end