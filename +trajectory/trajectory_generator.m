function [time_vector, joint_angles, end_positions] = trajectory_generator(current_angles, target_pos_a, target_pos_b, L1, L2)
% 打桩验证：每个关节都做完整 1~2 圈旋转
numFrames = 200;
time_vector = linspace(0, 4*pi, numFrames);  % 共用时间向量（时间单位可选）

joint_angles = zeros(length(time_vector), 6);
joint_angles(:,4) = linspace(0, 2*pi, length(time_vector));
joint_angles(:,1) = linspace(0, 2*pi, length(time_vector));
joint_angles(:,5) = linspace(0, 2*pi, length(time_vector));
joint_angles(:,2) = linspace(0, 2*pi, length(time_vector));
joint_angles(:,6) = linspace(0, 2*pi, length(time_vector));
joint_angles(:,3) = linspace(0, 2*pi, length(time_vector));
[L1, L2] = model.arm_parameters();

% 计算每帧末端位置（用于轨迹绘图）
end_positions = zeros(numFrames, 6);
for i = 1:numFrames
    [pos_a, pos_b] = get_current_positions(joint_angles(i, :), L1, L2);
    end_positions(i, 1:3) = pos_a;
    end_positions(i, 4:6) = pos_b;
end
    % 确保输入为行向量
    % current_angles = current_angles(:)';
    % 
    % % 参数设置
    % max_joint_velocity = 0.2 * pi; % 关节最大角速度 (rad/s)
    % control_freq = 50;             % 控制频率 (Hz)
    % 
    % % 计算两个机械臂的目标关节角度
    % [q1a_end, q2a_end, q3a_end] = inverse_kinematics(target_pos_a, L1, L2);
    % [q1b_end, q2b_end, q3b_end] = inverse_kinematics(target_pos_b, L1, L2);
    % 
    % % 计算关节角度差（分别处理A、B两个机械臂）
    % delta_angles_a = [q1a_end, q2a_end, q3a_end] - current_angles(1:3);
    % delta_angles_b = [q1b_end, q2b_end, q3b_end] - current_angles(4:6);
    % 
    % % 计算各关节所需时间（取最大值作为总时间）
    % time_per_joint_a = abs(delta_angles_a) / max_joint_velocity;
    % time_per_joint_b = abs(delta_angles_b) / max_joint_velocity;
    % total_time = max([time_per_joint_a, time_per_joint_b]);
    % 
    % % 生成时间向量
    % time_vector = 0:1/control_freq:total_time;
    % num_points = length(time_vector);
    % 
    % % 预分配内存
    % joint_angles = zeros(num_points, 6);
    % end_positions = zeros(num_points, 6); % [xA,yA,zA, xB,yB,zB]
    % 
    % % 生成轨迹（关节空间线性插值）
    % for i = 1:num_points
    %     t_frac = time_vector(i) / total_time;
    % 
    %     % 分别插值两个机械臂的关节角度
    %     angles_a = current_angles(1:3) + t_frac * delta_angles_a;
    %     angles_b = current_angles(4:6) + t_frac * delta_angles_b;
    % 
    %     % 合并结果
    %     joint_angles(i, :) = [angles_a, angles_b];
    % 
    %     % 计算当前末端位置
    %     [pos_a, pos_b] = get_current_positions(joint_angles(i, :), L1, L2);
    %     end_positions(i, 1:3) = pos_a;
    %     end_positions(i, 4:6) = pos_b;
    % end
    % 
    % % 验证末端位置准确性
    % final_pos_a = end_positions(end, 1:3);
    % final_pos_b = end_positions(end, 4:6);
    % 
    % if norm(final_pos_a - target_pos_a) > 0.1
    %     warning('机械臂A末端位置误差: %.4f cm', norm(final_pos_a - target_pos_a));
    % end
    % 
    % if norm(final_pos_b - target_pos_b) > 0.1
    %     warning('机械臂B末端位置误差: %.4f cm', norm(final_pos_b - target_pos_b));
    % end
end

function [q1, q2, q3] = inverse_kinematics(end_pos, L1, L2)
    % 改进的逆运动学求解 - 添加可达性验证
    x = end_pos(1); y = end_pos(2); z = end_pos(3);
    
    % 计算水平距离和高度
    r = sqrt(x^2 + y^2);
    h = z;
    
    % 验证可达性
    total_distance = norm([r, h]);
    if total_distance > (L1 + L2) || total_distance < abs(L1 - L2)
        error('目标点不可达: 距离原点 %.2f cm (工作空间范围: %.2f-%.2f cm)', ...
              total_distance, abs(L1-L2), L1+L2);
    end
    
    % 计算关节角度
    q1 = atan2(y, x);
    
    % 使用余弦定理计算关节2角度
    D = (r^2 + h^2 - L1^2 - L2^2) / (2 * L1 * L2);
    
    % 验证解的存在性
    if abs(D) > 1
        error('目标点不可达: 几何约束不满足 (D=%.2f)', D);
    end
    
    % 计算可能的解 (肘部向上/向下)
    q3 = atan2(sqrt(1 - D^2), D);  % 肘部向下解
    
    % 计算关节2角度
    alpha = atan2(h, r);
    beta = atan2(L2 * sin(q3), L1 + L2 * cos(q3));
    q2 = alpha - beta;
    
    % 应用关节限制
    [q1, q2, q3] = check_joint_limits([q1, q2, q3]);
    
    % 验证解的正确性
    [calc_pos] = forward_kinematics(q1, q2, q3, L1, L2);
    if norm(calc_pos - end_pos) > 0.1
        % 尝试另一种配置
        q3 = atan2(-sqrt(1 - D^2), D); % 肘部向上解
        beta = atan2(L2 * sin(q3), L1 + L2 * cos(q3));
        q2 = alpha - beta;
        [q1, q2, q3] = check_joint_limits([q1, q2, q3]);
        
        % 重新验证
        [calc_pos] = forward_kinematics(q1, q2, q3, L1, L2);
        if norm(calc_pos - end_pos) > 0.1
            error('逆运动学求解失败: 计算位置与目标位置偏差 %.2f cm', norm(calc_pos - end_pos));
        end
    end
end

function [pos] = forward_kinematics(q1, q2, q3, L1, L2)
    % 正运动学验证函数
    joint3_pos = [
        L1 * cos(q1) * cos(q2),
        L1 * sin(q1) * cos(q2),
        L1 * sin(q2)
    ];
    
    R = rotz(q1) * roty(q2) * rotz(q3);
    end_effector = joint3_pos + R * [L2; 0; 0];
    pos = end_effector';
end

% 保留原有的关节限制函数、正运动学函数和旋转矩阵函数
% (与之前相同)

% 保留原有的关节限制函数
function [q1, q2, q3] = check_joint_limits(angles)
    limits = [
        -pi, pi;          % θ1: 全范围旋转
        -pi/2, pi/2;      % θ2: ±90度
        -pi, pi           % θ3: 全范围旋转
    ];
    
    for i = 1:3
        angles(i) = max(min(angles(i), limits(i,2)), limits(i,1));
    end
    q1 = angles(1); 
    q2 = angles(2); 
    q3 = angles(3);
end

% 保留原有的正运动学函数
function [pos_a, pos_b] = get_current_positions(current_angles, L1, L2)
    % 从当前关节角度计算末端位置（包含基座偏移）
    thickness = 0.2; % 盒子厚度 (cm)
    boxLength = 30;  % 盒子长度 (cm)
    boxHeight = 10;  % 盒子高度 (cm)
    
    % 计算基座中心位置（全局坐标系）
    baseBlue = [2*thickness, 0, 2*thickness];                 % 蓝臂左下角靠里
    baseRed = [boxLength - 2*thickness, 0, boxHeight - 2*thickness];  % 红臂右上角靠里

    % 机械臂A
    theta1a = current_angles(1);
    theta2a = current_angles(2);
    theta3a = current_angles(3);
    
    joint3_pos_a = [
        L1 * cos(theta1a) * cos(theta2a),
        L1 * sin(theta1a) * cos(theta2a),
        L1 * sin(-theta2a)
    ];
    
    R_a = rotz(theta1a) * roty(theta2a) * rotz(theta3a);
    end_effector_a = joint3_pos_a + R_a * [L2; 0; 0];
    pos_a = end_effector_a' + baseBlue; % 添加基座偏移
    
    % 机械臂B
    theta1b = current_angles(4);
    theta2b = current_angles(5);
    theta3b = current_angles(6);
    
    joint3_pos_b = [
        L1 * cos(theta1b) * cos(theta2b),
        L1 * sin(theta1b) * cos(theta2b),
        L1 * sin(-theta2b)
    ];
    
    R_b = rotz(theta1b) * roty(theta2b) * rotz(theta3b);
    end_effector_b = joint3_pos_b + R_b * [L2; 0; 0];
    pos_b = end_effector_b' + baseRed; % 添加基座偏移
    
end

% 保留旋转矩阵函数
function R = rotz(theta)
    R = [cos(theta), -sin(theta), 0;
         sin(theta),  cos(theta), 0;
         0, 0, 1];
end

function R = roty(theta)
    R = [cos(theta), 0, sin(theta);
         0, 1, 0;
         -sin(theta), 0, cos(theta)];
end