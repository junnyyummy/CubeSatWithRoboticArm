function [time_vector, joint_angles, end_positions] = trajectory_generator(current_angles, target_pos_a, target_pos_b, L1, L2)
    % 确保输入为行向量
    current_angles = current_angles(:)';
    
    % 参数设置
    max_joint_velocity = 0.2 * pi; % 关节最大角速度 (rad/s)
    control_freq = 50;             % 控制频率 (Hz)
    min_speed = 0.01 * pi;         % 最小关节速度 (防止除零)
    
    % 盒子参数
    thickness = 0.2; % 盒子厚度 (cm)
    boxLength = 30;  % 盒子长度 (cm)
    boxHeight = 10;  % 盒子高度 (cm)
    
    % 计算基座中心位置（全局坐标系）
    baseBlue = [2*thickness, 0, 2*thickness];                 % 蓝臂左下角靠里
    baseRed = [boxLength - 2*thickness, 0, boxHeight - 2*thickness];  % 红臂右上角靠里
    
    % 将全局目标位置转换到基座坐标系
    target_local_a = target_pos_a - baseBlue;
    target_local_b = target_pos_b - baseRed;
    
    % 获取当前末端位置
    [current_pos_a, current_pos_b] = get_current_positions(current_angles, L1, L2);
    
    % 计算两个机械臂的目标关节角度
    [q1a_end, q2a_end, q3a_end] = inverse_kinematics(target_local_a, L1, L2, 'blue');
    [q1b_end, q2b_end, q3b_end] = inverse_kinematics(target_local_b, L1, L2, 'red');
    
    % 计算关节角度差（分别处理A、B两个机械臂）
    delta_angles_a = [q1a_end, q2a_end, q3a_end] - current_angles(1:3);
    delta_angles_b = [q1b_end, q2b_end, q3b_end] - current_angles(4:6);
    
    % 计算各关节所需时间（取最大值作为总时间）
    time_per_joint_a = abs(delta_angles_a) ./ max(max_joint_velocity, min_speed);
    time_per_joint_b = abs(delta_angles_b) ./ max(max_joint_velocity, min_speed);
    total_time = max([time_per_joint_a, time_per_joint_b]);
    
    % 生成时间向量
    time_vector = 0:1/control_freq:total_time;
    num_points = length(time_vector);
    
    % 预分配内存
    joint_angles = zeros(num_points, 6);
    end_positions = zeros(num_points, 6); % [xA,yA,zA, xB,yB,zB]
    
    % 生成轨迹（关节空间线性插值）
    for i = 1:num_points
        t_frac = time_vector(i) / total_time;
        
        % 使用平滑的S型曲线插值（避免速度突变）
        smoothed_frac = smoothstep(t_frac);
        
        % 分别插值两个机械臂的关节角度
        angles_a = current_angles(1:3) + smoothed_frac * delta_angles_a;
        angles_b = current_angles(4:6) + smoothed_frac * delta_angles_b;
        
        % 合并结果
        joint_angles(i, :) = [angles_a, angles_b];
        
        % 计算当前末端位置
        [pos_a, pos_b] = get_current_positions(joint_angles(i, :), L1, L2);
        end_positions(i, 1:3) = pos_a;
        end_positions(i, 4:6) = pos_b;
    end
    
    % 验证末端位置准确性
    final_pos_a = end_positions(end, 1:3);
    final_pos_b = end_positions(end, 4:6);
    
    if norm(final_pos_a - target_pos_a) > 0.1
        warning('机械臂A末端位置误差: %.4f cm', norm(final_pos_a - target_pos_a));
    end
    
    if norm(final_pos_b - target_pos_b) > 0.1
        warning('机械臂B末端位置误差: %.4f cm', norm(final_pos_b - target_pos_b));
    end
end

% 平滑过渡函数（S型曲线）
function y = smoothstep(x)
    % 三次多项式平滑过渡
    y = 3*x.^2 - 2*x.^3;
end

function [q1, q2, q3] = inverse_kinematics(end_pos, L1, L2, arm_color)
    % 完全按照您描述的算法实现的逆运动学
    % 输入：end_pos - 在基座坐标系中的目标位置 [x, y, z]
    %       L1, L2 - 连杆长度
    %       arm_color - 'blue'或'red'，用于确定左右臂配置
    
    x = end_pos(1); y = end_pos(2); z = end_pos(3);
    
    % 1. 计算水平距离和高度
    r = sqrt(x^2 + y^2);  % XY平面投影距离
    h = z;                % 高度
    
    % 2. 计算基座到目标点的距离
    R = sqrt(r^2 + h^2);  % 基座到目标点的直线距离
    
    % 3. 验证可达性
    if R > (L1 + L2) || R < abs(L1 - L2)
        error('目标点不可达: 距离基座 %.2f cm (工作空间范围: %.2f-%.2f cm)', ...
              R, abs(L1-L2), L1+L2);
    end
    
    % 4. 计算q2角度（由高度和水平距离决定）
    % q2控制整个机械臂的俯仰角
    q2 = atan2(h, r);
    
    % 5. 计算L1和基座到目标点连线之间的夹角
    % 使用余弦定理：cos(alpha) = (L1^2 + R^2 - L2^2) / (2 * L1 * R)
    cos_alpha = (L1^2 + R^2 - L2^2) / (2 * L1 * R);
    
    % 验证解的存在性
    if abs(cos_alpha) > 1
        error('目标点不可达: 几何约束不满足 (cos_alpha=%.2f)', cos_alpha);
    end
    
    alpha = acos(cos_alpha);
    
    % 6. 计算关节1角度
    % 基座到目标点连线在XY平面的投影角度
    phi = atan2(y, x);
    
    % 根据机械臂颜色确定加减关系
    if strcmpi(arm_color, 'blue')
        % 蓝色臂（右侧）使用减法
        q1 = phi - alpha;
    else
        % 红色臂（左侧）使用加法
        q1 = phi + alpha;
    end
    
    % 7. 计算关节3角度
    % 使用余弦定理：cos(q) = (L1^2 + L2^2 - R^2) / (2 * L1 * L2)
    cos_q = (L1^2 + L2^2 - R^2) / (2 * L1 * L2);
    
    % 验证解的存在性
    if abs(cos_q) > 1
        error('目标点不可达: 几何约束不满足 (cos_q=%.2f)', cos_q);
    end
    
    q = acos(cos_q);
    
    % 根据机械臂颜色确定加减关系
    if strcmpi(arm_color, 'blue')
        % 蓝色臂（右侧
        q3 = pi - q;  % 关节3转动角度为pi-q
    else
        % 红色臂（左侧）
        q3 = q-pi;
    end
    % 应用关节限制
    %[q1, q2, q3] = check_joint_limits([q1, q2, q3]);
    
    % 验证解的正确性
    [calc_pos] = forward_kinematics(q1, q2, q3, L1, L2);
    if norm(calc_pos - end_pos) > 3
        error('逆运动学求解失败: 计算位置与目标位置偏差 %.2f cm', norm(calc_pos - end_pos));
    else
        fprintf('逆运动学成功: 计算位置与目标位置偏差 %.2f cm\n', norm(calc_pos - end_pos));
    end
end


function [pos] = forward_kinematics(q1, q2, q3, L1, L2)
    % 正运动学验证函数
    % 1. 计算关节2位置
    joint2_pos = [
        L1 * cos(q1) * cos(q2),
        L1 * sin(q1) * cos(q2),
        L1 * sin(q2)
    ];
    
    % 2. 计算关节3位置（与关节2位置相同）
    joint3_pos = joint2_pos;
    
    % 3. 计算总旋转矩阵
    R03 = rotz(q1) * roty(q2) * rotz(q3);
    
    % 4. 计算末端位置
    end_effector = joint3_pos + R03 * [L2; 0; 0];
    pos = end_effector';
end

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

% 修正后的正运动学函数
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
    
    % 计算关节2位置
    joint2_pos_a = [
        L1 * cos(theta1a) * cos(theta2a),
        L1 * sin(theta1a) * cos(theta2a),
        L1 * sin(theta2a)
    ];
    
    % 计算总旋转矩阵
    R03a = rotz(theta1a) * roty(theta2a) * rotz(theta3a);
    
    % 计算末端位置
    end_effector_a = joint2_pos_a + R03a * [L2; 0; 0];
    pos_a = end_effector_a' + baseBlue; % 添加基座偏移
    
    % 机械臂B
    theta1b = current_angles(4);
    theta2b = current_angles(5);
    theta3b = current_angles(6);
    
    % 计算关节2位置
    joint2_pos_b = [
        L1 * cos(theta1b) * cos(theta2b),
        L1 * sin(theta1b) * cos(theta2b),
        L1 * sin(theta2b)
    ];
    
    % 计算总旋转矩阵
    R03b = rotz(theta1b) * roty(theta2b) * rotz(theta3b);
    
    % 计算末端位置
    end_effector_b = joint2_pos_b + R03b * [L2; 0; 0];
    pos_b = end_effector_b' + baseRed; % 添加基座偏移
end

% 保留旋转矩阵函数
function R = rotz(theta)
    R = [cos(theta), -sin(theta), 0;
         sin(theta),  cos(theta), 0;
         0, 0, 1];
end

function R = roty(theta)
    theta = - theta;
    R = [cos(theta), 0, sin(theta);
         0, 1, 0;
         -sin(theta), 0, cos(theta)];
end