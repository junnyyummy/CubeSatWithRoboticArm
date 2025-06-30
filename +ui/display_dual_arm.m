function display_dual_arm(targetsA, targetsB, L1, L2, boxLength, boxDepth, boxHeight, thickness)
    % 初始化设置
    figure('Name', 'Dual Arm Trajectory Planning', 'NumberTitle', 'off', 'Color', 'k');
    axis equal;
    axis([-100 100 -100 100 -100 100]);
    xlabel('X (cm)'); ylabel('Y (cm)'); zlabel('Z (cm)');
    set(gca, 'Color', 'k', 'GridColor', [0.4 0.4 0.4], 'XColor', 'w', 'YColor', 'w', 'ZColor', 'w');
    grid on; hold on; view(35, 25);
    
    % 添加光源
    light('Position', [0 0 100], 'Style', 'infinite', 'Color', [1 1 0.8]);
    light('Position', [50 50 50], 'Style', 'infinite', 'Color', [0.8 0.8 1]);
    lighting gouraud;
    
    % 关节和连杆参数
    jointRadius = 0.8; % 增大关节半径 (直径1.6cm)
    linkWidth = 0.8;   % 增大连杆截面宽度 (cm)

    % 基座位置
    baseBlue = [2*thickness, 0, 2*thickness];                 % 蓝臂左下角靠里
    baseRed = [boxLength - 2*thickness, 0, boxHeight - 2*thickness];  % 红臂右上角靠里

    % 绘制工作区域盒子
    draw_box(boxLength, boxDepth, boxHeight);
    
    % 初始关节角度（尝试从文件加载或使用零值）
    if exist('current_arm_state.mat', 'file') == 2
        load('current_arm_state.mat', 'final_angles');
        current_angles = final_angles;
    else
        current_angles = zeros(1, 6);
    end
    
    % 初始化轨迹数据
    all_joint_angles = [];
    all_end_positions = [];
    total_time_vector = [];
    
    % 遍历所有目标点
    for point_idx = 1:size(targetsA, 1)
        % 当前目标点
        targetA = targetsA(point_idx, :);
        targetB = targetsB(point_idx, :);
        
        % 执行轨迹规划
        [time_vector, joint_angles, end_positions] = trajectory.trajectory_generator(...
            current_angles, targetA, targetB);
        
        % 确保 time_vector 是列向量
        time_vector = time_vector(:);
        
        % 累积轨迹数据
        if isempty(all_joint_angles)
            all_joint_angles = joint_angles;
            all_end_positions = end_positions;
            total_time_vector = time_vector;
        else
            % 调整时间向量（累加）
            time_offset = total_time_vector(end);
            time_vector = time_vector + time_offset;
            
            % 累积数据
            all_joint_angles = [all_joint_angles; joint_angles];
            all_end_positions = [all_end_positions; end_positions];
            total_time_vector = [total_time_vector; time_vector];
        end
        
        % 更新当前角度
        current_angles = joint_angles(end, :);
    end
    
    % 保存最终状态
    final_angles = all_joint_angles(end, :);
    end_positions = all_end_positions;
    save('current_arm_state.mat', 'final_angles', 'end_positions');
    
    % 动画循环
    numFrames = length(total_time_vector);
    for t = 1:numFrames
        cla;
        draw_box(boxLength, boxDepth, boxHeight);
        
        % 绘制轨迹路径
        plot3(all_end_positions(1:t,1), all_end_positions(1:t,2), all_end_positions(1:t,3), 'b-', 'LineWidth', 1.5);
        plot3(all_end_positions(1:t,4), all_end_positions(1:t,5), all_end_positions(1:t,6), 'r-', 'LineWidth', 1.5);
        
        % 提取当前关节角度
        angles = all_joint_angles(t, :);
        
        % ==== 红色机械臂 (B) ====
        [joints_red, R_red] = calculate_arm_positions(baseRed, angles(4), angles(5), angles(6), L1, L2, 0);
        draw_arm(joints_red, R_red, linkWidth, jointRadius, [1 0.3 0.2], L2);
        
        % ==== 蓝色机械臂 (A) ====
        [joints_blue, R_blue] = calculate_arm_positions(baseBlue, angles(1), angles(2), angles(3), L1, L2, 0);
        draw_arm(joints_blue, R_blue, linkWidth, jointRadius, [0.2 0.4 1], L2);
        
        % 显示信息
        current_target_idx = find(t <= cumsum(size(targetsA, 1) * ones(1, numFrames), 1));
        if isempty(current_target_idx)
            current_target_idx = size(targetsA, 1);
        end
        
        title_str = sprintf('Target %d/%d | Time: %.1fs/%.1fs | A: (%.1f, %.1f, %.1f) | B: (%.1f, %.1f, %.1f)', ...
            current_target_idx, size(targetsA, 1), ...
            total_time_vector(t), total_time_vector(end), ...
            all_end_positions(t,1), all_end_positions(t,2), all_end_positions(t,3), ...
            all_end_positions(t,4), all_end_positions(t,5), all_end_positions(t,6));
        
        title(title_str, 'Color', 'w', 'FontSize', 10);
        drawnow;
        
        % 控制动画速度
        pause(0.02);
    end
    
    % 显示最终位置误差
    final_error_a = norm(all_end_positions(end,1:3) - targetsA(end,:));
    final_error_b = norm(all_end_positions(end,4:6) - targetsB(end,:));
    
    fprintf('轨迹完成:\n');
    fprintf('  机械臂A末端误差: %.4f cm\n', final_error_a);
    fprintf('  机械臂B末端误差: %.4f cm\n', final_error_b);
    
    % ==== 生成SVG和CSV文件 ====
    numFrames = length(total_time_vector);
    timestamp = datestr(now, 'yyyymmdd_HHMMSS');
    svg_filename = sprintf('joint_angles_%s.svg', timestamp);
    csv_filename = sprintf('joint_angles_%s.csv', timestamp);
    
    fprintf('正在将关节角度序列保存到文件:\n');
    fprintf('  SVG: %s\n  CSV: %s\n', svg_filename, csv_filename);
    
    % 首先保存CSV文件
    csvwrite(csv_filename, all_joint_angles);
    
    % 生成SVG文件
    try
        % 计算所需高度
        svg_height = numFrames * 20 + 150;
        
        % 创建并写入SVG文件
        fid = fopen(svg_filename, 'w');
        if fid == -1
            error('无法创建SVG文件');
        end
        
        % 写入XML头部
        fprintf(fid, '<?xml version="1.0" encoding="UTF-8" standalone="no"?>\n');
        fprintf(fid, '<!DOCTYPE svg PUBLIC "-//W3C//DTD SVG 1.1//EN" ');
        fprintf(fid, '"http://www.w3.org/Graphics/SVG/1.1/DTD/svg11.dtd">\n');
        fprintf(fid, '<svg width="1200" height="%d" ', svg_height);
        fprintf(fid, 'xmlns="http://www.w3.org/2000/svg" version="1.1">\n');
        fprintf(fid, '<style>text { font-family: monospace; }</style>\n');
        
        % 背景
        fprintf(fid, '<rect width=''100%%'' height=''100%%'' fill=''white''/>\n');
        
        fprintf(fid, '<text x="20" y="30" font-size="14" font-weight="bold">关节角度序列 (共%d个点)</text>\n', numFrames);
        fprintf(fid, '<text x="20" y="50" font-size="12">时间: %.2fs - %.2fs</text>\n', ...
                total_time_vector(1), total_time_vector(end));
        fprintf(fid, '<text x="20" y="70" font-size="10">索引   θ1A       θ2A       θ3A       θ1B       θ2B       θ3B</text>\n');
        
        % 写入数据行
        y_pos = 90;
        for i = 1:numFrames
            angles = all_joint_angles(i, :);
            fprintf(fid, '<text x="20" y="%d" font-size="10">%05d  %8.4f  %8.4f  %8.4f  %8.4f  %8.4f  %8.4f</text>\n', ...
                    y_pos, i, angles(1), angles(2), angles(3), angles(4), angles(5), angles(6));
            y_pos = y_pos + 20;
        end
        
        % 添加文件信息
        fprintf(fid, '<text x="20" y="%d" font-size="10" font-weight="bold">文件: %s</text>\n', y_pos + 20, svg_filename);
        fprintf(fid, '</svg>');
        fclose(fid);
        
        fprintf('SVG文件已成功生成\n');
    catch ME
        fprintf('生成SVG时出错: %s\n', ME.message);
        fprintf('已生成CSV文件作为替代\n');
    end
end

function draw_box(boxLength, boxDepth, boxHeight)
% 绘制机械臂盒子（除正面外）- 使用深色方案
% 顶点定义
vertices = [...
    0 0 0;
    boxLength 0 0;
    boxLength boxDepth 0;
    0 boxDepth 0;
    0 0 boxHeight;
    boxLength 0 boxHeight;
    boxLength boxDepth boxHeight;
    0 boxDepth boxHeight];

% 底面 - 深灰色带网格
patch('Vertices', vertices, 'Faces', [1 2 3 4], ...
      'FaceColor', [0.2 0.2 0.2], 'FaceAlpha', 0.9, 'EdgeColor', [0.4 0.4 0.4], ...
      'LineWidth', 1, 'AmbientStrength', 0.5, 'DiffuseStrength', 0.7);
  
% 顶面 - 深灰色
patch('Vertices', vertices, 'Faces', [5 6 7 8], ...
      'FaceColor', [0.25 0.25 0.25], 'FaceAlpha', 0.8, 'EdgeColor', 'none', ...
      'AmbientStrength', 0.4, 'DiffuseStrength', 0.6);

% 背面 - 纯黑色
patch('Vertices', vertices, 'Faces', [3 4 8 7], ...
      'FaceColor', 'k', 'FaceAlpha', 0.95, 'EdgeColor', [0.3 0.3 0.3], ...
      'LineWidth', 1, 'AmbientStrength', 0.3, 'DiffuseStrength', 0.4);
  
% 左侧面 - 深灰色
patch('Vertices', vertices, 'Faces', [1 4 8 5], ...
      'FaceColor', [0.15 0.15 0.15], 'FaceAlpha', 0.85, 'EdgeColor', 'none', ...
      'AmbientStrength', 0.5, 'DiffuseStrength', 0.7);
  
% 右侧面 - 深灰色
patch('Vertices', vertices, 'Faces', [2 3 7 6], ...
      'FaceColor', [0.15 0.15 0.15], 'FaceAlpha', 0.85, 'EdgeColor', 'none', ...
      'AmbientStrength', 0.5, 'DiffuseStrength', 0.7);
end

function [joints, R_total] = calculate_arm_positions(base, q1, q2, q3, L1, L2, z_offset)
% 计算机械臂关节位置和旋转矩阵
joint1 = base;
joint2 = base + [0, 0, z_offset];  % 垂直移动

% 累积旋转矩阵
R1 = rotz(q1);
R2 = R1 * roty(q2);
R_total = R2 * rotz(q3);  % 完整旋转矩阵

% 关节位置计算
joint3 = joint2 + (R2 * [L1; 0; 0])';
joint4 = joint3 + (R_total * [L2; 0; 0])';

joints = [joint1; joint2; joint3; joint4];
end

function draw_arm(joints, R_total, linkWidth, jointRadius, color, L2)
% 绘制单条机械臂
% 绘制关节
for i = 1:4
    draw_joint(joints(i,:), jointRadius, [1,1,0]);
end

% 绘制连杆 - 添加边缘增强可视性
draw_link(joints(1,:), joints(2,:), linkWidth, color, true);  % 垂直连杆
draw_link(joints(2,:), joints(3,:), linkWidth, color, true);  % L1
draw_link(joints(3,:), joints(4,:), linkWidth, color, true);  % L2

% 绘制末端执行器(夹爪)
draw_effector(joints(4,:), R_total, L2/4, color);
end

function draw_joint(center, radius, color)
% 绘制球形关节
[x,y,z] = sphere(24);
surf(radius*x + center(1), radius*y + center(2), radius*z + center(3), ...
    'FaceColor', color, 'EdgeColor', 'none', 'FaceLighting', 'gouraud', ...
    'AmbientStrength', 0.3, 'DiffuseStrength', 0.8, 'SpecularStrength', 1.0, ...
    'SpecularExponent', 20, 'SpecularColorReflectance', 0.8);
end

function draw_link(p1, p2, width, color, showEdges)
% 绘制方形截面连杆
v = p2 - p1;
len = norm(v);
if len < eps, return; end % 避免零长度连杆

% 创建局部坐标系
z_axis = [0, 0, 1];
x_axis = v/len;
if abs(dot(x_axis, z_axis)) > 0.99
    y_axis = [0, 1, 0];
else
    y_axis = cross(z_axis, x_axis);
    y_axis = y_axis/norm(y_axis);
end
z_axis = cross(x_axis, y_axis);

% 构建长方体顶点
w = width/2;
vertices = [...
    p1 + (-w*y_axis - w*z_axis);
    p1 + ( w*y_axis - w*z_axis);
    p1 + ( w*y_axis + w*z_axis);
    p1 + (-w*y_axis + w*z_axis);
    p2 + (-w*y_axis - w*z_axis);
    p2 + ( w*y_axis - w*z_axis);
    p2 + ( w*y_axis + w*z_axis);
    p2 + (-w*y_axis + w*z_axis)];

% 定义长方体面
faces = [...
    1,2,3,4;    % 起点端面
    5,6,7,8;    % 终点端面
    1,2,6,5;    % 底面
    2,3,7,6;    % 侧面1
    3,4,8,7;    % 顶面
    4,1,5,8];   % 侧面2

% 添加边缘增强可视性
edgeColor = 'none';
% if showEdges
%     edgeColor = [0.2 0.2 0.2];
% end

patch('Vertices', vertices, 'Faces', faces, ...
      'FaceColor', color, 'EdgeColor', edgeColor, 'LineWidth', 1.2, ...
      'FaceAlpha', 0.95, 'AmbientStrength', 0.4, 'DiffuseStrength', 0.8, ...
      'SpecularStrength', 0.6, 'SpecularExponent', 15);
end

function draw_effector(center, R, size, color)
% 绘制末端执行器（三指夹爪）
% 主夹爪方向（使用总旋转矩阵确定方向）
grip_dir = R(:,1)';  % X轴方向

% 绘制夹爪臂
grip_width = size/4;
draw_cone(center, grip_dir, size*0.3, grip_width*1.2, [0.9 0.9 0.3]);
end

function draw_cone(base, direction, height, radius, color)
% 绘制圆锥体
direction = direction / norm(direction);
[x,y,z] = cylinder([radius, 0], 12);
z = z * height;

% 旋转圆锥体指向正确方向
z_axis = [0,0,1];
if norm(cross(z_axis, direction)) > 1e-3
    rot_axis = cross(z_axis, direction);
    rot_angle = acos(dot(z_axis, direction));
    R = axang2rotm([rot_axis/norm(rot_axis), rot_angle]);
else
    R = eye(3);
end

% 应用旋转和平移
for i = 1:numel(x)
    pt = R * [x(i); y(i); z(i)];
    x(i) = pt(1) + base(1);
    y(i) = pt(2) + base(2);
    z(i) = pt(3) + base(3);
end

surf(x, y, z, 'FaceColor', color, 'EdgeColor', 'none', ...
     'FaceLighting', 'gouraud', 'SpecularStrength', 0.8);
end

function R = axang2rotm(axang)
% 轴角转换为旋转矩阵
theta = axang(4);
u = axang(1:3);
u = u / norm(u);

% 叉积矩阵
ux = [0 -u(3) u(2); u(3) 0 -u(1); -u(2) u(1) 0];

% 罗德里格斯公式
R = cos(theta)*eye(3) + sin(theta)*ux + (1-cos(theta))*(u'*u);
end

function R = rotz(theta)
% 绕Z轴旋转
R = [cos(theta), -sin(theta), 0;
     sin(theta),  cos(theta), 0;
     0,           0,          1];
end

function R = roty(theta)
% 绕Y轴旋转
theta = -theta;
R = [cos(theta),  0, sin(theta);
     0,           1, 0;
    -sin(theta),  0, cos(theta)];
end