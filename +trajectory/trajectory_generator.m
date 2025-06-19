function [q1a, q2a, q3a, q1b, q2b, q3b] = trajectory_generator(t)
    % 机械臂轨迹角度（单位：弧度）
    q1a = deg2rad(30 * sin(0.05 * t));
    q2a = deg2rad(45 * sin(0.07 * t));
    q3a = deg2rad(20 * sin(0.09 * t));

    q1b = deg2rad(30 * sin(0.05 * t + pi/4));
    q2b = deg2rad(45 * sin(0.07 * t + pi/6));
    q3b = deg2rad(20 * sin(0.09 * t + pi/3));
end