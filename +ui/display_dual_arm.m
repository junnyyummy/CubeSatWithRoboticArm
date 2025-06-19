function display_dual_arm()
    % 显示双机械臂动画

    figure('Name', 'Dual Robotic Arms', 'NumberTitle', 'off');
    axis equal;
    axis([-10 10 -10 10]);
    grid on;
    hold on;

    % 获取机械臂参数
    [L1, L2] = model.arm_parameters();

    % 帧数
    numFrames = 200;

    for t = 1:numFrames
        clf;
        axis equal;
        axis([-10 10 -10 10]);
        grid on;
        hold on;

        % 获取角度（弧度）
        [q1a, q2a, q3a, q1b, q2b, q3b] = trajectory.trajectory_generator(t);

        % ===== Arm A =====
        baseA = [0; 0];
        j1a = baseA + L1 * [cos(q1a); sin(q1a)];
        j2a = j1a + L2 * [cos(q1a + q2a); sin(q1a + q2a)];

        plot([baseA(1) j1a(1)], [baseA(2) j1a(2)], 'b-', 'LineWidth', 3);
        plot([j1a(1) j2a(1)], [j1a(2) j2a(2)], 'b--', 'LineWidth', 3);

        % ===== Arm B =====
        baseB = [5; 0];
        j1b = baseB + L1 * [cos(q1b); sin(q1b)];
        j2b = j1b + L2 * [cos(q1b + q2b); sin(q1b + q2b)];

        plot([baseB(1) j1b(1)], [baseB(2) j1b(2)], 'r-', 'LineWidth', 3);
        plot([j1b(1) j2b(1)], [j1b(2) j2b(2)], 'r--', 'LineWidth', 3);

        title(sprintf('Frame %d', t));
        pause(0.05);
    end
end