%% 第二问：轨迹跟踪
clear; clc; close all

% 车辆参数
lfr = 2.168 + 1.907; % 轴距 L
dt = 0.01;
v = 15; 
sim_steps = 2000;

% 参考轨迹 (正弦曲线)
X_ref = 0:0.1:200; 
Y_ref = 10 * sin(X_ref / 15); 

% 初始车辆状态 
X = X_ref(1); Y = Y_ref(1) + 3; phi = 0; 
X_vec = zeros(1, sim_steps); Y_vec = zeros(1, sim_steps);


for ii = 1:sim_steps
    X_vec(ii) = X; Y_vec(ii) = Y;
    
    
    % ===============================================================
    
    % ================= TODO 2.1: 实现某种跟踪算法 =================
    
    % 纯追踪法 sigma = arctan(2 * L * sin(alpha) / Ld)
    Ld = 5; % 前瞻距离 （经过我的尝试，Ld = 0.2时轨迹最贴合参考轨迹，5<=Ld<=8时，轨迹起始稍有偏差，后面依然贴合参考轨迹）

    % 1. 找到离车辆最近的参考点索引
    distances = sqrt((X_ref - X).^2 + (Y_ref - Y).^2);
    [~, nearest_idx] = min(distances);

    % 2. 从最近点向前搜索，找到距离>=Ld的前视点
    target_idx = nearest_idx;
    for j = nearest_idx:length(X_ref)
        if sqrt((X_ref(j) - X)^2 + (Y_ref(j) - Y)^2) >= Ld
            target_idx = j;
            break;
        end
    end

    X_target = X_ref(target_idx);
    Y_target = Y_ref(target_idx);
    
    % 3. 计算航向偏差alpha和转向角sigma
    alpha = atan2(Y_target - Y, X_target - X) - phi;

    alpha = atan2(sin(alpha), cos(alpha)); % 把超过[-pi,pi]范围的角度转化为范围内的角度

    sigma = atan2(2 * lfr * sin(alpha), Ld);

    % ===============================================================

    % ================= TODO 2.2: 车辆状态更新 =================
    % 提示: 将刚才求得的转向角 sigma 代入运动学模型（复用第一问代码），更新 X, Y, phi。
    
    phi_dot = v * tan(sigma) / lfr;
    phi = phi + phi_dot * dt;
    X = X + v * cos(phi) * dt;
    Y = Y + v * sin(phi) * dt;
    
    % ===============================================================
    
    % 到达终点提前结束
    if X >= X_ref(end), break; end
end

% 绘图对比
figure; hold on; grid on;
plot(X_ref, Y_ref, 'k--', 'LineWidth', 2);
plot(X_vec(1:ii), Y_vec(1:ii), 'r-', 'LineWidth', 2);
legend('参考规划轨迹', '实际行驶轨迹');
title(['Pure Pursuit 跟踪 (Ld = ', num2str(Ld), 'm)']);
xlabel('X [m]'); ylabel('Y [m]'); axis equal;