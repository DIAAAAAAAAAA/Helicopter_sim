%% 1. Getting model parameters
p = getpvec(nl_sys_estimated); 

%% 2. Solving equilibrium point (x0, u0)
% dx = 0 and theta = 0
% Define an optimization goal，using current state_space_function
objective = @(p_in) [state_space_function(0, p_in(1:4), p_in(5), ...
    p(1), p(2), p(3), p(4), p(5), p(6), p(7), p(8), p(9), p(10)); p_in(1)];

% Setting initial guess
x0_guess = [0; 0; 0; 0; 0]; 

% Solving the equilibrium point
options = optimoptions('fsolve', 'Display', 'off');
[sol, ~, exitflag] = fsolve(objective, x0_guess, options);

x0 = sol(1:4); % Equilibrium state
u0 = sol(5);   % Balancing voltage

fprintf('Equilibrium point solving finished：\n');
fprintf('Theta: %.4f, Theta_dot: %.4f, I: %.4f, w: %.4f\n', x0(1), x0(2), x0(3), x0(4));
fprintf('Voltage needed u0: %.4f\n', u0);

%% 3. linearization (Constructing A and B)

h = 1e-4; % differential step length
A = zeros(4,4);
B = zeros(4,1);

% Differential calculation of A matrix
for i = 1:4
    x_plus = x0; x_plus(i) = x_plus(i) + h;
    x_minus = x0; x_minus(i) = x_minus(i) - h;
    [dx_p, ~] = state_space_function(0, x_plus, u0, p(1), p(2), p(3), p(4), p(5), p(6), p(7), p(8), p(9), p(10));
    [dx_m, ~] = state_space_function(0, x_minus, u0, p(1), p(2), p(3), p(4), p(5), p(6), p(7), p(8), p(9), p(10));
    A(:, i) = (dx_p - dx_m) / (2*h);
end

%Differential calculation of B matrix
[dx_p, ~] = state_space_function(0, x0, u0 + h, p(1), p(2), p(3), p(4), p(5), p(6), p(7), p(8), p(9), p(10));
[dx_m, ~] = state_space_function(0, x0, u0 - h, p(1), p(2), p(3), p(4), p(5), p(6), p(7), p(8), p(9), p(10));
B = (dx_p - dx_m) / (2*h);

C = [1 0 0 0]; % Output is theta
D = 0;

sys_linear = ss(A, B, C, D);
disp('Linearization of system success！');

figure
pzplot(sys_linear)

%% 4. LQG Tracker 设计 (基于 PPT 三个步骤)

% --- Step 1: LQI (Tracker) 设计 ---
% 增广系统: [x_dot; x_i_dot] = [A, 0; -C, 0] * [x; x_i] + [B; 0] * u + [0; 1] * r
% 积分器 dynamics: dx_i = r - y = r - Cx
A_aug = [A, zeros(4,1); -C, 0];
B_aug = [B; 0];

% 权重矩阵: Q_aug 惩罚 [x; x_i], R_lqr 惩罚 u
% 注意：根据你的系统，Q_aug 最后一个元素是积分器权重，越大追踪越快
Q_aug = diag([100, 50, 0.1, 0.1, 500]); 
R_lqr = 1;
K_c = lqr(A_aug, B_aug, Q_aug, R_lqr);

Kx = K_c(1:4); % 状态反馈增益
Ki = K_c(5);   % 积分器增益

% --- Step 2: LQE 设计 ---
% 观测器增益 Kf (即你之前的 L)
Qn = diag([0.01, 0.01, 0.1, 0.1]); 
Rn = 0.001; 
Kf = lqe(A, eye(4), C, Qn, Rn);

% --- Step 3: Tracker Implementation (构建 PPT 中的控制器动态) ---
% 控制器输入向量: [e; ym] 其中 e = r - y
% 根据 PPT 公式 46:
% d(x_hat_dot) = (A - B*Kx - Kf*C) * x_hat - B*Ki * x_i + Kf * ym
% d(x_i_dot)   = e (注意: PPT 中 x_i_dot = e, 且 y_m = C*x)
% 简化实现: 将控制器作为 ss 对象
Ac_ctrl = [A - B*Kx - Kf*C, -B*Ki; 
           zeros(1, 4),     0];
Bc_ctrl = [zeros(4,1), Kf;   % 第一行对应 \dot{x}_hat，输入是 [e, ym]
           1,          0];  
Cc_ctrl = [-Kx, -Ki];       % u = -Kx*x_hat - Ki*x_i
Dc_ctrl = [zeros(1,1), zeros(1,1)]; 

LQG_Tracker = ss(Ac_ctrl, Bc_ctrl, Cc_ctrl, Dc_ctrl);

%% 6. 闭环稳定性验证
% 构造闭环系统: r -> y
% 闭环结构由 [Plant] 和 [Controller] 组成，控制器输入为 [e; ym]，其中 e = r - y
% 我们可以利用 MATLAB 的 connect 函数自动处理复杂的内部连接

% 1. 定义系统组件
sys_plant = ss(A, B, C, 0);
sys_ctrl = LQG_Tracker;

% 2. 定义连接关系
% 定义信号名称:
% 'r' 为参考输入, 'u' 为控制输入, 'y' 为物理输出, 'e' 为误差, 'ym' 为测量值
% 连接逻辑: e = r - y; ym = y; u = sys_ctrl * [e; ym]
sys_plant.InputName = 'u';
sys_plant.OutputName = 'y';
sys_ctrl.InputName = {'e', 'ym'};
sys_ctrl.OutputName = 'u';

% 3. 定义求和点 (Summing junction: e = r - y)
sum_node = sumblk('e = r - y');
% 定义测量点 (ym = y)
meas_node = sumblk('ym = y');

% 4. 使用 connect 函数自动构建闭环
sys_cl = connect(sys_plant, sys_ctrl, sum_node, meas_node, 'r', 'y');

% 5. 验证稳定性
cl_poles = eig(sys_cl);
disp('闭环系统极点:');
disp(cl_poles);

if all(real(cl_poles) < 0)
    disp('验证通过: 闭环系统所有极点均在左半平面，系统稳定！');
else
    disp('验证失败: 闭环系统包含不稳定极点，请调整 Q_aug 或 R_lqr。');
    % 绘制极点分布以便观察
    figure;
    pzmap(sys_cl);
    grid on;
    title('不稳定极点分布图');
end

% 6. 阶跃响应测试
figure;
step(sys_cl);
title('LQG Tracker 阶跃追踪响应 (验证)');
grid on;


%% 7. 控制器离散化与闭环验证
Ts = 0.01; % 设定采样时间 (根据你的系统动态调整，例如 10ms)

% 1. 离散化控制器 (使用 Tustin 变换以保持频率特性)
LQG_Tracker_d = c2d(LQG_Tracker, Ts, 'tustin');

% 2. 离散化物理模型
sys_plant = ss(A, B, C, 0);
sys_plant_d = c2d(sys_plant, Ts, 'zoh');

%% 3. 构建闭环系统 (离散域)
% 明确输入输出关系
sys_plant_d.InputName = 'u';
sys_plant_d.OutputName = 'y';

% 控制器有两个输入：'e' 和 'ym'，一个输出：'u'
LQG_Tracker_d.InputName = {'e', 'ym'};
LQG_Tracker_d.OutputName = 'u';

% 定义 sumblk 处理信号流：e = r - y 和 ym = y
% 这完全对应你 PPT Figure 12 的逻辑
sum_node = sumblk('e = r - y');
meas_node = sumblk('ym = y');

% 自动连接系统
sys_cl_d = connect(sys_plant_d, LQG_Tracker_d, sum_node, meas_node, 'r', 'y');

% 4. 离散域稳定性验证 (确保模长 < 1)
d_poles = eig(sys_cl_d);
pole_magnitudes = abs(d_poles);

disp('离散闭环系统极点 (模长):');
disp(pole_magnitudes);

if all(pole_magnitudes < 0.999) % 加一个微小的裕度确保稳定
    disp('验证通过: 离散闭环系统所有极点模长 < 1，系统稳定！');
else
    disp('验证失败: 存在不稳定极点 (模长 >= 1)。');
    figure;
    pzmap(sys_cl_d);
    grid on;
    title('离散闭环系统极点分布');
end



