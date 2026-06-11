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


%%
% --- 连续时间 MPC 设计 ---
mpc_cont = mpc(sys_linear, Ts);
mpc_cont.PredictionHorizon = 50; 
mpc_cont.ControlHorizon = 5;

% --- 仿真验证 ---
T_sim = 2;
r = 0.2 * ones(T_sim/0.01, 1);
[y_c, t_c, u_c] = sim(mpc_cont, length(r), r);

figure('Name', '连续时间 MPC 性能');
subplot(2,1,1); plot(t_c, y_c); title('连续系统输出'); ylabel('Theta');
subplot(2,1,2); plot(t_c, u_c); title('连续系统控制量'); xlabel('s');

% 1. 显式离散化
Ts = 0.01;
sys_d = c2d(sys_linear, Ts, 'zoh');

% 2. 构建离散 MPC 并显式设置参数 (消除 Warning)
mpc_d = mpc(sys_d, Ts);
mpc_d.PredictionHorizon = 50; 
mpc_d.ControlHorizon = 5;
mpc_d.Weights.OutputVariables = 100;
mpc_d.Weights.ManipulatedVariablesRate = 0.1;

% 3. 修复 zplane 报错：使用更稳定的绘图方式
% 错误原因是因为直接传入 SS 对象给 zplane 内部处理逻辑有 BUG
% 我们改用 eig 计算后的极点进行绘制
poles_d = eig(sys_d.A); 
zeros_d = eig(sys_d.A - sys_d.B*sys_d.C); % 简单的零点估计

figure('Name', '离散系统稳定性验证');
zplane(zeros_d, poles_d); % 直接绘制极点坐标，避开系统对象转换错误
title('离散系统的零极点图');
grid on;

% 4. 离散仿真验证 (使用 r 变量)
[y_d, t_d, u_d] = sim(mpc_d, length(r), r);
figure('Name', '离散 MPC 仿真对比');
stairs(t_d, y_d, 'LineWidth', 1.5); hold on;
stairs(t_d, r, 'r--');
legend('离散 MPC 输出', '目标值');
title('离散域阶跃响应');

