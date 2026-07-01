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

%% Feedforward calculation

sys_tf = tf(sys_linear);
gain_P = dcgain(sys_tf);
Kf = 1/gain_P;
wc = 3;

% 1. 定义线性化后的传递函数
sys_tf = tf(sys_linear);

% 2. 使用 pidtune 自动调参
% 指定响应时间 (由 wc 决定，约 1/wc) 和目标相位裕度 (通常 60 度)
% wc = 3 rad/s，则对应的目标响应时间约为 0.33s
opts = pidtuneOptions('PhaseMargin', 60);
[C_pid, info] = pidtune(sys_tf, 'PIDF', wc, opts); % PIDF 包含滤波

% 3. 查看设计的控制器参数
Kp = C_pid.Kp;
Ki = C_pid.Ki;
Kd = C_pid.Kd;
N  = C_pid.Tf; % 注意：MATLAB 中 T=1/N，这里转换一下
N_val = 1/N;

fprintf('设计的 PID 参数:\n');
fprintf('Kp = %.4f, Ki = %.4f, Kd = %.4f, N = %.4f\n', Kp, Ki, Kd, N_val);

% 假设你已经计算好了前馈增益 Kf (常数) 或前馈控制器 Cf (传递函数)
% Kf = 1/dcgain(sys_tf); % 你的前馈增益

% --- 正确的开环分析法 ---
% 1. 先验证 PID 反馈环路的稳定性 (这是根基)
sys_cl_feedback = feedback(C_pid * sys_tf, 1);
figure; step(sys_cl_feedback); title('仅 PID 反馈的响应');

% 2. 观察前馈带来的影响
% 前馈在数学上应该是：R -> [PID+FF] -> Plant
% 但在 Simulink 中，前馈通常应该避开 Sum_ref 节点
% 如果你的 Simulink 里前馈和 PID 的输出是在 Sum_ref 之后相加，代码应为：
T_total = (C_pid + sys_ff) * sys_tf / (1 + C_pid * sys_tf);

figure; step(T_total);
title('PID + 前馈 闭环响应');




