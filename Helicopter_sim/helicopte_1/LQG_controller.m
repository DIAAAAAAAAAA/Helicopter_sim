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

%% 4. LQG Tracker Design (Based on the Three Steps in the PPT)

% --- Step 1: LQI (Tracker) Design ---
% Augmented system: [x_dot; x_i_dot] = [A, 0; -C, 0] * [x; x_i] + [B; 0] * u + [0; 1] * r
% Integrator dynamics: dx_i = r - y = r - Cx
A_aug = [A, zeros(4,1); -C, 0];
B_aug = [B; 0];

% Weight matrices:
Q_aug = diag([100, 50, 0.1, 0.1, 500]); 
R_lqr = 1;
K_c = lqr(A_aug, B_aug, Q_aug, R_lqr);

Kx = K_c(1:4); % State feedback gain
Ki = K_c(5);   % Integrator gain

% --- Step 2: LQE Design ---
% Observer gain Kf (equivalent to the previous L)
Qn = diag([0.01, 0.01, 0.1, 0.1]); 
Rn = 0.001; 
Kf = lqe(A, eye(4), C, Qn, Rn);

% --- Step 3: Tracker Implementation (Construct Controller Dynamics from PPT) ---
Ac_ctrl = [A - B*Kx - Kf*C, -B*Ki; 
           zeros(1, 4),     0];
Bc_ctrl = [zeros(4,1), Kf;   % 第一行对应 \dot{x}_hat，输入是 [e, ym]
           1,          0];  
Cc_ctrl = [-Kx, -Ki];       % u = -Kx*x_hat - Ki*x_i
Dc_ctrl = [zeros(1,1), zeros(1,1)]; 

LQG_Tracker = ss(Ac_ctrl, Bc_ctrl, Cc_ctrl, Dc_ctrl);

%% 6. Closed-Loop Stability Verification

% 1. Define system components
sys_plant = ss(A, B, C, 0);
sys_ctrl = LQG_Tracker;

% 2. Define interconnections
% Signal names:
% 'r'  = reference input
% 'u'  = control input
% 'y'  = plant output
% 'e'  = tracking error
% 'ym' = measured output
% Connection logic:
% e = r - y; ym = y; u = sys_ctrl * [e; ym]
sys_plant.InputName = 'u';
sys_plant.OutputName = 'y';
sys_ctrl.InputName = {'e', 'ym'};
sys_ctrl.OutputName = 'u';

% 3. Define summing junction (e = r - y)
sum_node = sumblk('e = r - y');
% Define measurement signal (ym = y)
meas_node = sumblk('ym = y');

% 4. Use connect to automatically build the closed-loop system
sys_cl = connect(sys_plant, sys_ctrl, sum_node, meas_node, 'r', 'y');

% 5. Verify stability
cl_poles = eig(sys_cl);
disp('Closed-loop system poles:');
disp(cl_poles);

if all(real(cl_poles) < 0)
    disp('Verification passed: All closed-loop poles are in the left-half plane. The system is stable!');
else
    disp('Verification failed: The closed-loop system contains unstable poles. Please adjust Q_aug or R_lqr.');
    % Plot pole locations for inspection
    figure;
    pzmap(sys_cl);
    grid on;
    title('Unstable Pole Distribution');
end

% 6. Step response test
figure;
step(sys_cl);
title('LQG Tracker Step Tracking Response (Verification)');
grid on;


%% 7. Controller Discretization and Closed-Loop Verification
Ts = 0.01; % Sampling time (adjust according to system dynamics, e.g., 10 ms)

% 1. Discretize the controller (Tustin transformation preserves frequency characteristics)
LQG_Tracker_d = c2d(LQG_Tracker, Ts, 'tustin');

% 2. Discretize the plant model
sys_plant = ss(A, B, C, 0);
sys_plant_d = c2d(sys_plant, Ts, 'zoh');


%% 3. Construct Closed-Loop System (Discrete-Time Domain)
% Explicitly define input-output relationships
sys_plant_d.InputName = 'u';
sys_plant_d.OutputName = 'y';

% The controller has two inputs: 'e' and 'ym', and one output: 'u'
LQG_Tracker_d.InputName = {'e', 'ym'};
LQG_Tracker_d.OutputName = 'u';

% Define sumblk for signal flow:
% e = r - y and ym = y
% This exactly matches the logic in PPT Figure 12
sum_node = sumblk('e = r - y');
meas_node = sumblk('ym = y');

% Automatically connect the systems
sys_cl_d = connect(sys_plant_d, LQG_Tracker_d, sum_node, meas_node, 'r', 'y');

% 4. Discrete-time stability verification (all pole magnitudes must be < 1)
d_poles = eig(sys_cl_d);
pole_magnitudes = abs(d_poles);

disp('Discrete closed-loop system pole magnitudes:');
disp(pole_magnitudes);

if all(pole_magnitudes < 0.999) % Small margin for robust stability
    disp('Verification passed: All discrete closed-loop poles have magnitude < 1. The system is stable!');
else
    disp('Verification failed: Unstable poles detected (magnitude >= 1).');
    figure;
    pzmap(sys_cl_d);
    grid on;
    title('Discrete Closed-Loop Pole Distribution');
end
