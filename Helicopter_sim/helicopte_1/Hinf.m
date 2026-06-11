%% ============================================
% H-infinity controller design for helicopter pitch
% Uses your linearized model A,B,C,D around hover
%% ============================================
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


% Linearized plant
G = ss(A,B,C,0);
G = minreal(G);

figure;
pzmap(G); grid on;
title('Linearized Plant');

% Check controllability / observability
Co = ctrb(A,B);
Ob = obsv(A,C);

disp(['Controllability rank = ', num2str(rank(Co))]);
disp(['Observability rank   = ', num2str(rank(Ob))]);

% Weight selection
s = tf('s');

M  = 1.5;
wb = 4;       % target bandwidth [rad/s]
Aperf = 1e-3; % low-frequency tracking requirement

W1 = (s/M + wb)/(s + wb*Aperf);  % tracking / disturbance rejection
W2 = 2.5;                        % control activity penalty
W3 = makeweight(0.01, 30, 10);   % high-frequency robustness/noise attenuation

% Synthesize H-infinity controllersss
[K_hinf, CL, gamma] = mixsyn(G, W1, W2, W3);
K_hinf = minreal(K_hinf);

disp('-------------------------------------');
disp('H-infinity controller synthesis done');
disp(['gamma = ', num2str(gamma)]);
disp('-------------------------------------');

% Closed-loop sensitivity
L = G*K_hinf;
S = feedback(1, L);
T = feedback(L, 1);

figure;
sigma(S, T, {1e-2, 1e2});
grid on;
legend('S','T');
title('Sensitivity Functions');

figure;
step(feedback(G*K_hinf,1), 5);
grid on;
title('Closed-loop Step Response');

figure;
margin(L);
grid on;
title('Loop Margin');

% Closed-loop poles
sys_cl_hinf = feedback(G*K_hinf, 1);
disp('Closed-loop poles:');
disp(pole(sys_cl_hinf));

if isstable(sys_cl_hinf)
    disp('Continuous-time H-infinity closed-loop is stable.');
else
    disp('Continuous-time H-infinity closed-loop is NOT stable.');
end

% Discretize controller
Ts = 0.01;
K_hinf_d = c2d(K_hinf, Ts, 'tustin');
K_hinf_d = minreal(K_hinf_d);

disp('Discrete-time H-infinity controller:');
K_hinf_d

[Ak,Bk,Ck,Dk] = ssdata(K_hinf_d);

disp('Discrete controller matrices:');
disp('Ak ='); disp(Ak);
disp('Bk ='); disp(Bk);
disp('Ck ='); disp(Ck);
disp('Dk ='); disp(Dk);

%% Discrete closed-loop verification
Gd = c2d(G, Ts, 'zoh');
sys_cl_d = feedback(Gd*K_hinf_d, 1);

disp('Discrete closed-loop poles:');
disp(pole(sys_cl_d));

if all(abs(pole(sys_cl_d)) < 1)
    disp('Discrete H-infinity closed-loop is stable.');
else
    disp('Discrete H-infinity closed-loop is NOT stable.');
end

figure;
step(feedback(G*K_hinf,1), sys_cl_d, 5);
grid on;
legend('Continuous CL','Discrete CL');
title('Continuous vs Discrete Closed-Loop Step Response');

figure;
bode(feedback(G*K_hinf,1), sys_cl_d);
grid on;
legend('Continuous CL','Discrete CL');
title('Continuous vs Discrete Closed-Loop Frequency Response');