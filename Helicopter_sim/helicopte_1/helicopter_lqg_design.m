%% helicopter_lqg_design.m
% Helicopter pitch-axis LQI + Kalman filter design
% -------------------------------------------------------------------------
% This script performs the following steps for your identified grey-box model:
%   1) Load/use identified parameters
%   2) Compute an equilibrium (trim) point
%   3) Linearize the nonlinear model analytically around that trim point
%   4) Check controllability and observability
%   5) Design a continuous-time LQR regulator
%   6) Design a discrete-time LQI regulator for reference tracking
%   7) Design a discrete-time Kalman filter for state estimation
%   8) Simulate the linear closed-loop system with estimator and integral action
%   9) Provide a controller implementation structure for real-time / Simulink use
%
% IMPORTANT NOTES
% -------------------------------------------------------------------------
% 1) Your identified parameter C_T was negative in the result you shared.
%    Since your model uses T = C_T*w^2, a negative C_T is usually not physical.
%    This script DOES NOT silently change that. If the trim computation fails,
%    re-identify with physical constraints or check your sign convention.
%
% 2) The controller designed here is LOCAL. It is valid around the chosen
%    operating point theta_eq.
%
% 3) This script assumes a single-input pitch-axis model:
%       x = [theta; theta_dot; I; w]
%       y = theta
%
% 4) If only theta is measured, you need an observer because LQR/LQI uses the
%    full state vector.
%
% MATLAB toolboxes needed:
%   - Control System Toolbox
%   - System Identification Toolbox (only if loading nl_sys_estimated directly)
%
% -------------------------------------------------------------------------clear; clc; close all;

%% USER SETTINGS
% -------------------------------------------------------------------------
% Sampling frequency used in your identification dataset
Fs = 100;                  % [Hz]  <-- change if needed
Ts = 1/Fs;                 % [s]

% Desired operating point (pitch angle) for regulation / tracking
theta_eq = 0.0;            % [rad]

% Reference for closed-loop demo (constant pitch command)
theta_ref = deg2rad(5);    % [rad] example: 5 degrees

% Voltage saturation for implementation/demo
Vmax = 5;                  % [V] <-- change to your actuator limit

% Choose whether to load values from nl_sys_estimated if it exists in workspace
use_nl_sys_estimated_if_available = false;

%% KNOWN CONSTANTS FROM YOUR NONLINEAR MODEL
% -------------------------------------------------------------------------
g      = 9.81;             % [m/s^2]
m_cw   = 124e-3;           % [kg]
L_cw   = 0.02;             % [m]
L_heli = 0.035;            % [m]
L_m    = 0.3;              % [m]

%% IDENTIFIED PARAMETERS
% -------------------------------------------------------------------------
% Option A: load directly from nl_sys_estimated if available
if use_nl_sys_estimated_if_available && exist('nl_sys_estimated','var')
    Iyy     = nl_sys_estimated.Parameters(1).Value;
    m_heli  = nl_sys_estimated.Parameters(2).Value;
    C_pitch = nl_sys_estimated.Parameters(3).Value;
    T_bias  = nl_sys_estimated.Parameters(4).Value;
    c       = nl_sys_estimated.Parameters(5).Value;
    Km      = nl_sys_estimated.Parameters(6).Value;
    J       = nl_sys_estimated.Parameters(7).Value;
    L       = nl_sys_estimated.Parameters(8).Value;
    R       = nl_sys_estimated.Parameters(9).Value;
    C_T     = nl_sys_estimated.Parameters(10).Value;
else
    % Option B: use the identified values you posted
    Iyy     = 0.039857;
    m_heli  = 0.795216;
    C_pitch = 0.0039208;
    T_bias  = -0.00916309;
    c       = 0.000450553;
    Km      = 0.00232502;
    J       = 0.000131343;
    L       = 0.0148128;
    R       = 4.12861;
    C_T     = -0.266161;
end

%% DISPLAY PARAMETERS
% -------------------------------------------------------------------------
fprintf('\n================ IDENTIFIED PARAMETERS ================\n');
fprintf('Iyy     = %.9g\n', Iyy);
fprintf('m_heli  = %.9g\n', m_heli);
fprintf('C_pitch = %.9g\n', C_pitch);
fprintf('T_bias  = %.9g\n', T_bias);
fprintf('c       = %.9g\n', c);
fprintf('Km      = %.9g\n', Km);
fprintf('J       = %.9g\n', J);
fprintf('L       = %.9g\n', L);
fprintf('R       = %.9g\n', R);
fprintf('C_T     = %.9g\n', C_T);



%% STEP 1: COMPUTE TRIM (EQUILIBRIUM)
% -------------------------------------------------------------------------
% States: x = [theta; theta_dot; I; w]
% Input : u = pitch motor voltage
%
% At equilibrium:
%   theta_dot_eq = 0
%   0 = (C_T*w_eq^2*L_m/Iyy) - [g*(m_heli*L_heli + m_cw*L_cw)/Iyy] * sin(theta_eq) + T_bias/Iyy
%   0 = -(R/L) I_eq - (Km/L) w_eq + u_eq/L
%   0 = (Km/J) I_eq - (c/J) w_eq^2
%
Kg = g * (m_heli*L_heli + m_cw*L_cw);

numerator = Kg*sin(theta_eq) - T_bias;
denominator = C_T * L_m;



w_eq_sq = numerator / denominator;



w_eq = sqrt(w_eq_sq);

if abs(Km) < 1e-12
    error('Trim failed: Km is too small / zero.');
end

I_eq = c*w_eq^2 / Km;
u_eq = R*I_eq + Km*w_eq;

x_eq = [theta_eq; 0; I_eq; w_eq];

fprintf('\n================ TRIM POINT ================\n');
fprintf('theta_eq = %.9g rad\n', theta_eq);
fprintf('theta_dot_eq = 0 rad/s\n');
fprintf('I_eq     = %.9g A\n', I_eq);
fprintf('w_eq     = %.9g rad/s\n', w_eq);
fprintf('u_eq     = %.9g V\n', u_eq);

%% STEP 2: ANALYTICAL LINEARIZATION ABOUT THE TRIM POINT
% -------------------------------------------------------------------------
% Nonlinear model:
%   x1 = theta
%   x2 = theta_dot
%   x3 = I
%   x4 = w
%
%   dx1 = x2
%   dx2 = (C_T*x4^2*L_m/Iyy) - (Kg/Iyy)*sin(x1) - (C_pitch/Iyy)*x2 + T_bias/Iyy
%   dx3 = -(R/L)*x3 - (Km/L)*x4 + (1/L)*u
%   dx4 = (Km/J)*x3 - (c/J)*x4^2
%
% Linearized (deviation) model:
%   d(x_tilde)/dt = A*x_tilde + B*u_tilde
%   y_tilde       = C*x_tilde + D*u_tilde
%
A = [ 0,                           1,             0,                    0;
     -(Kg/Iyy)*cos(theta_eq), -(C_pitch/Iyy),    0,       (2*C_T*L_m*w_eq)/Iyy;
      0,                           0,         -(R/L),             -(Km/L);
      0,                           0,          (Km/J),       -(2*c*w_eq)/J ];

B = [0;
     0;
     1/L;
     0];

C = [1 0 0 0];
D = 0;

sysc = ss(A,B,C,D);

fprintf('\n================ LINEAR MODEL ================\n');
disp('A = '); disp(A);
disp('B = '); disp(B);
disp('C = '); disp(C);
disp('D = '); disp(D);

%% STEP 3: CHECK CONTROLLABILITY AND OBSERVABILITY
% -------------------------------------------------------------------------
Co = ctrb(A,B);
Ob = obsv(A,C);
rank_Co = rank(Co);
rank_Ob = rank(Ob);

fprintf('\n================ STRUCTURAL CHECKS ================\n');
fprintf('rank(ctrb(A,B)) = %d (out of 4)\n', rank_Co);
fprintf('rank(obsv(A,C)) = %d (out of 4)\n', rank_Ob);

if rank_Co < 4
    warning('The linearized model is NOT fully controllable at this operating point.');
end
if rank_Ob < 4
    warning('The linearized model is NOT fully observable from theta only at this operating point.');
end

%% STEP 4: CONTINUOUS-TIME LQR (REGULATION ONLY)
% -------------------------------------------------------------------------
% Cost: J = integral( x_tilde''Qx_tilde + u_tilde''Ru_tilde )
Q_lqr = diag([200, 20, 1, 1]);
R_lqr = 0.5;

K_lqr = lqr(A,B,Q_lqr,R_lqr);
Acl_lqr = A - B*K_lqr;

fprintf('\n================ CONTINUOUS LQR ================\n');
disp('K_lqr = '); disp(K_lqr);
disp('Closed-loop poles of A-B*K_lqr = '); disp(eig(Acl_lqr));

%% STEP 5: DISCRETIZE FOR DIGITAL CONTROLLER IMPLEMENTATION
% -------------------------------------------------------------------------
sysd = c2d(sysc, Ts, 'zoh');
[Ad, Bd, Cd, Dd] = ssdata(sysd);

fprintf('\n================ DISCRETE MODEL ================\n');
disp('Ad = '); disp(Ad);
disp('Bd = '); disp(Bd);

%% STEP 6: DESIGN DISCRETE-TIME LQI FOR REFERENCE TRACKING
% -------------------------------------------------------------------------
% Add integrator state xi with update:
%   xi(k+1) = xi(k) + Ts*(r_tilde(k) - y_tilde(k))
%
% For design (with r_tilde treated externally):
%   [x_tilde(k+1)] = Ad*x_tilde(k) + Bd*u_tilde(k)
%   [xi(k+1)]      = xi(k) - Ts*Cd*x_tilde(k) + Ts*r_tilde(k)
%
A_aug = [Ad,            zeros(4,1);
        -Ts*Cd,         1        ];

B_aug = [Bd;
         0 ];

Q_lqi = diag([200, 20, 1, 1, 400]);
R_lqi = 0.5;

K_aug = dlqr(A_aug, B_aug, Q_lqi, R_lqi);
Kx = K_aug(1:4);
Ki = K_aug(5);

fprintf('\n================ DISCRETE LQI ================\n');
disp('K_aug = '); disp(K_aug);
disp('Kx = '); disp(Kx);
disp('Ki = '); disp(Ki);
disp('Augmented closed-loop poles = '); disp(eig(A_aug - B_aug*K_aug));

%% STEP 7: DESIGN DISCRETE KALMAN FILTER (STATE ESTIMATOR)
% -------------------------------------------------------------------------
% Tuning matrices for process / sensor noise
% Increase Qn if you trust the model less.
% Increase Rn if you trust the angle measurement less.
Qn = diag([1e-6, 1e-4, 1e-4, 1e-3]);
Rn = 1e-4;

% Discrete-time Kalman estimator gain
Lk = dlqe(Ad, eye(4), Cd, Qn, Rn);

fprintf('\n================ DISCRETE KALMAN FILTER ================\n');
disp('Lk = '); disp(Lk);
disp('Estimator poles = '); disp(eig(Ad - Lk*Cd));

%% STEP 8: LINEAR CLOSED-LOOP DEMO SIMULATION (WITH ESTIMATOR + INTEGRAL ACTION)
% -------------------------------------------------------------------------
Tsim = 6;                           % [s]
N = round(Tsim/Ts);
t = (0:N)'*Ts;

% True plant deviation state x_tilde
x_tilde = zeros(4, N+1);

% Estimated deviation state xhat_tilde
xhat_tilde = zeros(4, N+1);

% Integrator state
xi = zeros(1, N+1);

% Measured output deviation
y_tilde = zeros(1, N+1);

% Applied total input and deviation input
u_total = zeros(1, N+1);
u_tilde = zeros(1, N+1);

% Reference deviation (constant step)
r_tilde = (theta_ref - theta_eq) * ones(1, N+1);

% Optional: nonzero initial condition to test regulation
x_tilde(:,1) = [deg2rad(-8); 0; 0; 0];
xhat_tilde(:,1) = [0; 0; 0; 0];
xi(1) = 0;

for k = 1:N
    % Plant output (measured angle deviation), with optional measurement noise
    v_meas = sqrt(Rn) * randn;
    y_tilde(k) = Cd*x_tilde(:,k) + v_meas;

    % Control law (use estimated states)
    u_tilde_unsat = -Kx*xhat_tilde(:,k) - Ki*xi(k);
    u_total_unsat = u_eq + u_tilde_unsat;

    % Saturation (implementation realism)
    u_total(k) = min(max(u_total_unsat, -Vmax), Vmax);
    u_tilde(k) = u_total(k) - u_eq;

    % Linear plant update with process noise for demo
    w_proc = chol(Qn + 1e-15*eye(4), 'lower') * randn(4,1);
    x_tilde(:,k+1) = Ad*x_tilde(:,k) + Bd*u_tilde(k) + w_proc;

    % Kalman estimator update
    xhat_tilde(:,k+1) = Ad*xhat_tilde(:,k) + Bd*u_tilde(k) + ...
                        Lk*(y_tilde(k) - Cd*xhat_tilde(:,k));

    % Integrator update using measured output error
    % Anti-windup: freeze integrator when saturation occurs and command pushes further into saturation
    sat_high = (u_total_unsat > Vmax);
    sat_low  = (u_total_unsat < -Vmax);
    err_k = r_tilde(k) - y_tilde(k);

    if (sat_high && err_k > 0) || (sat_low && err_k < 0)
        xi(k+1) = xi(k);  % simple anti-windup
    else
        xi(k+1) = xi(k) + Ts*err_k;
    end
end

y_tilde(N+1) = Cd*x_tilde(:,N+1);
u_total(N+1) = u_total(N);
u_tilde(N+1) = u_tilde(N);

% Convert deviations back to physical variables
x_true = x_tilde + x_eq;
x_hat  = xhat_tilde + x_eq;
y_true = y_tilde + theta_eq;
r_true = r_tilde + theta_eq;

%% STEP 9: PLOTS
% -------------------------------------------------------------------------
figure('Name','Closed-loop response','Color','w');
subplot(4,1,1);
plot(t, rad2deg(y_true), 'b', 'LineWidth', 1.5); hold on;
plot(t, rad2deg(r_true), 'r--', 'LineWidth', 1.2);
grid on;
ylabel('\theta [deg]');
legend('Measured \theta','Reference','Location','best');
title('Pitch-angle tracking with LQI + Kalman filter');

subplot(4,1,2);
plot(t, rad2deg(x_true(1,:)), 'k', 'LineWidth', 1.2); hold on;
plot(t, rad2deg(x_hat(1,:)), '--g', 'LineWidth', 1.2);
grid on;
ylabel('\theta [deg]');
legend('True','Estimated','Location','best');

subplot(4,1,3);
plot(t, x_true(4,:), 'm', 'LineWidth', 1.2); hold on;
plot(t, x_hat(4,:), '--c', 'LineWidth', 1.2);
grid on;
ylabel('w [rad/s]');
legend('True','Estimated','Location','best');

subplot(4,1,4);
plot(t, u_total, 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('u [V]');
legend('Input voltage','Location','best');

figure('Name','State estimates','Color','w');
state_names = {'\theta [rad]','\theta dot [rad/s]','I [A]','w [rad/s]'};
for i = 1:4
    subplot(4,1,i);
    plot(t, x_true(i,:), 'b', 'LineWidth', 1.2); hold on;
    plot(t, x_hat(i,:), 'r--', 'LineWidth', 1.2);
    grid on;
    ylabel(state_names{i});
    if i == 1
        title('True vs estimated states');
    end
end
xlabel('Time [s]');

%% STEP 10: CONTROLLER IMPLEMENTATION TEMPLATE
% -------------------------------------------------------------------------
% Save the designed controller/observer data for later use in Simulink or code
controller_data = struct();
controller_data.Ts      = Ts;
controller_data.Ad      = Ad;
controller_data.Bd      = Bd;
controller_data.Cd      = Cd;
controller_data.Dd      = Dd;
controller_data.Kx      = Kx;
controller_data.Ki      = Ki;
controller_data.Lk      = Lk;
controller_data.x_eq    = x_eq;
controller_data.u_eq    = u_eq;
controller_data.theta_eq = theta_eq;
controller_data.Vmax    = Vmax;

save('helicopter_lqg_controller_data.mat', 'controller_data');

fprintf('\nSaved controller data to: helicopter_lqg_controller_data.mat\n');

%% OPTIONAL: SIMPLE REAL-TIME / SIMULINK STYLE UPDATE EQUATIONS
% -------------------------------------------------------------------------
% The equations below show how you would implement the controller online.
%
% Persistent variables you keep between sample times:
%   xhat_tilde_k : 4x1 estimated deviation state
%   xi_k         : scalar integral state
%
% At each sample k:
%   y_k          : measured theta [rad]
%   r_k          : desired theta [rad]
%
% Convert to deviation variables:
%   y_tilde_k = y_k - theta_eq
%   r_tilde_k = r_k - theta_eq
%
% Control law:
%   u_tilde_k = -Kx*xhat_tilde_k - Ki*xi_k
%   u_k = u_eq + u_tilde_k
%   u_k = saturate(u_k, -Vmax, Vmax)
%   u_tilde_k = u_k - u_eq
%
% Observer update:
%   xhat_tilde_{k+1} = Ad*xhat_tilde_k + Bd*u_tilde_k + Lk*(y_tilde_k - Cd*xhat_tilde_k)
%
% Integrator update:
%   xi_{k+1} = xi_k + Ts*(r_tilde_k - y_tilde_k)
%
% If you implement in Simulink, this is straightforward with Unit Delay blocks,
% Gain blocks, Sum blocks, Saturation, and a MATLAB Function block if desired.

%% OPTIONAL: VERIFY MATCH AGAINST NONLINEAR MODEL NUMERICALLY
% -------------------------------------------------------------------------
% If your nonlinear state function exists on the MATLAB path as
% state_space_function.m and you want to compare the linear controller on the
% nonlinear plant, you can build a simulation around it. That is typically the
% next step after this file.

fprintf('\nDone. Review warnings carefully before using the controller on hardware.\n');

%% LOCAL HELPER FUNCTIONS
% -------------------------------------------------------------------------
function y = saturate(u, umin, umax)
    y = min(max(u, umin), umax);
end
