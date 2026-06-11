%% ============================================================
% H-infinity tracking controller design with auto-tuning
% For helicopter pitch control around hover
%
% This script:
%   1) gets equilibrium point
%   2) linearizes the nonlinear model
%   3) builds linear plant G(s)
%   4) auto-tunes a tracking-focused Hinf controller
%   5) discretizes controller
%   6) verifies continuous and discrete closed-loop stability
%   7) exports Ak, Bk, Ck, Dk, u0, Ts to workspace for Simulink
%
% Required in workspace before running:
%   - nl_sys_estimated
%   - state_space_function.m
%% ============================================================

clc;
clearvars -except nl_sys_estimated
close all;

%% ============================================================
% 1. Get identified model parameters
%% ============================================================
p = getpvec(nl_sys_estimated);

%% ============================================================
% 2. Solve equilibrium point (x0, u0)
% Solve dx = 0 with theta = 0
%% ============================================================
objective = @(p_in) [ ...
    state_space_function(0, p_in(1:4), p_in(5), ...
    p(1), p(2), p(3), p(4), p(5), p(6), p(7), p(8), p(9), p(10)); ...
    p_in(1)];

x0_guess = [0; 0; 0; 0; 0];
options = optimoptions('fsolve','Display','off');
[sol,~,exitflag] = fsolve(objective, x0_guess, options);

if exitflag <= 0
    warning('Equilibrium solve may not have converged properly.');
end

x0 = sol(1:4);
u0 = sol(5);

fprintf('\n=============================================\n');
fprintf('Equilibrium point solving finished\n');
fprintf('=============================================\n');
fprintf('Theta     = %.6f rad\n', x0(1));
fprintf('Theta_dot = %.6f rad/s\n', x0(2));
fprintf('I         = %.6f A\n', x0(3));
fprintf('w         = %.6f rad/s\n', x0(4));
fprintf('u0        = %.6f V\n', u0);

%% ============================================================
% 3. Linearization around equilibrium
%% ============================================================
h = 1e-4;
A = zeros(4,4);
B = zeros(4,1);

for i = 1:4
    x_plus = x0;
    x_minus = x0;
    x_plus(i)  = x_plus(i)  + h;
    x_minus(i) = x_minus(i) - h;

    [dx_p,~] = state_space_function(0, x_plus, u0, ...
        p(1),p(2),p(3),p(4),p(5),p(6),p(7),p(8),p(9),p(10));
    [dx_m,~] = state_space_function(0, x_minus, u0, ...
        p(1),p(2),p(3),p(4),p(5),p(6),p(7),p(8),p(9),p(10));

    A(:,i) = (dx_p - dx_m)/(2*h);
end

[dx_p,~] = state_space_function(0, x0, u0 + h, ...
    p(1),p(2),p(3),p(4),p(5),p(6),p(7),p(8),p(9),p(10));
[dx_m,~] = state_space_function(0, x0, u0 - h, ...
    p(1),p(2),p(3),p(4),p(5),p(6),p(7),p(8),p(9),p(10));

B = (dx_p - dx_m)/(2*h);

C = [1 0 0 0];
D = 0;

%% ============================================================
% 4. Build linearized plant
%% ============================================================
G = ss(A,B,C,D);
G = minreal(G);

fprintf('\n=============================================\n');
fprintf('Linearized plant G(s)\n');
fprintf('=============================================\n');
disp(G);

figure;
pzmap(G);
grid on;
title('Linearized Plant Poles');

Co = ctrb(A,B);
Ob = obsv(A,C);

fprintf('Controllability rank = %d\n', rank(Co));
fprintf('Observability rank   = %d\n', rank(Ob));

%% ============================================================
% 5. Auto-tuner settings
%% ============================================================
Ts = 0.01;                % controller sample time
Gd = c2d(G, Ts, 'zoh');   % discrete plant for verification
s = tf('s');

% ---- Search ranges ----
% These ranges are chosen to reduce oscillation
M_list    = [2.5 3 4];
wb_list   = [0.4 0.6 0.8 1.0];
Aerr_list = [0.03 0.05 0.08];
W2_list   = [0.1 0.2 0.3 0.5];

% ---- Evaluation settings ----
tFinal = 15;
t = 0:Ts:tFinal;
r_step = 0.05;         % test step amplitude [rad]
u_limit = 0.25;        % assumed actuator voltage limit for scoring

bestCost = inf;
bestData = struct();
candCount = 0;

fprintf('\n=============================================\n');
fprintf('Starting H-infinity auto-tuning...\n');
fprintf('=============================================\n');

%% ============================================================
% 6. Auto-tuning loop
%% ============================================================
for M = M_list
    for wb = wb_list
        for Aerr = Aerr_list

            % Tracking weight on sensitivity S = E/R
            Wr = (s/M + wb)/(s + wb*Aerr);

            for W2 = W2_list
                try
                    % Synthesize Hinf controller
                    [K_try, CL_try, gamma_try] = mixsyn(G, Wr, W2, []);
                    K_try = minreal(K_try);

                    % Continuous closed-loop transfer functions
                    L_try = G * K_try;
                    S_try = feedback(1, L_try);    % E/R
                    T_try = feedback(L_try, 1);    % Y/R

                    % Reject unstable continuous designs
                    if ~isstable(T_try)
                        continue;
                    end

                    % Discretize controller
                    Kd_try = c2d(K_try, Ts, 'tustin');
                    Kd_try = minreal(Kd_try);

                    % Discrete closed-loop
                    Ld_try = Gd * Kd_try;
                    Td_try = feedback(Ld_try, 1);

                    % Reject unstable discrete designs
                    p_d = pole(Td_try);
                    rho = max(abs(p_d));

                    if any(abs(p_d) >= 1)
                        continue;
                    end

                    % Reject poles too close to unit circle
                    if rho >= 0.995
                        continue;
                    end

                    % Step response of output tracking
                    [y, tout] = step(r_step * Td_try, t);
                    y = squeeze(y);

                    if any(isnan(y)) || any(isinf(y))
                        continue;
                    end

                    % Step response metrics
                    info = stepinfo(y, tout, r_step, 'SettlingTimeThreshold', 0.02);

                    if isempty(info.SettlingTime) || isnan(info.SettlingTime) || isinf(info.SettlingTime)
                        continue;
                    end

                    % Controller output relationship:
                    % U/R = K / (1 + G*K)
                    Ud_try = feedback(Kd_try, Gd);

                    [u_dev, ~] = step(r_step * Ud_try, t);
                    u_dev = squeeze(u_dev);

                    if any(isnan(u_dev)) || any(isinf(u_dev))
                        continue;
                    end

                    % Add trim voltage
                    u_total = u0 + u_dev;
                    u_peak = max(abs(u_total));

                    % Approximate minimum damping ratio from continuous poles
                    p_c = pole(T_try);
                    idx_cplx = abs(imag(p_c)) > 1e-6;

                    if any(idx_cplx)
                        zeta_vals = -real(p_c(idx_cplx)) ./ abs(p_c(idx_cplx));
                        zeta_min = min(zeta_vals);
                    else
                        zeta_min = 1;
                    end

                    overshoot = max(info.Overshoot, 0);
                    settling  = info.SettlingTime;
                    rise      = info.RiseTime;

                    % Control effort penalty
                    u_violation = max(0, u_peak - u_limit);

                    % Penalize poor damping
                    zeta_target = 0.40;
                    damp_penalty = max(0, zeta_target - zeta_min);

                    % Penalize poles too close to unit circle
                    rho_penalty = max(0, rho - 0.98);

                    % Cost function
                    % Weight damping and pole margin strongly to reduce oscillation
                    J = ...
                        2.0 * overshoot + ...
                        1.5 * settling + ...
                        0.5 * rise + ...
                        80  * damp_penalty + ...
                        100 * u_violation + ...
                        250 * rho_penalty + ...
                        0.7 * gamma_try;

                    candCount = candCount + 1;

                    fprintf(['Candidate %3d | M=%4.2f wb=%4.2f Aerr=%5.3f W2=%4.2f ', ...
                             '| gamma=%6.3f | OS=%6.2f%% | Ts=%5.2f s | zeta=%.3f ', ...
                             '| rho=%.4f | umax=%.3f | J=%.3f\n'], ...
                             candCount, M, wb, Aerr, W2, gamma_try, ...
                             overshoot, settling, zeta_min, rho, u_peak, J);

                    % Store best controller
                    if J < bestCost
                        bestCost = J;
                        bestData.M = M;
                        bestData.wb = wb;
                        bestData.Aerr = Aerr;
                        bestData.W2 = W2;
                        bestData.Wr = Wr;
                        bestData.gamma = gamma_try;
                        bestData.K = K_try;
                        bestData.Kd = Kd_try;
                        bestData.S = S_try;
                        bestData.T = T_try;
                        bestData.Td = Td_try;
                        bestData.info = info;
                        bestData.zeta_min = zeta_min;
                        bestData.rho = rho;
                        bestData.u_peak = u_peak;
                    end

                catch ME
                    fprintf('Skipped: M=%.2f wb=%.2f Aerr=%.3f W2=%.2f --> %s\n', ...
                        M, wb, Aerr, W2, ME.message);
                    continue;
                end
            end
        end
    end
end

%% ============================================================
% 7. Check tuning result
%% ============================================================
if isempty(fieldnames(bestData))
    error('No acceptable stable controller was found. Try increasing W2 and/or reducing wb.');
end

K_hinf   = bestData.K;
K_hinf_d = bestData.Kd;
Wr       = bestData.Wr;
gamma    = bestData.gamma;

[Ak, Bk, Ck, Dk] = ssdata(K_hinf_d);

fprintf('\n=============================================\n');
fprintf('BEST CONTROLLER FOUND\n');
fprintf('=============================================\n');
fprintf('M            = %.3f\n', bestData.M);
fprintf('wb           = %.3f rad/s\n', bestData.wb);
fprintf('Aerr         = %.3f\n', bestData.Aerr);
fprintf('W2           = %.3f\n', bestData.W2);
fprintf('gamma        = %.4f\n', bestData.gamma);
fprintf('Overshoot    = %.2f %%\n', bestData.info.Overshoot);
fprintf('SettlingTime = %.3f s\n', bestData.info.SettlingTime);
fprintf('RiseTime     = %.3f s\n', bestData.info.RiseTime);
fprintf('Min damping  = %.3f\n', bestData.zeta_min);
fprintf('Max |pole_d| = %.5f\n', bestData.rho);
fprintf('Peak input   = %.4f V\n', bestData.u_peak);

disp('Best tracking weighting filter Wr(s):');
disp(Wr);

fprintf('\nContinuous-time H-infinity controller K(s):\n');
disp(K_hinf);

fprintf('\nDiscrete-time H-infinity controller K(z):\n');
disp(K_hinf_d);

fprintf('\nDiscrete controller matrices:\n');
disp('Ak ='); disp(Ak);
disp('Bk ='); disp(Bk);
disp('Ck ='); disp(Ck);
disp('Dk ='); disp(Dk);

%% ============================================================
% 8. Final stability checks
%% ============================================================
T_best_c = bestData.T;
T_best_d = bestData.Td;

disp('Closed-loop poles (continuous):');
disp(pole(T_best_c));

if isstable(T_best_c)
    disp('Continuous-time H-infinity closed-loop is stable.');
else
    disp('Continuous-time H-infinity closed-loop is NOT stable.');
end

disp('Closed-loop poles (discrete):');
disp(pole(T_best_d));

if all(abs(pole(T_best_d)) < 1)
    disp('Discrete H-infinity closed-loop is stable.');
else
    disp('Discrete H-infinity closed-loop is NOT stable.');
end

%% ============================================================
% 9. Plots
%% ============================================================
figure;
bodemag(Wr);
grid on;
title('Selected Tracking Weight Wr(s) on E/R');

figure;
step(T_best_c, 15);
grid on;
title('Best Continuous Closed-Loop Tracking Response');

figure;
step(T_best_d, 15);
grid on;
title('Best Discrete Closed-Loop Tracking Response');

figure;
step(T_best_c, T_best_d, 15);
grid on;
legend('Continuous CL','Discrete CL');
title('Continuous vs Discrete Closed-Loop Tracking Response');

figure;
step(bestData.S, 15);
grid on;
title('Best Error Transfer E/R');

figure;
sigma(bestData.S, {1e-2,1e2});
grid on;
title('Sensitivity S = E/R');

figure;
pzmap(T_best_d);
grid on;
title('Best Discrete Closed-Loop Poles');

%% ============================================================
% 10. Export variables for Simulink
%% ============================================================
assignin('base','Ak',Ak);
assignin('base','Bk',Bk);
assignin('base','Ck',Ck);
assignin('base','Dk',Dk);
assignin('base','u0',u0);
assignin('base','Ts',Ts);
assignin('base','K_hinf',K_hinf);
assignin('base','K_hinf_d',K_hinf_d);
assignin('base','Wr',Wr);

fprintf('\nVariables exported to base workspace:\n');
fprintf('  Ak, Bk, Ck, Dk, u0, Ts, K_hinf, K_hinf_d, Wr\n');

%% ============================================================
% 11. Notes for Simulink implementation
%% ============================================================
disp(' ');
disp('SIMULINK IMPLEMENTATION:');
disp('1) Use a Discrete State-Space block with A=Ak, B=Bk, C=Ck, D=Dk, Ts=Ts');
disp('2) Form error: e = r - theta');
disp('3) Controller output = delta_u');
disp('4) Add trim voltage: u = u0 + delta_u');
disp('5) Add Saturation block after u = u0 + delta_u');
disp('6) Feed plant input as [u_sat; 0] if second motor is unused');
disp('7) Start with small steps, e.g. 0.02 rad or 0.05 rad');