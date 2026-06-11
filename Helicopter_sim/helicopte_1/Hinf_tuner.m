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
%% =========================================================
% IMPROVED AUTO-TUNER FOR TRACKING + DISTURBANCE REJECTION
% Main weight is Wr on S = E/R
% Includes W2 and optional W3
%% =========================================================

Ts = 0.01;
Gd = c2d(G, Ts, 'zoh');
s = tf('s');

% ---- Search ranges ----
% These are less extreme and more likely to work for your plant
M_list    = [1.6 1.8 2.0 2.2];
wb_list   = [0.8 1.0 1.2 1.5 1.8 2.0];
Aerr_list = [0.003 0.005 0.01 0.02];
W2_list   = [0.01 0.03 0.05 0.08 0.12];

% Include an empty W3 option and two mild W3 options
W3_list = { ...
    [], ...
    makeweight(0.02, 8, 3), ...
    makeweight(0.02, 12, 2) ...
};

% ---- Evaluation settings ----
tFinal = 25;          % longer simulation horizon
t = 0:Ts:tFinal;

r_step = 0.03;        % smaller reference for evaluation
d_step = 0.01;        % disturbance amplitude
u_limit = 0.50;       % relaxed actuator limit for tuning

% Set this depending on where you inject disturbance in Simulink:
% true  = disturbance added at plant input
% false = disturbance added at plant output
disturbance_is_input = true;

bestCost = inf;
bestData = struct();
candCount = 0;

fprintf('\n=============================================\n');
fprintf('Starting improved H-infinity auto-tuning...\n');
fprintf('=============================================\n');

for M = M_list
    for wb = wb_list
        for Aerr = Aerr_list

            Wr = (s/M + wb)/(s + wb*Aerr);

            for W2 = W2_list
                for iW3 = 1:length(W3_list)

                    W3 = W3_list{iW3};

                    try
                        % H-infinity synthesis
                        [K_try, ~, gamma_try] = mixsyn(G, Wr, W2, W3);
                        K_try = minreal(K_try);

                        % Continuous closed-loop
                        L_try = G * K_try;
                        S_try = feedback(1, L_try);   % E/R
                        T_try = feedback(L_try, 1);   % Y/R

                        if ~isstable(T_try)
                            continue;
                        end

                        % Discretize controller
                        Kd_try = c2d(K_try, Ts, 'tustin');
                        Kd_try = minreal(Kd_try);

                        % Discrete closed-loop
                        Ld_try = Gd * Kd_try;
                        Sd_try = feedback(1, Ld_try);
                        Td_try = feedback(Ld_try, 1);

                        p_d = pole(Td_try);
                        rho = max(abs(p_d));

                        % Hard reject only if REALLY too close to instability
                        if any(abs(p_d) >= 1)
                            continue;
                        end

                        if rho >= 0.9997
                            continue;
                        end

                        %% ---- 1) Tracking test ----
                        [y_track, tout] = step(r_step * Td_try, t);
                        y_track = squeeze(y_track);

                        if any(isnan(y_track)) || any(isinf(y_track))
                            continue;
                        end

                        info_track = stepinfo(y_track, tout, r_step, ...
                            'SettlingTimeThreshold', 0.05);

                        % If it does not settle within tFinal, do not reject immediately.
                        % Just assign a large settling time.
                        if isempty(info_track.SettlingTime) || isnan(info_track.SettlingTime)
                            Ts_track = tFinal;
                        else
                            Ts_track = info_track.SettlingTime;
                        end

                        if isempty(info_track.RiseTime) || isnan(info_track.RiseTime)
                            Tr_track = tFinal;
                        else
                            Tr_track = info_track.RiseTime;
                        end

                        if isempty(info_track.Overshoot) || isnan(info_track.Overshoot)
                            OS_track = 100;
                        else
                            OS_track = max(info_track.Overshoot, 0);
                        end

                        %% ---- 2) Disturbance rejection test ----
                        % If disturbance is added at plant input, path is G*S
                        % If disturbance is added at plant output, path is S
                        if disturbance_is_input
                            Gdist_try = minreal(Gd * Sd_try);
                        else
                            Gdist_try = Sd_try;
                        end

                        [y_dist, tout2] = step(d_step * Gdist_try, t);
                        y_dist = squeeze(y_dist);

                        if any(isnan(y_dist)) || any(isinf(y_dist))
                            continue;
                        end

                        peak_dist = max(abs(y_dist));

                        % Settling band based on actual disturbance response size
                        band = max(0.05 * peak_dist, 1e-4);
                        idx_settle = find(abs(y_dist) > band, 1, 'last');

                        if isempty(idx_settle)
                            Ts_dist = 0;
                        else
                            Ts_dist = tout2(idx_settle);
                        end

                        osc_energy = trapz(tout2, y_dist.^2);

                        %% ---- 3) Control effort test ----
                        % U/R = K / (1 + G*K)
                        Ud_try = feedback(Kd_try, Gd);

                        [u_dev, ~] = step(r_step * Ud_try, t);
                        u_dev = squeeze(u_dev);

                        if any(isnan(u_dev)) || any(isinf(u_dev))
                            continue;
                        end

                        u_total = u0 + u_dev;
                        u_peak = max(abs(u_total));
                        u_violation = max(0, u_peak - u_limit);

                        %% ---- 4) Damping metric from continuous poles ----
                        p_c = pole(T_try);
                        idx_cplx = abs(imag(p_c)) > 1e-6;

                        if any(idx_cplx)
                            zeta_vals = -real(p_c(idx_cplx)) ./ abs(p_c(idx_cplx));
                            zeta_min = min(zeta_vals);
                        else
                            zeta_min = 1;
                        end

                        zeta_target = 0.30;
                        damp_penalty = max(0, zeta_target - zeta_min);

                        % Soft pole-radius penalty
                        rho_penalty = max(0, rho - 0.995);

                        %% ---- 5) Total cost ----
                        % Lower is better
                        J = ...
                            0.8 * Ts_track + ...
                            0.4 * Tr_track + ...
                            0.15 * OS_track + ...
                            4.0 * peak_dist + ...
                            1.2 * Ts_dist + ...
                            20  * osc_energy + ...
                            50  * damp_penalty + ...
                            60  * u_violation + ...
                            120 * rho_penalty + ...
                            0.3 * gamma_try;

                        candCount = candCount + 1;

                        fprintf(['Cand %3d | M=%3.1f wb=%3.1f A=%6.3f W2=%4.2f W3#=%d ', ...
                                 '| g=%6.3f | OSt=%5.1f | Tst=%5.2f | Pd=%6.4f | Tsd=%5.2f ', ...
                                 '| zeta=%5.3f | rho=%6.4f | umax=%5.3f | J=%7.3f\n'], ...
                                 candCount, M, wb, Aerr, W2, iW3, gamma_try, ...
                                 OS_track, Ts_track, peak_dist, Ts_dist, ...
                                 zeta_min, rho, u_peak, J);

                        if J < bestCost
                            bestCost = J;
                            bestData.M = M;
                            bestData.wb = wb;
                            bestData.Aerr = Aerr;
                            bestData.W2 = W2;
                            bestData.W3 = W3;
                            bestData.Wr = Wr;
                            bestData.gamma = gamma_try;
                            bestData.K = K_try;
                            bestData.Kd = Kd_try;
                            bestData.S = S_try;
                            bestData.T = T_try;
                            bestData.Sd = Sd_try;
                            bestData.Td = Td_try;
                            bestData.Ts_track = Ts_track;
                            bestData.Tr_track = Tr_track;
                            bestData.OS_track = OS_track;
                            bestData.peak_dist = peak_dist;
                            bestData.Ts_dist = Ts_dist;
                            bestData.osc_energy = osc_energy;
                            bestData.zeta_min = zeta_min;
                            bestData.rho = rho;
                            bestData.u_peak = u_peak;
                        end

                    catch ME
                        fprintf('Skipped M=%.2f wb=%.2f Aerr=%.3f W2=%.2f W3#=%d : %s\n', ...
                            M, wb, Aerr, W2, iW3, ME.message);
                        continue;
                    end
                end
            end
        end
    end
end

if isempty(fieldnames(bestData))
    error(['No acceptable stable controller found.' newline ...
           'Try these next: ' newline ...
           '1) Set wb_list = [1.0 1.2 1.5 1.8 2.0 2.5]' newline ...
           '2) Set W2_list = [0.005 0.01 0.03 0.05]' newline ...
           '3) Set u_limit = 0.8' newline ...
           '4) Set rho hard reject to 0.9999']);
end

K_hinf   = bestData.K;
K_hinf_d = bestData.Kd;
Wr       = bestData.Wr;
W3       = bestData.W3;
gamma    = bestData.gamma;

[Ak,Bk,Ck,Dk] = ssdata(K_hinf_d);

fprintf('\n=============================================\n');
fprintf('BEST CONTROLLER FOUND\n');
fprintf('=============================================\n');
fprintf('M            = %.3f\n', bestData.M);
fprintf('wb           = %.3f rad/s\n', bestData.wb);
fprintf('Aerr         = %.3f\n', bestData.Aerr);
fprintf('W2           = %.3f\n', bestData.W2);
fprintf('gamma        = %.4f\n', bestData.gamma);
fprintf('Track OS     = %.2f %%\n', bestData.OS_track);
fprintf('Track Ts     = %.3f s\n', bestData.Ts_track);
fprintf('Track Tr     = %.3f s\n', bestData.Tr_track);
fprintf('Peak dist    = %.4f rad\n', bestData.peak_dist);
fprintf('Dist settle  = %.3f s\n', bestData.Ts_dist);
fprintf('Min damping  = %.3f\n', bestData.zeta_min);
fprintf('Max |pole_d| = %.5f\n', bestData.rho);
fprintf('Peak input   = %.4f V\n', bestData.u_peak);
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