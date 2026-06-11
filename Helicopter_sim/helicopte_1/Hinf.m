%% ============================================
% H-infinity tracking controller design
% Main weight is from reference r to error e
% Uses a small W2 regularization so the controller stays stable/usable
%% ============================================



%% 1. Get identified nonlinear grey-box model parameters
p = getpvec(nl_sys_estimated);

%% 2. Solve equilibrium point (x0, u0)
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

fprintf('Equilibrium point solving finished:\n');
fprintf('Theta     = %.6f rad\n', x0(1));
fprintf('Theta_dot = %.6f rad/s\n', x0(2));
fprintf('I         = %.6f A\n', x0(3));
fprintf('w         = %.6f rad/s\n', x0(4));
fprintf('u0        = %.6f V\n', u0);

%% 3. Linearization around equilibrium
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

%% 4. Linearized plant
G = ss(A,B,C,D);
G = minreal(G);

disp(' ');
disp('Linearized plant G(s):');
G

figure;
pzmap(G);
grid on;
title('Linearized Plant Poles');

%% 5. Check controllability and observability
Co = ctrb(A,B);
Ob = obsv(A,C);

fprintf('Controllability rank = %d\n', rank(Co));
fprintf('Observability rank   = %d\n', rank(Ob));

%% 6. Tracking weight from reference to error
% S = E/R = 1/(1+GK)
s = tf('s');

M    = 2.0;     % less aggressive than 1.5
wb   = 1.5;     % target tracking bandwidth [rad/s]
Aerr = 0.02;    % low-frequency tracking requirement (relaxed)

Wr = (s/M + wb)/(s + wb*Aerr);

% Small control regularization (recommended)
W2 = 0.05;

disp('Tracking weighting filter Wr(s) = ');
Wr

figure;
bodemag(Wr);
grid on;
title('Tracking Weight Wr(s) on E/R');

%% 7. H-infinity synthesis
% Main objective = tracking (Wr on S = E/R)
% W2 only regularizes control so the design stays usable
[K_hinf, CL, gamma] = mixsyn(G, Wr, W2, []);
K_hinf = minreal(K_hinf);

disp('-------------------------------------');
disp('H-infinity tracking controller synthesis done');
disp(['gamma = ', num2str(gamma)]);
disp('-------------------------------------');

disp('Continuous-time H-infinity controller K(s):');
K_hinf

%% 8. Closed-loop analysis
L = G*K_hinf;
S = feedback(1, L);   % Error / Reference
T = feedback(L, 1);   % Output / Reference

disp('Closed-loop poles (continuous):');
disp(pole(T));

if isstable(T)
    disp('Continuous-time H-infinity closed-loop is stable.');
else
    disp('Continuous-time H-infinity closed-loop is NOT stable.');
end

figure;
step(T, 10);
grid on;
title('Closed-Loop Tracking Response Y/R (Continuous)');

figure;
step(S, 10);
grid on;
title('Error Transfer E/R = S (Continuous)');

figure;
sigma(S, {1e-2,1e2});
grid on;
title('Sensitivity S = E/R');

%% 9. Discretize controller
Ts = 0.01;   % 100 Hz sample time
K_hinf_d = c2d(K_hinf, Ts, 'tustin');
K_hinf_d = minreal(K_hinf_d);

disp('Discrete-time controller K(z):');
K_hinf_d

[Ak, Bk, Ck, Dk] = ssdata(K_hinf_d);

disp('Discrete controller matrices:');
disp('Ak ='); disp(Ak);
disp('Bk ='); disp(Bk);
disp('Ck ='); disp(Ck);
disp('Dk ='); disp(Dk);

%% 10. Verify discrete closed-loop stability
Gd = c2d(G, Ts, 'zoh');
Td = feedback(Gd*K_hinf_d, 1);

disp('Closed-loop poles (discrete):');
disp(pole(Td));

if all(abs(pole(Td)) < 1)
    disp('Discrete H-infinity closed-loop is stable.');
else
    disp('Discrete H-infinity closed-loop is NOT stable.');
end

%% 11. Continuous vs discrete comparison
figure;
step(T, Td, 10);
grid on;
legend('Continuous CL','Discrete CL');
title('Continuous vs Discrete Closed-Loop Tracking Response');

figure;
bode(T, Td);
grid on;
legend('Continuous CL','Discrete CL');
title('Continuous vs Discrete Closed-Loop Frequency Response');

%% 12. Export variables for Simulink
assignin('base','Ak',Ak);
assignin('base','Bk',Bk);
assignin('base','Ck',Ck);
assignin('base','Dk',Dk);
assignin('base','u0',u0);
assignin('base','Ts',Ts);

disp('Variables exported to base workspace: Ak, Bk, Ck, Dk, u0, Ts');