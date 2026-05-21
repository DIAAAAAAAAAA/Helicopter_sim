%% --- Plot measured output ---
figure; plot(simout.Time, simout.Data(:,1)); grid on
title('Measured \theta'); xlabel('Time (s)'); ylabel('\theta (rad)');

%% --- Data processing (DO THIS FIRST) ---
samples_per_period = T_period * Fs;
start_idx = samples_per_period + 1;

y_trim = simout.Data(start_idx:end, 1);
u_trim = V_p(start_idx:end)';

% Make sure column vectors
y_trim = y_trim(:);
u_trim = u_trim(:);

% Match lengths
Nmin = min(length(y_trim), length(u_trim));
y_trim = y_trim(1:Nmin);
u_trim = u_trim(1:Nmin);

% Check for NaN/Inf in data (IMPORTANT)
if any(~isfinite(y_trim)) || any(~isfinite(u_trim))
    error('y_trim or u_trim contains NaN/Inf. Clean your data first.');
end

% Optional: enforce full number of periods (only needed if you reshape)
N_total = floor(length(y_trim)/samples_per_period);
N_use   = N_total * samples_per_period;
y_trim  = y_trim(1:N_use);
u_trim  = u_trim(1:N_use);

% Optional averaging (kept from your original)
y_matrix = reshape(y_trim, samples_per_period, N_total);
u_matrix = reshape(u_trim, samples_per_period, N_total);
y_avg = mean(y_matrix, 2);
u_avg = mean(u_matrix, 2);
t_avg = (0:samples_per_period-1)'/Fs;

figure; plot(t_avg, y_avg); grid on
title('Averaged output over periods');
xlabel('Time (s)'); ylabel('\theta (rad)');

%% --- Data object (you are using raw trimmed data) ---
data = iddata(y_trim, u_trim, 1/Fs);
data.Name = 'Helicopter Flight Data';
data.InputName = {'Pitch Voltage'};
data.InputUnit = {'V'};
data.OutputName = {'Theta'};
data.OutputUnit = {'rad'};

%% --- Grey-box model definition ---
% IMPORTANT: use simulation-safe initial guesses (especially kt/kq/Jm/bm)
p_init = [ ...
    1.5 , ...    % Iyy
    0.7 , ...    % m_heli
    0.02, ...    % C_pitch
    2   , ...    % R
    0.01, ...    % L
    0.05, ...    % Kt
    0.05, ...    % Ke
    1e-4, ...    % Jm  (smaller than 0.001 to avoid explosive omega)
    1e-4, ...    % bm  (smaller)
    1e-6, ...    % kq  (much smaller)
    1e-5  ...    % kt  (much smaller to avoid thrust explosion)
];

nl_sys = idnlgrey('state_space_function', [1,1,4], p_init);

% Parameter names
names = {'Iyy','m_heli','C_pitch','R','L','Kt','Ke','Jm','bm','kq','kt'};
for k = 1:length(names)
    nl_sys.Parameters(k).Name = names{k};
end

%% --- Bounds (keep yours, add safety) ---
nl_sys.Parameters(strcmp(names,'Iyy')).Minimum = 1e-5;
nl_sys.Parameters(strcmp(names,'L')).Minimum   = 1e-5;
nl_sys.Parameters(strcmp(names,'Jm')).Minimum  = 1e-8;

nl_sys.Parameters(strcmp(names,'C_pitch')).Minimum = 0;
nl_sys.Parameters(strcmp(names,'bm')).Minimum      = 0;
nl_sys.Parameters(strcmp(names,'kq')).Minimum      = 0;
nl_sys.Parameters(strcmp(names,'kt')).Minimum      = 0;

%% --- Reduce estimation complexity (your approach kept) ---
nl_sys.Parameters(strcmp(names,'m_heli')).Fixed = true;
nl_sys.Parameters(strcmp(names,'R')).Fixed      = true;
nl_sys.Parameters(strcmp(names,'L')).Fixed      = true;
nl_sys.Parameters(strcmp(names,'Ke')).Fixed     = true;
nl_sys.Parameters(strcmp(names,'Kt')).Fixed     = true;

nl_sys.Parameters(strcmp(names,'Jm')).Fixed = true;
nl_sys.Parameters(strcmp(names,'bm')).Fixed = true;
nl_sys.Parameters(strcmp(names,'kq')).Fixed = true;
% Free: Iyy, C_pitch, kt (good starting point)

%% --- Initial states (KEY FIX: robust thetaDot0) ---
theta0 = y_trim(1);

% BAD (noisy): (y_trim(2)-y_trim(1))*Fs
% GOOD: start with 0 (and let estimator refine it)
thetaDot0 = 0;

nl_sys.InitialStates(1).Value = theta0;
nl_sys.InitialStates(2).Value = thetaDot0;
nl_sys.InitialStates(3).Value = 0;
nl_sys.InitialStates(4).Value = 0;

nl_sys.InitialStates(1).Fixed = false;  % estimate theta0
nl_sys.InitialStates(2).Fixed = false;  % estimate thetaDot0
nl_sys.InitialStates(3).Fixed = true;
nl_sys.InitialStates(4).Fixed = true;

%% --- Set solver on the MODEL (idnlgrey has SimulationOptions) ---
% SimulationOptions is a property of idnlgrey. [1](https://www.mathworks.com/help/ident/ref/idnlgrey.html)
nl_sys.SimulationOptions.Solver  = 'ode15s'; % stiff solver often helps [2](https://www.mathworks.com/help/matlab/ref/ode15s.html)
% --- Set simulation options in the format YOUR idnlgrey expects ---
simopt = nl_sys.SimulationOptions;   % start from existing struct

simopt.Solver      = 'ode15s';  % good for stiff dynamics
simopt.RelTol      = 1e-4;
simopt.AbsTol      = 1e-6;

simopt.MinStep     = 0;         % 0 = let solver decide
simopt.MaxStep     = 1/Fs;      % don't step larger than sample time
simopt.InitialStep = 0;         % 0 = let solver decide
simopt.MaxOrder    = 5;         % typical for ode15s (can keep default)
simopt.FixedStep   = 0;         % not used for variable-step solvers, but required

nl_sys.SimulationOptions = simopt;
%% --- Estimation options ---
opt = nlgreyestOptions;
opt.Display = 'on';
opt.SearchMethod = 'lm';
opt.EstimateCovariance = true;

%% --- Feasibility check (MUST pass before nlgreyest) ---
figure; compare(data, nl_sys); grid on
title('Feasibility check: initial model vs data');

%% --- Estimate ---
nl_sys_estimated = nlgreyest(data, nl_sys, opt);
present(nl_sys_estimated);

figure; compare(data, nl_sys_estimated); grid on
title('Estimated model vs data');