


figure
plot(simout.Time,simout.Data(:,1))


%%

g = 9.81;

p_init = [ 1.5 , 0.7, 0.02, 2, 0.01, 0.05 , 0.05, 0.001, 0.001, 0.001, 0.001]; % (Iyy,m_heli, C_pitch, R, L, Kt, Ke, Jm, bm, kq, kt)


%[2, 4, 2] 

nl_sys = idnlgrey('state_space_function', [1,1,4], p_init);

% Parameter names
names = {'Iyy','m_heli','C_pitch','R','L','Kt','Ke','Jm','bm','kq','kt'};
for k=1:length(names)
    nl_sys.Parameters(k).Name = names{k};
end

theta0 = y_trim(1);
thetaDot0 = (y_trim(2)-y_trim(1))*Fs;

nl_sys.InitialStates(1).Value = theta0;
nl_sys.InitialStates(2).Value = thetaDot0;
nl_sys.InitialStates(3).Value = 0;
nl_sys.InitialStates(4).Value = 0;

nl_sys.InitialStates(1).Fixed = false;
nl_sys.InitialStates(2).Fixed = false;
nl_sys.InitialStates(3).Fixed = true;
nl_sys.InitialStates(4).Fixed = true;

% Bounds (very important)
nl_sys.Parameters(strcmp(names,'Iyy')).Minimum = 1e-5;
nl_sys.Parameters(strcmp(names,'L')).Minimum = 1e-5;
nl_sys.Parameters(strcmp(names,'Jm')).Minimum = 1e-6;

nl_sys.Parameters(strcmp(names,'C_pitch')).Minimum = 0;
nl_sys.Parameters(strcmp(names,'bm')).Minimum = 0;
nl_sys.Parameters(strcmp(names,'kq')).Minimum = 0;
nl_sys.Parameters(strcmp(names,'kt')).Minimum = 0;

% Reduce estimation complexity
nl_sys.Parameters(strcmp(names,'Jm')).Fixed = true;
nl_sys.Parameters(strcmp(names,'bm')).Fixed = true;
nl_sys.Parameters(strcmp(names,'kq')).Fixed = true;

% Initial states
nl_sys.InitialStates(1).Fixed = false;
nl_sys.InitialStates(2).Fixed = false;
nl_sys.InitialStates(3).Fixed = true;
nl_sys.InitialStates(4).Fixed = true;


%% Data processing
samples_per_period = T_period * Fs;

% Remove first period
start_idx = samples_per_period + 1;

% 1) Define trimmed signals FIRST (this fixes your error)
y_trim = simout.Data(start_idx:end, 1);
u_trim = V_p(start_idx:end)';

% 2) Make sure they are the same length
Nmin = min(length(y_trim), length(u_trim));
y_trim = y_trim(1:Nmin);
u_trim = u_trim(1:Nmin);

% 3) Ensure length is a multiple of samples_per_period (for reshape)
N_total = floor(length(y_trim)/samples_per_period);
N_use = N_total * samples_per_period;

y_trim = y_trim(1:N_use);
u_trim = u_trim(1:N_use);

% --- Optional averaging (only if you really want it) ---
y_matrix = reshape(y_trim, samples_per_period, N_total);
u_matrix = reshape(u_trim, samples_per_period, N_total);

y_avg = mean(y_matrix, 2);
u_avg = mean(u_matrix, 2);

t_avg = (0:samples_per_period-1)'/Fs;

figure; plot(t_avg, y_avg); grid on
title('Averaged output over periods');
xlabel('Time (s)'); ylabel('\theta (rad)');


%% Data definition

data = iddata(y_trim, u_trim, 1/Fs);

data.Name = 'Helicopter Flight Data';
data.InputName = {'Pitch Voltage'};
data.InputUnit = {'V' };
data.OutputName = {'Theta'};
data.OutputUnit = {'rad' };


opt = nlgreyestOptions;
opt.Display = 'on';          % 
opt.SearchMethod = 'lm';     % 
opt.EstimateCovariance = true;



nl_sys_estimated = nlgreyest(data, nl_sys, opt);


present(nl_sys_estimated);