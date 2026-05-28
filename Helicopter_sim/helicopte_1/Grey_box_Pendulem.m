


figure
plot(simout.Time,simout.Data(:,1))


%%

g = 9.81;

p_init = [ 0.039857 , 0.795216, 0.0039208,-0.00916309, 0, 0, 0, 0, 0]; % Iyy,m_heli, C_pitch,T_bias, a, b, c ,d,C_T

%[2, 4, 2] 

nl_sys = idnlgrey('state_space_function', [1,1,4], p_init);

% 1. Parameter names
nl_sys.Parameters(1).Name = 'Iyy';
nl_sys.Parameters(2).Name = 'm_heli';
nl_sys.Parameters(3).Name = 'C_pitch';
nl_sys.Parameters(4).Name = 'T_bias'; 
nl_sys.Parameters(5).Name = 'a';
nl_sys.Parameters(6).Name = 'b';
nl_sys.Parameters(7).Name = 'c';
nl_sys.Parameters(8).Name = 'd';
nl_sys.Parameters(9).Name = 'C_T';

% 2. Boundary setting
nl_sys.Parameters(1).Minimum = 1e-5;
nl_sys.Parameters(2).Minimum = 0.05;
nl_sys.Parameters(2).Maximum = 1.5;

% 3. Fixing parameters
nl_sys.Parameters(4).Fixed = false; % 
nl_sys.Parameters(5).Fixed = true;  % fix a
nl_sys.Parameters(6).Fixed = true;  % fix b
nl_sys.Parameters(7).Fixed = true;  % fix c
nl_sys.Parameters(8).Fixed = true;  % fix d
nl_sys.Parameters(9).Fixed = true;  % fix C_T



%% Zero input signal (Pendulum case)
T_start = 28.81;
T_end = 40.35;
num_samples_start = T_start * Fs;
num_samples_end = T_end * Fs;
y_avg = simout.Data(:,1);
y_avg = y_avg(num_samples_start:num_samples_end);
u_avg = zeros(length(y_avg),1);





%{
%% Data processing
samples_per_period = T_period * Fs;

% Remove first period
start_idx = samples_per_period + 1;

y_trim = simout.Data(start_idx:end-1,1);
u_trim = V_p(start_idx:end)';

% Reshape into 4 periods
y_matrix = reshape(y_trim, samples_per_period, []);
u_matrix = reshape(u_trim, samples_per_period, []);

% Average last 4 periods
y_avg = mean(y_matrix, 2);
u_avg = mean(u_matrix, 2);

% New time vector for one averaged period
t_avg = (0:samples_per_period-1)'/Fs;

figure
plot(t_avg,y_avg)
%}
%% Data definition

data = iddata(y_avg, u_avg, 1/Fs);
data.Name = 'Helicopter Flight Data';
data.InputName = {'Pitch Voltage'};
data.InputUnit = {'V' };
data.OutputName = {'Theta'};
data.OutputUnit = {'rad' };


opt = nlgreyestOptions;
opt.Display = 'on';          % 
opt.SearchMethod = 'lsqnonlin';     % 
opt.EstimateCovariance = false;

%% Pendulum initial conditions
nl_sys.InitialStates(1).Fixed = false; % theta_0 not fixed
nl_sys.InitialStates(1).Value = y_avg(1); % initial theta equal to measurement data

nl_sys.InitialStates(2).Fixed = false; % theta_dot_0
nl_sys.InitialStates(2).Value = 0;     % Initial theta 




%%
nl_sys_estimated = nlgreyest(data, nl_sys, opt);


present(nl_sys_estimated);