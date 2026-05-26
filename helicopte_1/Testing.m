figure
plot(simout.Time,simout.Data(:,1))


%%

g = 9.81;

p_init = [ 1.5 , 1, 0.7, 0.06, 0.02, 1]; % (Iyy,K_pitch,m_heli,L_heli,L_cw)

%[2, 4, 2] 

nl_sys = idnlgrey('state_space_function', [1,1,2], p_init);
% Parameter names
nl_sys.Parameters(1).Name = 'Iyy';
nl_sys.Parameters(2).Name = 'K_pitch';
nl_sys.Parameters(3).Name = 'm_heli';
nl_sys.Parameters(4).Name = 'L_heli';
nl_sys.Parameters(5).Name = 'L_cw';

nl_sys.Parameters(1).Minimum = 1e-5;

nl_sys.Parameters(3).Minimum = 0.05;
nl_sys.Parameters(3).Maximum = 1.5;

nl_sys.Parameters(5).Minimum = 0;
nl_sys.Parameters(5).Maximum = 0.4;


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

y_matrix = y_matrix - y_matrix(1,:);
y_avg = mean(y_matrix, 2)
u_avg = mean(u_matrix, 2)
% New time vector for one averaged period
t_avg = (0:samples_per_period-1)'/Fs;

figure
plot(t_avg,y_avg)

%% Data definition

data = iddata(y_avg, u_avg, 1/Fs);
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