
%% Drawing testing data
figure
plot(simout.Time,simout.Data(:,1))


%%
g = 9.81;


p_init = [5, 0.44482, 0.05, 0.01, 0.124921, 3.90808, 1.0, 0.5, 0.1]; % (Iyy,K_pitch,m_heli,L_heli,L_cw)

%[2, 4, 2] 

nl_sys = idnlgrey('state_space_function', [2,2,4], p_init);
% Parameter names
nl_sys.Parameters(1).Name = 'Iyy';
nl_sys.Parameters(2).Name = 'K_pitch';
nl_sys.Parameters(3).Name = 'm_heli';
nl_sys.Parameters(4).Name = 'L_heli';
nl_sys.Parameters(5).Name = 'L_cw';
nl_sys.Parameters(6).Name = 'C_pitch';
nl_sys.Parameters(7).Name = 'K_yaw';
nl_sys.Parameters(8).Name = 'Izz';
nl_sys.Parameters(9).Name = 'C_yaw';

% Fix parameters 1 to 6 so they are not modified during estimation
for idx = 1:6
    nl_sys.Parameters(idx).Fixed = true;
end

nl_sys.Parameters(8).Minimum = 1e-4;   % Izz
nl_sys.Parameters(9).Minimum = 1e-3;   % C_yaw

%{
nl_sys.Parameters(1).Minimum = 1e-4;   nl_sys.Parameters(1).Maximum = 5;      % Iyy
nl_sys.Parameters(2).Minimum = 1e-3;   nl_sys.Parameters(2).Maximum = 50;     % K_pitch
nl_sys.Parameters(3).Minimum = 0.05;   nl_sys.Parameters(3).Maximum = 1.5;    % m_heli
nl_sys.Parameters(4).Minimum = 0.01;   nl_sys.Parameters(4).Maximum = 0.5;    % L_heli
nl_sys.Parameters(5).Minimum = 0;      nl_sys.Parameters(5).Maximum = 0.4;    % L_cw
nl_sys.Parameters(6).Minimum = 0.001;  nl_sys.Parameters(6).Maximum = 10;     % C_pitch (阻尼下限绝不能为0)
%}

%{
%% Data processing
samples_per_period = T_period * Fs;

% Remove first period
start_idx = samples_per_period + 1;

y_trim = simout.Data(start_idx:end-1,[1,3]);
u_trim = [zeros(size(V_p(start_idx:end)')),V_p(start_idx:end)'];

% Reshape into 4 periods
y_matrix = reshape(y_trim, samples_per_period, []);
u_matrix = reshape(u_trim, samples_per_period, []);

% Average last 4 periods
y_avg = mean(y_matrix, 2);
u_avg = mean(u_matrix, 2);

% New time vector for one averaged period
t_avg = (0:samples_per_period-1)'/Fs;




%% Data definition

data = iddata(y_avg, u_avg, 1/Fs);
data.Name = 'Helicopter Flight Data';
data.InputName = {'Pitch Voltage'};
data.InputUnit = {'V' };
data.OutputName = {'Theta'};
data.OutputUnit = {'rad' };
%}



%% Data processing
samples_per_period = T_period * Fs;

% Remove first period
start_idx = samples_per_period + 1;
y_trim = simout.Data(start_idx:end-1, [1,3]); % 2 columns (Outputs)
u_trim = [zeros(size(V_p(start_idx:end)')), V_p(start_idx:end)']; % 2 columns (Inputs)

% Determine how many periods are available
num_periods = size(y_trim, 1) / samples_per_period;

% Correctly reshape multi-channel data: [Samples Per Period x Channels x Periods]
y_3d = reshape(y_trim, samples_per_period, 2, num_periods);
u_3d = reshape(u_trim, samples_per_period, 2, num_periods);

% Average across the 3rd dimension (the periods)
y_avg = mean(y_3d, 3);
u_avg = mean(u_3d, 3);

% New time vector for one averaged period
t_avg = (0:samples_per_period-1)'/Fs;


%% Data definition
data = iddata(y_avg, u_avg, 1/Fs);
data.Name = 'Helicopter Flight Data';

% Map the names to match your 2 inputs and 2 outputs
data.InputName = {'Zero Input', 'Pitch Voltage'};
data.InputUnit = {'V', 'V'};
data.OutputName = {'Theta', 'Phi'}; % Replace 'Phi' with your actual 2nd output name
data.OutputUnit = {'rad', 'rad'};



opt = nlgreyestOptions;
opt.Display = 'on';          % 
opt.SearchMethod = 'lsqnonlin';     % 
opt.EstimateCovariance = true;


nl_sys_estimated = nlgreyest(data, nl_sys, opt);


present(nl_sys_estimated);