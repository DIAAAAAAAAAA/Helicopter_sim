


figure
plot(simout.Time,simout.Data(:,1))


%%

g = 9.81;

p_init = [ 0.039857 , 0.795216, 0.0039208,-0.00916309, 0.000612219, 0.0051767, 0.000220079, 0.0148128 , 4.12861, 0.353859]; % Iyy,m_heli, C_pitch,T_bias,c, Km, J ,L , R,C_T

%[2, 4, 2] 

nl_sys = idnlgrey('state_space_function', [1,1,4], p_init);

% 1. Parameter names
nl_sys.Parameters(1).Name = 'Iyy';
nl_sys.Parameters(2).Name = 'm_heli';
nl_sys.Parameters(3).Name = 'C_pitch';
nl_sys.Parameters(4).Name = 'T_bias'; 
nl_sys.Parameters(5).Name = 'c';
nl_sys.Parameters(6).Name = 'Km';
nl_sys.Parameters(7).Name = 'J';
nl_sys.Parameters(8).Name = 'L';
nl_sys.Parameters(9).Name = 'R';
nl_sys.Parameters(10).Name = 'C_T';


% 3. Fixing parameters
nl_sys.Parameters(1).Fixed = true; % 
nl_sys.Parameters(2).Fixed = true;  % fix 
nl_sys.Parameters(3).Fixed = true;  % fix 
nl_sys.Parameters(4).Fixed = true;  % fix 


% 2. 电气参数 (这些是本次识别的重点)
% c: 阻尼系数 (对应 w^2 项)，通常为很小的正数
nl_sys.Parameters(5).Minimum = 1e-6;    
nl_sys.Parameters(5).Maximum = 0.1;

% Km: 电机常数，必须为正
nl_sys.Parameters(6).Minimum = 1e-4;    
nl_sys.Parameters(6).Maximum = 1.0;

% J: 电机转子转动惯量，必须为微小的正数
nl_sys.Parameters(7).Minimum = 1e-6;    
nl_sys.Parameters(7).Maximum = 0.01;

% L: 电感，必须为微小的正数
nl_sys.Parameters(8).Minimum = 1e-6;    
nl_sys.Parameters(8).Maximum = 0.5;

% R: 电阻，必须为正 (根据你的电动机规格，设定合理范围)
nl_sys.Parameters(9).Minimum = 0.1;     
nl_sys.Parameters(9).Maximum = 20.0;

% 3. 推力参数
% C_T: 推力系数，必须为正
nl_sys.Parameters(10).Minimum = 1e-4;   
nl_sys.Parameters(10).Maximum = 2.0;



%% Data processing
samples_per_period = T_period * Fs;

% Remove first period
start_idx = 2*samples_per_period + 1;
end_idx = 4 * samples_per_period;

y_trim = simout.Data(start_idx:end_idx,1);
u_trim = V_p(start_idx:end_idx)';

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


% --- ADD THESE TO SPEED UP ---
opt.SearchOption.TolFun = 1e-4;    % Terminate if change in cost function is small
opt.SearchOption.TolX = 1e-4;      % Terminate if parameter changes are tiny
opt.SearchOption.MaxIter = 15;     % Cap the iterations so it won't run forever
% Set the model internal simulation solver to fixed-step Runge-Kutta 4


nl_sys.SimulationOptions.Solver = 'ode4'; 
nl_sys.SimulationOptions.FixedStep = 1/Fs; % Set step size to exactly 0.01s

%% Pendulum initial conditions
nl_sys.InitialStates(1).Fixed = true; % theta_0 not fixed
nl_sys.InitialStates(1).Value = y_avg(1); % initial theta equal to measurement data

nl_sys.InitialStates(2).Fixed = false; % theta_dot_0
nl_sys.InitialStates(2).Value = V_p(1);     % Initial theta 

nl_sys.InitialStates(3).Fixed = false; % I


nl_sys.InitialStates(4).Fixed = false; % w



%%
nl_sys_estimated = nlgreyest(data, nl_sys, opt);


present(nl_sys_estimated);

%% Validation(Using the 5th period)

start_idx_val = 4 * samples_per_period + 1;
end_idx_val   = 5 * samples_per_period;

y_val = simout.Data(start_idx_val:end_idx_val, 1);
u_val = V_p(start_idx_val:end_idx_val)';


data_val = iddata(y_val, u_val, 1/Fs);
data_val.Name = 'Helicopter Validation Data (5th Period)';
data_val.InputName = {'Pitch Voltage'};
data_val.InputUnit = {'V'};
data_val.OutputName = {'Theta'};
data_val.OutputUnit = {'rad'};

%% Model Validation and Comparison
fprintf('\n--- Running Validation on 5th Period Data ---\n');


% Setting initial states
nl_sys_estimated.InitialStates(1).Value = y_val(1); 
nl_sys_estimated.InitialStates(1).Fixed = false;    


% Plotting the comparsion diagram (Fit Percentage)
figure('Name', 'Model Validation - 5th Period');
compare(data_val, nl_sys_estimated);
grid on;
title('Model Validation Comparison (5th Period Data)');






