


figure
plot(simout.Time,simout.Data(:,1))


%%

g = 9.81;

p_init = [ 0.039857 , 0.795216, 0.0039208,-0.00916309, 0.000612219, 0.0051767, 0.000220079, 0.0148128 , 4.12861, -0.353859]; % Iyy,m_heli, C_pitch,T_bias,c, Km, J ,L , R,C_T

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

%{
% --- Parameter 6: Km (Motor Constant) ---
nl_sys.Parameters(6).Minimum = 0.005;
nl_sys.Parameters(6).Maximum = 0.100;

% --- Parameter 7: J (Rotor Inertia) ---
nl_sys.Parameters(7).Minimum = 1.0e-6;
nl_sys.Parameters(7).Maximum = 1.0e-3;

% --- Parameter 8: L (Inductance) ---
nl_sys.Parameters(8).Minimum = 1.0e-4;
nl_sys.Parameters(8).Maximum = 0.050;

% --- Parameter 9: R (Resistance) ---
nl_sys.Parameters(9).Minimum = 0.40;
nl_sys.Parameters(9).Maximum = 5.00;

% --- Parameter 10: C_T (Thrust Coefficient) ---
% Allow c to go higher if needed
nl_sys.Parameters(5).Minimum = 1e-7;
nl_sys.Parameters(5).Maximum = 0.1;   % Increased from 0.001

% Allow C_T to go lower or higher
nl_sys.Parameters(10).Maximum = 0.1;
%}




%% Data processing
samples_per_period = T_period * Fs;

% Remove first period
start_idx = 2*samples_per_period + 1;

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
nl_sys.InitialStates(1).Fixed = false; % theta_0 not fixed
nl_sys.InitialStates(1).Value = y_avg(1); % initial theta equal to measurement data

nl_sys.InitialStates(2).Fixed = false; % theta_dot_0
nl_sys.InitialStates(2).Value = V_p(1);     % Initial theta 

nl_sys.InitialStates(3).Fixed = false; % I


nl_sys.InitialStates(4).Fixed = false; % w



%%
nl_sys_estimated = nlgreyest(data, nl_sys, opt);


present(nl_sys_estimated);