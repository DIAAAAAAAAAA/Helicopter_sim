% Time vector
Fs = 100;                          % Sampling frequency (Hz)
N = 1;                             % number of running times
T_period = 15;
T_tot = N*T_period;                % Duration (seconds)
t = (0:1/Fs:T_tot - 1/Fs);

% Step parameters
t_step = 3;                        % Step start time (seconds)
A_p = 0.3;                         % Amplitude of the step
V_p_hover = 0;                     % Hover voltage offset

% Generate Step Signal: 
% (t >= t_step) creates a logical array of 0s before 3s, and 1s after 3s
u1_step = double(t >= t_step); 

% Calculate the final input voltage
V_p = V_p_hover + (A_p * u1_step);

% --- Plotting to verify the signal ---
figure;
plot(t, V_p, 'LineWidth', 2);
grid on;
xlabel('Time (seconds)');
ylabel('Voltage V_p (V)');
title('Step Input Signal starting at t = 3s');
ylim([V_p_hover - 0.1, V_p_hover + A_p + 0.1]); % Zoom window nicely

sim_input = [t', V_p'];


