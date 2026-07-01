% Time vector
Fs = 100;              % Sampling frequency (Hz)

                    % number of running times
T_period = 60;             % Duration (seconds)
t = (0:1/Fs:T_tot - 1/Fs);



V_p_hover = 0;
A_p = 0.5;

%V_y_bias = 0;
%A_y = 0.3;

V_p =  (A_p *ones(size(t)));


sim_input_p = timeseries(V_p,t);
%sim_input_y = timeseries(V_y,t);


angle1 = [ 0.02, 0.1, 0.18, 0.3, 0.4, 0.51]
input = [0.1, 0.2,]