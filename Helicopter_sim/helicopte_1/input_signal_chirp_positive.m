% Time vector
Fs = 100;              % Sampling frequency (Hz)

N = 6;                             % number of running times
T_period = 15;
T_tot = N*T_period;                % Duration (seconds)
t = (0:1/Fs:T_tot - 1/Fs);

t_period = 0:1/Fs:T_period-1/Fs;

f_start = 0.1 ;
f_end = 2.5;

u1_raw =[];
for i = 1:N
    u1 = chirp(t_period, f_start, T_period, f_end, 'linear');
    u1_raw = [u1_raw, u1];
end 

V_p_hover = 0.15;
A_p = V_p_hover;



V_p =  V_p_hover + (A_p * u1_raw);


sim_input_p = timeseries(V_p,t);

sim_input_p_negative = timeseries(-V_p,t);
