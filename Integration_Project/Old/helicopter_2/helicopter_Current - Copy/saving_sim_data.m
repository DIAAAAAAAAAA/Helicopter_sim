time = simout.Time;
data = simout.Data;

plot(time,data)

save('simulation_data4.mat','simout')

M = [time data];

writematrix(M,'simout.csv')