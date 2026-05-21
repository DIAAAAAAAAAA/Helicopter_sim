time = simout.Time;
data = simout.Data;

plot(time,data(:,3))

save('simulation_data6.mat','simout')

M = [time data];

writematrix(M,'simout.csv')