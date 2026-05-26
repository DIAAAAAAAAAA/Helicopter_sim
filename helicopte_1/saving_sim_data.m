time = simout.Time;
data = simout.Data;

plot(time,data(:,1))

save('pendelum_data2.mat','simout')

M = [time data];

writematrix(M,'simout.csv')