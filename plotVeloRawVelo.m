%% load data
clear
data = readtable("Frames.xlsx");
t = table2array(data(3:end,1));
t = datenum(t, 'HH:MM:SS,FFF');
t = single((t - t(1)) * 86400);
% VeloN and VeloE could have been swappd
VeloN = single(table2array(data(3:end,19)));
VeloE = single(table2array(data(3:end,18)));
% rawVeloN and rawVeloE could have been swappd
rawVeloN = single(table2array(data(3:end,35)));
rawVeloE = single(table2array(data(3:end,34)));

%% set data range in time (seconds)
% data_range = (22 <= t) & (t <= 35);
data_range = (65 <= t) & (t <= 75);
% data_range = (109 <= t) & (t <= 120);
% data_range = (144 <= t) & (t <= 160);
% data_range = (169 <= t) & (t <= 177);

%% plot data
subplot(2,1,1);
plot(t(data_range), VeloN(data_range), t(data_range), rawVeloN(data_range))
title('velocity North')
legend('VeloN','rawVeloN')
% xticks(0:10:200)
grid on
hold on

subplot(2,1,2);
plot(t(data_range), VeloE(data_range), t(data_range), rawVeloE(data_range))
title('velocity East')
% xticks(0:10:200)
grid on
hold on
legend('VeloE','rawVeloE')
