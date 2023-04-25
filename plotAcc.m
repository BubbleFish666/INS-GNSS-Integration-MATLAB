%% load data
clear
data = readtable("Frames.xlsx");
t = table2array(data(3:end,1));
t = datenum(t, 'HH:MM:SS,FFF');
t = single((t - t(1)) * 86400);
% % rawVeloN and rawVeloE could have been swappd
% VeloN = single(table2array(data(3:end,35)));
% VeloE = single(table2array(data(3:end,34)));
% heading = single(table2array(data(3:end,25)));

accx = single(table2array(data(3:end,31)));
accy = single(table2array(data(3:end,32)));
frAccx = single(table2array(data(3:end,7)));
frAccy= single(table2array(data(3:end,8)));

%% set data range in time (seconds)
% data_range = (144 <= t) & (t <= 160);
data_range = (22 <= t) & (t <= 35);
% data_range = (109 <= t) & (t <= 120);
% data_range = (0 <= t) & (t <= 186);

%% plot data
subplot(2,1,1);
plot(t(data_range), accx(data_range), t(data_range), accy(data_range))
title('acc')
% xticks(0:10:200)
grid on
hold on
legend('accx','accy')

subplot(2,1,2);
plot(t(data_range), frAccx(data_range), t(data_range), frAccy(data_range))
title('free acc')
% xticks(0:10:200)
grid on
hold on
legend('free accx','free accy')

