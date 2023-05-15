%% load data
clear; close all;
data = readtable("Frames.xlsx");
t = table2array(data(3:end,1));
t = datenum(t, 'HH:MM:SS,FFF');
t = single((t - t(1)) * 86400);
% % rawVeloN and rawVeloE could have been swappd
% VeloN = single(table2array(data(3:end,35)));
% VeloE = single(table2array(data(3:end,34)));
% heading = single(table2array(data(3:end,25)));

lat = single(table2array(data(3:end,16)));
lon = single(table2array(data(3:end,17)));
rlat = single(table2array(data(3:end,29)));
rlon = single(table2array(data(3:end,30)));

%% set data range in time (seconds)
% data_range = (144 <= t) & (t <= 160);
% data_range = (22 <= t) & (t <= 35);
% data_range = (109 <= t) & (t <= 120);
data_range = (8 <= t) & (t <= 160);

%% plot data
subplot(2,1,1);
plot(t(data_range), (lat(data_range)-49.06588).*meridionalRadius(49.06588), 'x',...
     t(data_range), (rlat(data_range)-49.06588).*meridionalRadius(49.06588), '.')
legend('lat (m)', 'raw lat (m)')
xlabel('time (s)')
% xticks(0:10:200)
grid on
hold on

subplot(2,1,2);
plot(t(data_range), (lon(data_range)-9.26066).*transverseRadius(9.26066), 'x',...
     t(data_range), (rlon(data_range)-9.26066).*transverseRadius(9.26066), '.')
legend('lon (m)', 'raw lon (m)')
xlabel('time (s)')
% xticks(0:10:200)
grid on
hold on

% subplot(3,1,3);
% plot(t(data_range), heading(data_range), 'r')
% title('Yaw Angle')
% % xticks(0:10:200)
% grid on
% hold on
% 
% % Orientation based on Velocity
% OboV = atan2d(VeloN, VeloE);
% % OboV = -OboV + 360 + 90;
% % OboV = mod(OboV,360);
% OboV = invTanDegToCompassDeg(OboV);
% subplot(3,1,3);
% plot(t(data_range), OboV(data_range), 'b')
% % xticks(0:10:200)
% xlabel('Time [s]')
% ylabel('Yaw Angle [degree]')
% legend('Heading','atan2d(VeloN, VeloE)')
% grid on
% hold on
% 
