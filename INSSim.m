%% set trajectory
% load data
clear; close all;
data = readtable("Frames.xlsx");
t = table2array(data(3:end,1));
t = datenum(t, 'HH:MM:SS,FFF');
t = single((t - t(1)) * 86400);

% GPS data
% rawrawVeloN and rawrawVeloE could have been swappd
VeloN = single(table2array(data(3:end,19)));  % m/s
VeloE = single(table2array(data(3:end,18)));  % m/s
VeloU = 0;  % m/s
lat_GNSS = deg2rad(single(table2array(data(3:end,16))));  % deg -> rad
lon_GNSS = deg2rad(single(table2array(data(3:end,17))));  % deg -> rad
% IMU data
% MTi-3
gyrox = single(table2array(data(3:end,10)));  % rad/s
gyroy = single(table2array(data(3:end,11)));  % rad/s
gyroz = single(table2array(data(3:end,12)));  % rad/s
% acceleration in x- and y- direction could have been swapped
faccx = single(table2array(data(3:end,7)));  % m/s^2
faccy = single(table2array(data(3:end,8)));  % m/s^2
faccz = single(table2array(data(3:end,9)));  % m/s^2
% MTi-7
% accx = single(table2array(data(3:end,31)));  % m/s^2
% accy = single(table2array(data(3:end,32)));  % m/s^2
% accz = single(table2array(data(3:end,33)));  % m/s^2

Yaw = single(table2array(data(3:end,15)));
% YawOffset = 121.6556;
YawOffset = 0;
heading = single(table2array(data(3:end,25)));

% set data range in time (seconds)
% data_range = (22 <= t) & (t <= 35);
% data_range = (26 <= t) & (t <= 35);
data_range = (22 <= t) & (t <= 160);
% data_range = (65 <= t) & (t <= 75);
% data_range = (109 <= t) & (t <= 120);
% data_range = (144 <= t) & (t <= 160);
% data_range = (169 <= t) & (t <= 177);

range_start = find(data_range,1,'first');
range_end = find(data_range,1,'last');
k = range_start;

%% initialize
% time step (s)
T = 0.025;

% initial orientation
% psi0 = deg2rad(360 - 150.16);  % yaw at 67.0 s
psi0 = deg2rad(360 - 324.37);  % yaw at 26.0 s
% psi0 = 0;  % yaw at 26.0 s
theta0 = 0;  % pitch
phi0 = 0;  % roll

% MTi-3 frame is rotated about z-axis by -88.75 (or -59) deg (probably due to disabling the magnetometer)
R_board_sensor = R3(deg2rad(-59));  % original MTi-3 board frame (ENU) to sensor frame
Rnb0 = R3(0.5 * pi) * R1(pi);  % n-frame to original MTi-3 board frame (body frame)
Rb0b = R3(psi0)*R2(theta0)*R1(phi0);  % align yaw of MTi-3 frame to actual heading
Rnb = Rnb0 * Rb0b;  % n-frame to b-frame

% initial velocity
v_eb_n = [0; 0; 0];

% initial latitude, longitude (degree) and height (m)
% lat0 = single(deg2rad(48.1351));
% lon0 = single(deg2rad(11.5820));
% lat0 = single(deg2rad(49.065972574));
% lon0 = single(deg2rad(9.260714896));
lat0 = single(lat_GNSS(range_start));
lon0 = single(lon_GNSS(range_start));
h0 = 520;

% earth rotational rate (rad/s)
w_ie = 7.292115e-5;
% earth rotational rate in n frame
omega_ie_n = w_ie * [0, sin(lat0), 0; 
                     -sin(lat0), 0, -cos(lat0);
                     0, cos(lat0), 0];

% assume gravity to be constant (m/s^2)
g = [0; 0; -9.80743];

% scale latutude and longitude
llh_scale = 1e3;  % milli rad
lat0 = lat0 * llh_scale;
lon0 = lon0 * llh_scale;

%% strapdown solution
% pre-allocate vectors for logging
LOG.llh(1:range_end-range_start+1, 1:3) = nan;
LOG.v_eb_n(1:range_end-range_start+1, 1:3) = nan;
% LOG.Rnb(1:range_end-range_start+1, 1:3) = nan;
LOG.eul_b0b(1:range_end-range_start+1, 1:3) = nan;
LOG.llh_incre_total(1:range_end-range_start+1, 1:3) = nan;
LOG.llh_incre(1:range_end-range_start+1, 1:3) = nan;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%% PAY ATTENTION TO ORIENTATION OF THE BODY FRAME %%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

lat_incre_total = 0;
lon_incre_total = 0;
lat = lat0;
lon = lon0;
lat0_int = single(floor(lat0));
lon0_int = single(floor(lon0));
lat0_frac = lat0 - lat0_int;
lon0_frac = lon0 - lon0_int;

h = h0;

while k <= range_end
    % read data from IMU
    % w_ibx_b = gyrox(k);
    % w_iby_b = gyroy(k);
    % w_ibz_b = gyroz(k);
    w_ib_b0 = R_board_sensor * [gyrox(k); gyroy(k); gyroz(k)];
    % note that fAcc are already compensated by sensor with gravity
    % It seems that MTi-3 frame is a global floating frame,
    % which means sensor data are measured in that global frame.
    % With facc multiplied by R_board_sensor (a quasi constant orientation
    % offset), f_ib_b0 represents the acc measured in b0 frame.
    f_ib_b0 = R_board_sensor * [faccx(k); faccy(k); faccz(k)];
    % f_ib_b = [accx(k); accy(k); accz(k)];

    % rotation
    Rb0b = Rnb0' * Rnb;
    w_ib_b = Rb0b' * w_ib_b0;
    w_ibx_b = w_ib_b(1);
    w_iby_b = w_ib_b(2);
    w_ibz_b = w_ib_b(3);
    Rnb_ = Rnb;
    omega_ib_b = [0, -w_ibz_b, w_iby_b;
                  w_ibz_b, 0, -w_ibx_b;
                  -w_iby_b, w_ibx_b, 0];
    omega_ie_n_ = omega_ie_n;
    Rnb = Rnb_ * (eye(3) + omega_ib_b * T) - omega_ie_n_ * Rnb_ * T;

    % velocity
    v_eb_n_ = v_eb_n;
    % v_eb_n = v_eb_n_ + (Rnb_ * f_ib_b + g - cross(2 * Ren' * w_ie, v_eb_n)) * T;
    % v_eb_n = v_eb_n_ + (Rnb_ * f_ib_b + g) * T;
    % v_eb_n = v_eb_n_ + (Rnb_ * f_ib_b) * T;
    v_eb_n = v_eb_n_ + Rnb0 * f_ib_b0 * T;
    v_eb_n(3) = 0;  % disable velocity in Down direction

    % position
    % h = h;
    lat_ = lat;
    lat_incre = (v_eb_n_(1) / (meridionalRadius(lat_ / llh_scale) + h) + v_eb_n(1) / (meridionalRadius(lat_ / llh_scale) + h))...
                * 0.5 * T * llh_scale;
    lat_incre_total = lat_incre_total + lat_incre;
    lat = lat0 + lat_incre_total;

    lon_incre = (v_eb_n_(2) / ((transverseRadius(lat_ / llh_scale) + h) * cos(lat_ / llh_scale)) + v_eb_n(2) / ((transverseRadius(lat / llh_scale) + h) * cos(lat / llh_scale)))...
                * 0.5 * T * llh_scale;
    lon_incre_total = lon_incre_total + lon_incre;
    lon = lon0 + lon_incre_total;

    % logging
    eul_nb = rotm2eul(Rnb) * 180 / pi;
    eul_nb(1) = changeDegRange360(eul_nb(1));
    LOG.eul_nb(k-range_start+1, :) = eul_nb;
    
    eul_b0b = rotm2eul(Rnb0'*Rnb) * 180 / pi;
    eul_b0b(1) = changeDegRange360(eul_b0b(1));
    LOG.eul_b0b(k-range_start+1, :) = eul_b0b;

    LOG.v_eb_n(k-range_start+1, :) = v_eb_n;
    LOG.llh(k-range_start+1, :) = [lat, lon, h];
    LOG.llh_incre_total(k-range_start+1, :) = [lat_incre_total, lon_incre_total, 0];
    LOG.llh_incre(k-range_start+1, :) = [lat_incre, lon_incre, 0];

    % increment step
    k = k + 1;

end

%% plot
close all;

subplot(4,1,1);
plot(t(data_range), LOG.eul_b0b(:, 1), t(data_range), Yaw(data_range) + YawOffset,...
     t(data_range), 360-heading(data_range))
title('yaw')
legend('INS-yaw', 'sensor-yaw')
grid on
hold on

% subplot(4,1,1);
% plot(t(data_range), LOG.Rnb(:, 1), t(data_range), Yaw(data_range) + YawOffset,...
%      t(data_range), 360-heading(data_range))
% title('yaw')
% legend('INS-yaw', 'sensor-yaw')
% grid on
% hold on

subplot(4,1,2);
plot(t(data_range), LOG.v_eb_n(:, 1), '.', t(data_range), LOG.v_eb_n(:, 2), '.',...
     t(data_range), VeloN(data_range), t(data_range), VeloE(data_range),...
     t(data_range), LOG.v_eb_n(:, 3))
title('velocity')
legend('Vn INS', 'Ve INS', 'Vn MTi7', 'Ve MTi7', 'Vh')
grid on
hold on

subplot(4,1,3);
plot(t(data_range), LOG.llh_incre_total(:,1) + lat0_frac)
title('position')
legend('lat frac (milli rad)')
grid on
hold on

subplot(4,1,4);
plot(t(data_range), LOG.llh_incre_total(:,2) + lon0_frac)
title('position')
legend('lon frac (milli rad)')
grid on
hold on

% subplot(4,1,3);
% plot(t(data_range), LOG.llh_incre_total(:,1) + lat0_frac - (lat_GNSS(data_range) * llh_scale - lat0_int))
% title('position')
% legend('lat (milli rad)')
% % xticks(0:10:200)
% grid on
% hold on
% 
% subplot(4,1,4);
% plot(t(data_range), LOG.llh_incre_total(:,2) + lon0_frac - (lon_GNSS(data_range) * llh_scale - lon0_int))
% title('position')
% legend('lon (milli rad)')
% % xticks(0:10:200)
% grid on
% hold on
