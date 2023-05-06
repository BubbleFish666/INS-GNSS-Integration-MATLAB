%% load data
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
gyrox = single(table2array(data(3:end,10)));  % rad/s
gyroy = single(table2array(data(3:end,11)));  % rad/s
gyroz = single(table2array(data(3:end,12)));  % rad/s
faccx = single(table2array(data(3:end,7)));  % m/s^2
faccy = single(table2array(data(3:end,8)));  % m/s^2
faccz = single(table2array(data(3:end,9)));  % m/s^2

Yaw = single(table2array(data(3:end,15)));

heading = single(table2array(data(3:end,25)));

% set data range in time (seconds)
% data_range = (22 <= t) & (t <= 35);
data_range = (65 <= t) & (t <= 75);
% data_range = (109 <= t) & (t <= 120);
% data_range = (144 <= t) & (t <= 160);
% data_range = (169 <= t) & (t <= 177);


%% initialize
% time step (s)
T = 0.025;

% initial orientation
% Rnb = eye(3);
psi0 = deg2rad(360 - 329.32);  % yaw
theta0 = 0;  % pitch
phi0 = 0;  % roll
Rnb0 = [0, 1, 0; 1, 0, 0; 0, 0, -1];  % n-frame to original b-frame
Rb0b = R3(psi0)*R2(theta0)*R1(phi0);  % b-frame to original b-frame
Rnb = Rnb0 * Rb0b;  % n-frame to b-frame

% % initial velocity
% v_nb_n = [0; 0; 0];

% initial latitude, longitude (degree) and height (m)
lat0 = single(deg2rad(49.065972574));
lon0 = single(deg2rad(9.260714896));
h0 = 520;

% earth rotational rate (rad/s)
w_ie = 7.292115e-5;
% earth rotational rate in n frame
omega_ie_n = w_ie * [0, sin(lat0), 0; 
                     -sin(lat0), 0, -cos(lat0);
                     0, cos(lat0), 0];

% % assume gravity to be constant (m/s^2)
% g = -9.80743;

% error states
dpsi_nb = [0; 0; 0];  % attitude error
dv_eb_n = [0; 0; 0];  % velocity error
dllh = [0; 0; 0];  % position error
ba = [0; 0; 0];  % accelorometer error
bg = [0; 0; 0];  % gyro error

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%% PAY ATTENTION TO SCALING %%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% covariance of estimate
P = zeros(15);
P(1,1) = 0.002^2;  % roll variance (0.002 rad)^2
P(2,2) = 0.002^2;  % pitch variance (0.002 rad)^2
P(3,3) = 0.02^2;  % yaw variance (0.02 rad)^2
P(4,4) = 1.0^2;  % north velocity variance (1.0 m/s)^2
P(5,5) = 1.0^2;  % east velocity variance (1.0 m/s)^2
P(6,6) = 0;  % vertical velocity variance
P(7,7) = (2.0 / meridionalRadius(lat0) * 1e3)^2;  % GNSS horizontal accuracy 2m -> lat (milli rad)^2
P(8,8) = (2.0 / transverseRadius(lat0) * 1e3)^2;  % GNSS horizontal accuracy 2m -> lon (milli rad)^2
P(9,9) = 0;  % height
P(10,10) = (0.03 * 10 * 1e-3)^2;  % accelorometer bias (0.03 mg)^2
P(11,11) = (0.03 * 10 * 1e-3)^2;  % accelorometer bias (0.03 mg)^2
P(12,12) = (0.03 * 10 * 1e-3)^2;  % accelorometer bias (0.03 mg)^2
P(13,13) = (deg2rad(8) / 3600)^2;  % gyro bias (8 deg/h)^2
P(14,14) = (deg2rad(8) / 3600)^2;  % gyro bias (8 deg/h)^2
P(15,15) = (deg2rad(8) / 3600)^2;  % gyro bias (8 deg/h)^2

% measurement matrix
H = [zeros(2,3), zeros(2,3), [eye(2), zeros(2,1)], zeros(2,3), zeros(2,3)];

% system noise
Q = zeros(15);
Q(4:6 , 4:6) = (1.6 * 10 * 1e-3 * T)^2 * eye(3);  % velocity error from accelorometer noise 1.6 mg
Q(1:3 , 1:3) = (deg2rad(0.06) * T)^2 * eye(3);  % attitude error from gyro noise 0.06 deg/s
% measurement noise
% error from GNSS 2.0m horizontal (only consider GNSS horizontal data)
% R = [(2.0 / meridionalRadius(deg2rad(lat)) * 1e3)^2, 0;
%      0, (2.0 / transverseRadius(deg2rad(lat)) * 1e3)^2];
R = [(2.0 / meridionalRadius(lat0))^2, 0;
     0, (2.0 / transverseRadius(lat0))^2];

% scale latutude and longitude
llh_scale = 1e3;  % milli rad
lat0 = lat0 * llh_scale;
lon0 = lon0 * llh_scale;

%% Kalman Filter
lat_incre_total = 0;
lon_incre_total = 0;
lat = lat0;
lon = lon0;
lat0_int = single(int32(lat0));
lon0_int = single(int32(lon0));
lat0_frac = lat0 - lat0_int;
lon0_frac = lon0 - lon0_int;

h = h0;

range_start = find(data_range,1,'first');
range_end = find(data_range,1,'last');
k = range_start;

LOG.KF.P3(1:2*(range_end-range_start+1)) = nan;
LOG.KF.bg(1:2*(range_end-range_start+1), 3) = nan;

while k <= range_end
    % propagation
    % attitude
    dpsi_nb_ = dpsi_nb;
    dpsi_nb = dpsi_nb_ + Rnb * T * bg;

    % velocity
    dv_eb_n_ = dv_eb_n;
    f_ib_b = [faccx(k); faccy(k); faccz(k)];
    f_ib_n = Rnb * f_ib_b;
    F21n = [0, f_ib_n(3), -f_ib_n(2);
            -f_ib_n(3), 0, f_ib_n(1);
            f_ib_n(2), -f_ib_n(1), 0];
    F23n = zeros(3);
    dv_eb_n = dv_eb_n_ + F21n * T * dpsi_nb_ + F23n * T * dllh + Rnb * T * ba;
    dv_eb_n(3) = 0;  % disable velocity in Down direction

    % position
    F32n = [1 * llh_scale / (meridionalRadius(lat / llh_scale) + h), 0, 0;
            0, 1 * llh_scale / ((transverseRadius(lat / llh_scale) + h) * cos(lat / llh_scale)), 0;
            0, 0, -1];
    dllh = dllh + F32n * T * dv_eb_n_;
    dllh(3) = 0;  % disable height

    % covariance of estimate
    PHI = [eye(3), zeros(3), zeros(3), zeros(3), Rnb * T;
           F21n * T, eye(3), F23n * T, Rnb * T, zeros(3);
           zeros(3), F32n * T, eye(3), zeros(3), zeros(3);
           zeros(3), zeros(3), zeros(3), eye(3), zeros(3);
           zeros(3), zeros(3), zeros(3), zeros(3), eye(3)];
    P = PHI * P * PHI' + Q;

    LOG.KF.P3(2 * (k - range_start) + 1) = P(3, 3);
    LOG.KF.bg(2 * (k - range_start) + 1, :) = bg;

    % correction
    % pay attention to numerical issues,
    % consider milli rad and split int part and decimal part
    % subtraction of 2 nearly equal numbers may cause numerical problem
    % y = llh_GNSS(1:2) - llh_INS(1:2);  % disable height
    lat_frac = lat_GNSS(k) * llh_scale - lat0_int;
    lon_frac = lon_GNSS(k) * llh_scale - lon0_int;
    y = [lat_frac; lon_frac] - [lat0_frac; lon0_frac];
    z_pre = [dpsi_nb; dv_eb_n; dllh; ba; bg];

    S = H * P * H' + R;
    K = P * H' * S^(-1);
    z = z_pre + K * (y - H * z_pre);

    dpsi_nb = z(1:3);
    dv_eb_n = z(4:6);
    dllh = z(7:9);
    ba = z(10:12);
    bg = z(13:15);

    % dpsi_nb = [0; 0; 0];  % attitude error
    % dv_eb_n = [0; 0; 0];  % velocity error
    % dllh = [0; 0; 0];  % position error
    % ba = [0; 0; 0];  % accelorometer error
    % bg = [0; 0; 0];  % gyro error

    P = (eye(15) - K * H) * P * (eye(15) - K * H)' + K * R * K';
    
    LOG.KF.P3(2 * (k - range_start + 1)) = P(3, 3);
    LOG.KF.bg(2 * (k - range_start + 1), :) = z(13:15);

    % increment step
    k = k + 1;
end

figure('name', 'P3')
plot(LOG.KF.P3)

figure('name', 'bg')
hold on
plot(LOG.KF.bg(:, 1))
plot(LOG.KF.bg(:, 2))
plot(LOG.KF.bg(:, 3))