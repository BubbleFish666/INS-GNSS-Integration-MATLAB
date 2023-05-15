%% load data
clear; close all;
ref_traj = load("trajData.mat");
t = ref_traj.timeVector(2:end);

% reference latitude and longitude
ref.lat = ref_traj.pos_geo_incre_log(:, 2) + ref_traj.lat0 * 1000;  % milli rad
ref.lon = ref_traj.pos_geo_incre_log(:, 1) + ref_traj.lon0 * 1000;  % milli rad
% reference velocity
ref.VeloN = ref_traj.vel_log(:, 2);  % m/s
ref.VeloE = ref_traj.vel_log(:, 1);  % m/s
ref.VeloU = 0;  % m/s
% reference acceleration in body frame
ref.accx = ref_traj.accb_log(:, 1);  % m/s^2
ref.accy = ref_traj.accb_log(:, 2);  % m/s^2
ref.accz = ref_traj.accb_log(:, 3);  % always 0
% reference orientation
ref.eulb0b = ref_traj.eulerAngles;  % deg, yaw-pitch-roll
%reference angular velocity
ref.gyrox = ref_traj.angVelb_log(:, 1);  % always 0
ref.gyroy = ref_traj.angVelb_log(:, 2);  % always 0
ref.gyroz = -ref_traj.angVelb_log(:, 3);  % rad/s, not sure why but it's reversed

%% simulate IMU data
initIMUModel
[IMU_acc, IMU_gyro] = IMUModel([ref.accx, ref.accy, ref.accz],...
                               [ref.gyrox, ref.gyroy, ref.gyroz]);
IMU.faccx = -single(IMU_acc(:, 1));  % m/s^2
IMU.faccy = -single(IMU_acc(:, 2));  % m/s^2
IMU.faccz = -single(IMU_acc(:, 3) - 9.81);  % m/s^2
IMU.gyrox = single(IMU_gyro(:, 1));  % rad/s
IMU.gyroy = single(IMU_gyro(:, 2));  % rad/s
IMU.gyroz = single(IMU_gyro(:, 3));  % rad/s

%% plot simulated IMU data
figure('Name', 'IMU Acc data')
plot(t, IMU.faccx, t, IMU.faccy, t, IMU.faccz,...
     t, ref.accx, t, ref.accy, t, ref.accz)
legend('IMU accx', 'IMU accy', 'IMU accz', 'ref accx', 'ref accy', 'ref accz')
grid on

figure('Name', 'IMU Gyro data')
plot(t, IMU.gyrox, t, IMU.gyroy, t, IMU.gyroz,...
     t, ref.gyrox, t, ref.gyroy, t, ref.gyroz)
legend('IMU gyrox', 'IMU gyroy', 'IMU gyroz', 'ref gyrox', 'ref gyroy', 'ref gyroz')
grid on

%% simulate GNSS data
initGNSSModel
GNSS.lat_GNSS = deg2rad(single(table2array(data(3:end,16))));  % deg -> rad
GNSS.lon_GNSS = deg2rad(single(table2array(data(3:end,17))));  % deg -> rad

% set data range in time (seconds)
% data_range = (22 <= t) & (t <= 35);
% data_range = (22 <= t) & (t <= 160);
% data_range = (65 <= t) & (t <= 75);
% data_range = (109 <= t) & (t <= 120);
% data_range = (144 <= t) & (t <= 160);
% data_range = (169 <= t) & (t <= 177);
% data_range = (8 <= t) & (t <= 160);

data_range = (0 <= t) & (t <= 100);

%% initialize
% time step (s)
T = 0.025;

% earth rotational rate (rad/s)
w_ie = 7.292115e-5;

% assume gravity to be constant (m/s^2)
g = -9.80743;

% set data range
range_start = find(data_range,1,'first');
range_end = find(data_range,1,'last');
k = range_start;

% remember to manually set the initial states for INS first!
initINS  % must init INS before init KF!!!
initLOG
initKF

%% navigation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%% PAY ATTENTION TO ORIENTATION OF THE BODY FRAME %%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

while k <= range_end
    % strapdown solution
    INSstep
    
    % KF prediction
    KFpredict

    % KF correction
    KFcorrect

    % increment step
    k = k + 1;

end

%% plot
close all

% roll pitch yaw
figure('Name', 'roll-pitch-yaw')
subplot(3, 1, 1)
hold on
grid on
plot(t(data_range), LOG.INS.eul_b0b(:, 1), t(data_range), Yaw(data_range) + YawOffset)
legend('INS-yaw', 'sensor-yaw')

subplot(3, 1, 2)
hold on
grid on
plot(t(data_range), LOG.INS.eul_b0b(:, 2), t(data_range), Pitch(data_range))
legend('INS-pitch', 'sensor-pitch')

subplot(3, 1, 3)
hold on
grid on
plot(t(data_range), LOG.INS.eul_b0b(:, 3), t(data_range), Roll(data_range))
legend('INS-roll', 'sensor-roll')

% INS
figure('Name', 'INS')
subplot(4,1,1);
plot(t(data_range), LOG.INS.eul_b0b(:, 1), t(data_range), Yaw(data_range) + YawOffset,...
     t(data_range), 360-heading(data_range))
title('yaw')
legend('INS-yaw', 'sensor-yaw')
grid on
hold on

subplot(4,1,2);
plot(t(data_range), LOG.INS.v_eb_n(:, 1), '.', t(data_range), LOG.INS.v_eb_n(:, 2), '.',...
     t(data_range), ref.VeloN(data_range), t(data_range), ref.VeloE(data_range))
title('velocity')
legend('Vn INS', 'Ve INS', 'Vn MTi7', 'Ve MTi7')
grid on
hold on

subplot(4,1,3);
plot(t(data_range), LOG.INS.llh_incre_total(:,1) + INS.lat0_frac - (GNSS.lat_GNSS(data_range) * llh_scale - INS.lat0_int))
title('position')
legend('lat (milli rad)')
grid on
hold on

subplot(4,1,4);
plot(t(data_range), LOG.INS.llh_incre_total(:,2) + INS.lon0_frac - (GNSS.lon_GNSS(data_range) * llh_scale - INS.lon0_int))
title('position')
legend('lon (milli rad)')
grid on
hold on

% Kalman Filter
figure('Name', 'navigation states error KF')
subplot(3, 1, 1)
hold on
grid on
plot(t(data_range), rad2deg(LOG.KF.dpsi_nb(:, 1)))
plot(t(data_range), rad2deg(LOG.KF.dpsi_nb(:, 2)))
plot(t(data_range), rad2deg(LOG.KF.dpsi_nb(:, 3)))
legend('d roll', 'd pitch', 'd yaw')
ylabel('deg')

subplot(3, 1, 2)
hold on
grid on
plot(t(data_range), LOG.KF.dv_eb_n(:, 1))
plot(t(data_range), LOG.KF.dv_eb_n(:, 2))
% plot(t(data_range), LOG.KF.dv_eb_n(:, 3))
legend('d vn', 'd ne')

subplot(3, 1, 3)
hold on
grid on
plot(t(data_range), LOG.KF.dllh(:, 1))
plot(t(data_range), LOG.KF.dllh(:, 2))
% plot(t(data_range), LOG.KF.dllh(:, 3))
legend('d lat', 'd lon')

figure('Name', 'IMU sensor error KF')
subplot(2, 1, 1)
hold on
grid on
plot(t(data_range), LOG.KF.ba(:, 1))
plot(t(data_range), LOG.KF.ba(:, 2))
plot(t(data_range), LOG.KF.ba(:, 3))
legend('ba x', 'ba y', 'ba z')

subplot(2, 1, 2)
hold on
grid on
plot(t(data_range), LOG.KF.bg(:, 1))
plot(t(data_range), LOG.KF.bg(:, 2))
plot(t(data_range), LOG.KF.bg(:, 3))
legend('bg x', 'bg y', 'bg z')

% INS corrected
figure('Name', 'INS corrected')
subplot(4,1,1);
plot(t(data_range), LOG.INS.eul_b0b_corrected(:, 1),...
     t(data_range), Yaw(data_range) + YawOffset,...
     t(data_range), 360-heading(data_range))
title('yaw')
legend('INS-corrected yaw', 'sensor yaw', 'MTi-7 yaw')
grid on
hold on

subplot(4,1,2);
plot(t(data_range), LOG.INS.v_eb_n_corrected(:, 1), '.',...
     t(data_range), LOG.INS.v_eb_n_corrected(:, 2), '.',...
     t(data_range), ref.VeloN(data_range),...
     t(data_range), ref.VeloE(data_range))
title('velocity')
legend('Vn INS corrected', 'Ve INS corrected', 'Vn MTi7', 'Ve MTi7')
ylim([-4 4])
grid on
hold on

subplot(4,1,3);
plot(t(data_range),...
     LOG.INS.llh_incre_total_corrected(:,1) + INS.lat0_frac...
     - (GNSS.lat_GNSS(data_range) * llh_scale - INS.lat0_int))
title('position')
legend('lat (milli rad)')
grid on
hold on

subplot(4,1,4);
plot(t(data_range),...
     LOG.INS.llh_incre_total_corrected(:,2) + INS.lon0_frac...
     - (GNSS.lon_GNSS(data_range) * llh_scale - INS.lon0_int))
title('position')
legend('lon (milli rad)')
grid on
hold on

figure('Name', 'covariance')
subplot(2,1,1)
grid on
hold on
plot(t(data_range), LOG.KF.P(:, 3,3))
legend('yaw error variance')

subplot(2,1,2)
grid on
hold on
plot(t(data_range), LOG.KF.P(:, 13,13))
legend('bg z variance')

% figure('Name', 'navigation states error KF')
% subplot(3, 1, 1)
% hold on
% grid on
% plot(rad2deg(LOG.KF.dpsi_nb(:, 1)))
% plot(rad2deg(LOG.KF.dpsi_nb(:, 2)))
% plot(rad2deg(LOG.KF.dpsi_nb(:, 3)))
% legend('d roll', 'd pitch', 'd yaw')
% ylabel('deg')
% 
% subplot(3, 1, 2)
% hold on
% grid on
% plot(LOG.KF.dv_eb_n(:, 1))
% plot(LOG.KF.dv_eb_n(:, 2))
% plot(LOG.KF.dv_eb_n(:, 3))
% legend('d vn', 'd ne', 'd vd')
% 
% subplot(3, 1, 3)
% hold on
% grid on
% plot(LOG.KF.dllh(:, 1))
% plot(LOG.KF.dllh(:, 2))
% plot(LOG.KF.dllh(:, 3))
% legend('d lat', 'd lon', 'd h')
% 
% figure('Name', 'IMU sensor error KF')
% subplot(2, 1, 1)
% hold on
% grid on
% plot(LOG.KF.ba(:, 1))
% plot(LOG.KF.ba(:, 2))
% plot(LOG.KF.ba(:, 3))
% legend('ba x', 'ba y', 'ba z')
% 
% subplot(2, 1, 2)
% hold on
% grid on
% plot(LOG.KF.bg(:, 1))
% plot(LOG.KF.bg(:, 2))
% plot(LOG.KF.bg(:, 3))
% legend('bg x', 'bg y', 'bg z')

% figure('Name', 'P')
% plot(k)
