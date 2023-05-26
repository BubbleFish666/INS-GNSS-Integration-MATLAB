%% load data
clear; close all;

% select reference trajectory
ref_traj = load("trajData600.mat");
% ref_traj = load("trajData200.mat");
t = ref_traj.timeVector(2:end);

% use saved sensor data
archive_data = true;
% data_name = 'datafusion_not_a_very_good_result.mat';
% data_name = 'datafusion_test_600.mat';
% data_name = 'datafusion_a_typical_result_600.mat';
% data_name = 'datafusion_nice_result_with_GNSS_velocity.mat';
% data_name = 'datafusion_nice_result_with_GNSS_velocity_larger_uncertainty.mat';
addpath("test_data\");
% data_name = 'test_data\datafusion_test_data_closed_loop_1.mat';
data_name = 'temp.mat';

% reference latitude and longitude
ref.lat = ref_traj.pos_geo_incre_log(:, 2) + ref_traj.lat0 * 1000;  % milli rad
ref.lon = ref_traj.pos_geo_incre_log(:, 1) + ref_traj.lon0 * 1000;  % milli rad
% reference velocity
ref.VeloE = ref_traj.vel_log(:, 1);  % m/s
ref.VeloN = ref_traj.vel_log(:, 2);  % m/s
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
if archive_data
    load(data_name, 'GNSS', 'IMU')
else
    initIMUModel
    [IMU_acc, IMU_gyro] = IMUModel([ref.accx, ref.accy, ref.accz],...
                                   [ref.gyrox, ref.gyroy, ref.gyroz]);
    IMU.faccx = -single(IMU_acc(:, 1));  % m/s^2
    IMU.faccy = -single(IMU_acc(:, 2));  % m/s^2
    IMU.faccz = -single(IMU_acc(:, 3) - 9.81);  % m/s^2
    IMU.gyrox = single(IMU_gyro(:, 1));  % rad/s
    IMU.gyroy = single(IMU_gyro(:, 2));  % rad/s
    IMU.gyroz = single(IMU_gyro(:, 3));  % rad/s

%% simulate GNSS data (in our case, GPS)
    initGPSModel
    [GPS_pos, GPS_vel, ~, ~] = GPSModel([rad2deg(ref.lat * 0.001), rad2deg(ref.lon * 0.001), zeros(size(ref.lat))],...
                                  [ref.VeloE, ref.VeloN, zeros(size(ref.VeloE))]);
    GNSS.lat_GNSS = deg2rad(GPS_pos(:, 1));  % deg -> rad
    GNSS.lon_GNSS = deg2rad(GPS_pos(:, 2));  % deg -> rad
    GNSS.ve_GNSS = GPS_vel(:, 1);  % m/s
    GNSS.vn_GNSS = GPS_vel(:, 2);  % m/s
end

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

%% plot simulated GNSS data
figure('Name', 'GNSS data')
subplot(3,1,1)
plot(t, GNSS.lat_GNSS * 1e3, t, ref.lat)
legend('GNSS lat', 'ref lat')
grid on
subplot(3,1,2)
plot(t, GNSS.lon_GNSS * 1e3, t, ref.lon)
legend('GNSS lon', 'ref lon')
grid on
subplot(3,1,3)
plot(t, GNSS.vn_GNSS, t, GNSS.ve_GNSS, ...
     t, ref.VeloN, t, ref.VeloE)
legend('GNSS Vn', 'GNSS Ve', 'ref Vn', 'ref Ve')
grid on

%% initialize
% set data range in time (seconds)
% data_range = (22 <= t) & (t <= 35);
% data_range = (22 <= t) & (t <= 160);
% data_range = (65 <= t) & (t <= 75);
% data_range = (109 <= t) & (t <= 120);
% data_range = (144 <= t) & (t <= 160);
% data_range = (169 <= t) & (t <= 177);
% data_range = (8 <= t) & (t <= 160);

data_range = 0 <= t;

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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
k = k + 1;  % to align the data with the ref traj
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
while k <= range_end
    % strapdown solution
    INSstep
    
    % KF prediction
    KFpredict

    % KF correction
    if mod(k, 10) == 0
        KFcorrect
    end

    % increment step
    k = k + 1;

end

%% plot result
plotNavResult
