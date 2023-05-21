% error states
KF.dpsi_nb = [0; 0; 0];  % attitude error (rad) numerical issue?
KF.Rnn = zeros(3);
% KF.dv_eb_n = [0; 0; 0];  % velocity error (m/s)
% KF.dllh = [0; 0; 0];  % position error (milli rad)
KF.dv_eb_n = [0; 0];  % velocity error (m/s)
KF.dllh = [0; 0];  % position error (milli rad)
KF.ba = [0; 0; 0];  % accelorometer error (m/s^2)
KF.bg = [0; 0; 0];  % gyro error (rad/s)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%% PAY ATTENTION TO SCALING %%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% covariance of estimate
% KF.P = zeros(15);
% KF.P(1,1) = 0.002^2;  % roll variance (0.002 rad)^2
% KF.P(2,2) = 0.002^2;  % pitch variance (0.002 rad)^2
% KF.P(3,3) = 0.02^2;  % yaw variance (0.02 rad)^2
% KF.P(4,4) = 1.0^2;  % north velocity variance (1.0 m/s)^2
% KF.P(5,5) = 1.0^2;  % east velocity variance (1.0 m/s)^2
% KF.P(6,6) = 0;  % vertical velocity variance
% KF.P(7,7) = (2.0 * llh_scale / meridionalRadius(INS.lat0 / llh_scale))^2;  % GNSS horizontal accuracy 2m -> lat (milli rad)^2
% KF.P(8,8) = (2.0 * llh_scale / transverseRadius(INS.lat0 / llh_scale))^2;  % GNSS horizontal accuracy 2m -> lon (milli rad)^2
% KF.P(9,9) = 0;  % height
% KF.P(10,10) = (0.03 * 10 * 1e-3)^2;  % accelorometer bias (0.03 mg)^2
% KF.P(11,11) = (0.03 * 10 * 1e-3)^2;  % accelorometer bias (0.03 mg)^2
% KF.P(12,12) = (0.03 * 10 * 1e-3)^2;  % accelorometer bias (0.03 mg)^2
% KF.P(13,13) = (deg2rad(8) / 3600)^2;  % gyro bias (8 deg/h)^2
% KF.P(14,14) = (deg2rad(8) / 3600)^2;  % gyro bias (8 deg/h)^2
% KF.P(15,15) = (deg2rad(8) / 3600)^2;  % gyro bias (8 deg/h)^2

KF.P = zeros(13);
% KF.P(1,1) = deg2rad(8)^2;  % roll variance
% KF.P(2,2) = deg2rad(8)^2;  % pitch variance
KF.P(3,3) = deg2rad(8)^2;  % yaw variance
KF.P(4,4) = 0.5^2;  % north velocity variance (1.0 m/s)^2
KF.P(5,5) = 0.5^2;  % east velocity variance (1.0 m/s)^2
KF.P(6,6) = (4.0 * llh_scale / meridionalRadius(INS.lat0 / llh_scale))^2;  % GNSS horizontal accuracy 2m -> lat (milli rad)^2
KF.P(7,7) = (4.0 * llh_scale / transverseRadius(INS.lat0 / llh_scale) / cos(INS.lat0 / llh_scale))^2;  % GNSS horizontal accuracy 2m -> lon (milli rad)^2
KF.P(8,8) = (0.03e-3 * 10)^2;  % accelorometer bias (0.03 mg)^2
KF.P(9,9) = (0.03e-3 * 10)^2;  % accelorometer bias (0.03 mg)^2
% KF.P(10,10) = (0.03 * 10 * 1e-3)^2;  % accelorometer bias (0.03 mg)^2
% KF.P(11,11) = (deg2rad(10) / 3600)^2;  % gyro bias (8 deg/h)^2
% KF.P(12,12) = (deg2rad(10) / 3600)^2;  % gyro bias (8 deg/h)^2
KF.P(13,13) = (deg2rad(10) / 3600)^2;  % gyro bias (8 deg/h)^2

% measurement matrix
% KF.H = [zeros(2,3), zeros(2,3), [eye(2), zeros(2,1)], zeros(2,3), zeros(2,3)];
% KF.H = [zeros(2,3), zeros(2), eye(2), zeros(2,3), zeros(2,3)];
KF.H = [zeros(2,3), eye(2), zeros(2), zeros(2,3), zeros(2,3);
        zeros(2,3), zeros(2), eye(2), zeros(2,3), zeros(2,3)];

% system noise
% KF.Q = zeros(15);
% KF.Q(4:6 , 4:6) = (1.6 * 10 * 1e-3 * T)^2 * eye(3);  % velocity error from accelorometer noise 1.6 mg
% KF.Q(1:3 , 1:3) = (deg2rad(0.06) * T)^2 * eye(3);  % attitude error from gyro noise 0.06 deg/s

KF.Q = zeros(13);
% KF.Q(1:3, 1:3) = (10 * deg2rad(0.007))^2 * T * eye(3);  % gyro noise 0.007 deg/s/sqr(Hz)
% KF.Q(4:5, 4:5) = (50 * 120e-6 * 10)^2 * T * eye(2);  % accelorometer noise 120 ug/sqr(Hz)
% KF.Q(8:10, 8:10) = (50 * 0.04e-3 * 10)^2 * eye(3);  % accelerometer dynamic bias 0.04 mg
% KF.Q(11:13, 11:13) = (10 * deg2rad(10) / 3600)^2 * eye(3);  % gyro dynamic bias 10 deg/h
KF.Q(3 ,3) = (10 * deg2rad(0.007))^2 * T;  % gyro noise 0.007 deg/s/sqr(Hz)
KF.Q(4:5 , 4:5) = (50 * 120e-6 * 10)^2 * T * eye(2);  % accelorometer noise 120 ug/sqr(Hz)
KF.Q(8:9, 8:9) = (50 * 0.04e-3 * 10)^2 * eye(2);  % accelerometer dynamic bias 0.04 mg
KF.Q(13, 13) = (10 * deg2rad(10) / 3600)^2;  % gyro dynamic bias 10 deg/h

% measurement noise
% error from GNSS 2.0m horizontal (only consider GNSS horizontal data)
% KF.R = [(2.0 * llh_scale / meridionalRadius(INS.lat / llh_scale))^2, 0;
%         0, (2.0 * llh_scale / transverseRadius(INS.lat / llh_scale) / cos(INS.lat / llh_scale))^2];
KF.R = [0.5^2, 0, 0, 0;
        0, 0.5^2, 0, 0;
        0, 0, (4.0 * llh_scale / meridionalRadius(INS.lat / llh_scale))^2, 0;
        0, 0, 0, (4.0 * llh_scale / transverseRadius(INS.lat / llh_scale) / cos(INS.lat / llh_scale))^2];
