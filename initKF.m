% error states
KF.dpsi_nb = single([0; 0; 0]);  % attitude error (rad)
KF.Rnn = single(zeros(3));
KF.dv_eb_n = single([0; 0]);  % velocity error (m/s)
KF.dllh = single([0; 0]);  % position error (milli rad)
KF.ba = single([0; 0; 0]);  % accelorometer error (m/s^2)
KF.bg = single([0; 0; 0]);  % gyro error (rad/s)

KF.P = single(zeros(13));
KF.P(1,1) = deg2rad(10)^2;  % roll variance
KF.P(2,2) = deg2rad(10)^2;  % pitch variance
KF.P(3,3) = deg2rad(10)^2;  % yaw variance
KF.P(4,4) = 0.5^2;  % north velocity variance (1.0 m/s)^2
KF.P(5,5) = 0.5^2;  % east velocity variance (1.0 m/s)^2
KF.P(6,6) = (2.5 * llh_scale / meridionalRadius(INS.lat0 / llh_scale))^2;  % GNSS horizontal accuracy 2m -> lat (milli rad)^2
KF.P(7,7) = (2.5 * llh_scale / transverseRadius(INS.lat0 / llh_scale) / cos(INS.lat0 / llh_scale))^2;  % GNSS horizontal accuracy 2m -> lon (milli rad)^2
KF.P(8,8) = (0.04e-3 * 10)^2;  % accelorometer bias (0.03 mg)^2
KF.P(9,9) = (0.04e-3 * 10)^2;  % accelorometer bias (0.03 mg)^2
KF.P(10,10) = (0.04 * 10 * 1e-3)^2;  % accelorometer bias (0.03 mg)^2
KF.P(11,11) = (deg2rad(10) / 3600)^2;  % gyro bias (8 deg/h)^2
KF.P(12,12) = (deg2rad(10) / 3600)^2;  % gyro bias (8 deg/h)^2
KF.P(13,13) = (deg2rad(10) / 3600)^2;  % gyro bias (8 deg/h)^2

% measurement matrix
KF.H = single([zeros(2,3), eye(2), zeros(2), zeros(2,3), zeros(2,3);
               zeros(2,3), zeros(2), eye(2), zeros(2,3), zeros(2,3)]);

% system noise
KF.Q = single(zeros(13));
KF.Q(1:3 , 1:3) = (3 * deg2rad(0.007))^2 * T * eye(3);  % gyro noise 0.007 deg/s/sqr(Hz)
KF.Q(4:5 , 4:5) = (3 * 120e-6 * 10)^2 * T * eye(2);  % accelorometer noise 120 ug/sqr(Hz)
KF.Q(8:10, 8:10) = (3 * 0.04e-3 * 10)^2 * eye(3);  % accelerometer dynamic bias 0.04 mg
KF.Q(11:13, 11:13) = (3 * deg2rad(10) / 3600)^2 * eye(3);  % gyro dynamic bias 10 deg/h

% measurement noise
% error from GNSS 2.0m horizontal (only consider GNSS horizontal data)
KF.R = single([0.05^2, 0, 0, 0;
        0, 0.05^2, 0, 0;
        0, 0, (2.5 * llh_scale / meridionalRadius(INS.lat / llh_scale))^2, 0;
        0, 0, 0, (2.5 * llh_scale / transverseRadius(INS.lat / llh_scale) / cos(INS.lat / llh_scale))^2]);
