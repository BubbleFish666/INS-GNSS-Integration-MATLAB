%% KF correction
% calculate measurement y with only fractional parts to avoid numerical
% issue
KF.lat_GNSS_frac = GNSS.lat_GNSS(k) * llh_scale - INS.lat0_int;
KF.lon_GNSS_frac = GNSS.lon_GNSS(k) * llh_scale - INS.lon0_int;
KF.y = [KF.lat_GNSS_frac; KF.lon_GNSS_frac]...
       - [INS.lat0_frac + INS.lat_incre_total; INS.lon0_frac + INS.lon_incre_total];
KF.z_pre = [KF.dpsi_nb; KF.dv_eb_n; KF.dllh; KF.ba; KF.bg];

% covariance of innovation
KF.S = KF.H * KF.P * KF.H' + KF.R;
% Kalman Gain
KF.K = KF.P * KF.H' * KF.S^(-1);
% update z
KF.z = KF.z_pre + KF.K * (KF.y - KF.H * KF.z_pre);

KF.dpsi_nb = KF.z(1:3);
KF.dv_eb_n = KF.z(4:6);
KF.dllh = KF.z(7:9);
KF.ba = KF.z(10:12);
KF.bg = KF.z(13:15);

% dpsi_nb = [0; 0; 0];  % attitude error
% dv_eb_n = [0; 0; 0];  % velocity error
% dllh = [0; 0; 0];  % position error
% ba = [0; 0; 0];  % accelorometer error
% bg = [0; 0; 0];  % gyro error

KF.P = (eye(15) - KF.K * KF.H) * KF.P * (eye(15) - KF.K * KF.H)' + KF.K * KF.R * KF.K';