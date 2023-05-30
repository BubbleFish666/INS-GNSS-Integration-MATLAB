%% KF correction
% calculate measurement y with only fractional parts to avoid numerical issue
KF.lat_GNSS_frac = GNSS.lat_GNSS(k) * llh_scale - INS.lat0_int;
KF.lon_GNSS_frac = GNSS.lon_GNSS(k) * llh_scale - INS.lon0_int;

KF.y = [GNSS.vn_GNSS(k); GNSS.ve_GNSS(k); KF.lat_GNSS_frac; KF.lon_GNSS_frac]...
     - [INS.v_eb_n_fedback(1);
        INS.v_eb_n_fedback(2);
        INS.lat0_frac + INS.lat_incre_total_fedback;
        INS.lon0_frac + INS.lon_incre_total_fedback];  % milli rad

KF.z_pre = [KF.dpsi_nb; KF.dv_eb_n; KF.dllh; KF.ba; KF.bg];

% covariance of innovation
KF.S = KF.H * KF.P * KF.H' + KF.R;
% Kalman Gain
KF.K = KF.P * KF.H' * KF.S^(-1);
% update z
KF.z = KF.z_pre + KF.K * (KF.y - KF.H * KF.z_pre);

KF.dpsi_nb = KF.z(1:3);
KF.Rnn = R3(KF.dpsi_nb(3)) * R2(KF.dpsi_nb(2)) * R1(KF.dpsi_nb(1));
KF.dv_eb_n = KF.z(4:5);
KF.dllh = KF.z(6:7);
KF.ba = KF.z(8:10);
KF.bg = KF.z(11:13);

KF.P = (eye(13) - KF.K * KF.H) * KF.P * (eye(13) - KF.K * KF.H)'...
       + KF.K * KF.R * KF.K';

% % correct the INS
% INS.llh_incre_total_corrected = [INS.lat_incre_total + KF.dllh(1);
%                                  INS.lon_incre_total + KF.dllh(2);
%                                  0];
% INS.llh_corrected = [INS.lat0 + INS.llh_incre_total_corrected(1);
%                      INS.lon0 + INS.llh_incre_total_corrected(2);
%                      0];
% 
% INS.v_eb_n_corrected = INS.v_eb_n + KF.dv_eb_n;
% 
% INS.eul_nb_corrected = rotm2eul(KF.Rnn * INS.Rnb) * 180 / pi;
% % INS.eul_nb_corrected(1) = changeDegRange360(INS.eul_nb_corrected(1));
% 
% INS.eul_b0b_corrected = rotm2eul(INS.Rnb0' * KF.Rnn * INS.Rnb) * 180 / pi;
% % INS.eul_b0b_corrected(1) = changeDegRange360(INS.eul_b0b_corrected(1));

% correct the fedback INS
INS.llh_incre_total_corrected = [INS.lat_incre_total_fedback + KF.dllh(1);
                                 INS.lon_incre_total_fedback + KF.dllh(2);
                                 0];
INS.llh_corrected = [INS.lat0 + INS.llh_incre_total_corrected(1);
                     INS.lon0 + INS.llh_incre_total_corrected(2);
                     0];

INS.v_eb_n_corrected = INS.v_eb_n_fedback + KF.dv_eb_n;

% INS.eul_nb_corrected = rotm2eul(KF.Rnn * INS.Rnb_fedback) * 180 / pi;
% % INS.eul_nb_corrected(1) = changeDegRange360(INS.eul_nb_corrected(1));

INS.eul_b0b_corrected = rotm2eul(INS.Rnb0' * KF.Rnn * INS.Rnb_fedback) * 180 / pi;
% INS.eul_b0b_corrected(1) = changeDegRange360(INS.eul_b0b_corrected(1));

%% feed back corrections to INS
INS.Rnb_fedback = KF.Rnn * INS.Rnb_fedback;
INS.v_eb_n_fedback = INS.v_eb_n_fedback + KF.dv_eb_n;
INS.lat_incre_total_fedback = INS.lat_incre_total_fedback + KF.dllh(1);
INS.lon_incre_total_fedback = INS.lon_incre_total_fedback + KF.dllh(2);
INS.ba = INS.ba + KF.ba;
INS.bg = INS.bg + KF.bg;

% log KF error states
LOG.KF.dpsi_nb(k - range_start + 1, :) = KF.dpsi_nb;
LOG.KF.dv_eb_n(k - range_start + 1, :) = KF.dv_eb_n;
LOG.KF.dllh(k - range_start + 1, :) = KF.dllh;
LOG.KF.ba(k - range_start + 1, :) = KF.ba;
LOG.KF.bg(k - range_start + 1, :) = KF.bg;
LOG.KF.P(k - range_start + 1, :, :) = KF.P;

% fed back errors are zeroed
KF.dpsi_nb = [0;0;0];
KF.dv_eb_n = [0;0];
KF.dllh = [0;0];
KF.ba = [0;0;0];
KF.bg = [0;0;0];

%% logging
% LOG.KF.dpsi_nb(2 * (k - range_start + 1), :) = KF.dpsi_nb;
% LOG.KF.dv_eb_n(2 * (k - range_start + 1), :) = KF.dv_eb_n;
% LOG.KF.dllh(2 * (k - range_start + 1), :) = KF.dllh;
% LOG.KF.ba(2 * (k - range_start + 1), :) = KF.ba;
% LOG.KF.bg(2 * (k - range_start + 1), :) = KF.bg;
% LOG.KF.P{2 * (k - range_start + 1)} = KF.P;

% INS corrected
% LOG.INS.eul_nb_corrected(k - range_start + 1, :) = INS.eul_nb_corrected;
LOG.INS.eul_b0b_corrected(k - range_start + 1, :) = INS.eul_b0b_corrected;
LOG.INS.v_eb_n_corrected(k - range_start + 1, :) = INS.v_eb_n_corrected;
LOG.INS.llh_incre_total_corrected(k - range_start + 1, :) = INS.llh_incre_total_corrected;
LOG.INS.llh_corrected(k - range_start + 1, :) = INS.llh_corrected;
