%% KF correction
% calculate measurement y with only fractional parts to avoid numerical issue
KF.lat_GNSS_frac = GNSS.lat_GNSS(k) * llh_scale - INS.lat0_int;
KF.lon_GNSS_frac = GNSS.lon_GNSS(k) * llh_scale - INS.lon0_int;

KF.z = [GNSS.vn_GNSS(k); GNSS.ve_GNSS(k); KF.lat_GNSS_frac; KF.lon_GNSS_frac]...
     - [INS.v_eb_n(1);
        INS.v_eb_n(2);
        INS.lat0_frac + INS.lat_incre_total;
        INS.lon0_frac + INS.lon_incre_total];  % milli rad

KF.x_pre = [KF.dpsi_nb; KF.dv_eb_n; KF.dllh; KF.ba; KF.bg];

% covariance of innovation
KF.S = KF.H * KF.P * KF.H' + KF.R;
% Kalman Gain
KF.K = KF.P * KF.H' * KF.S^(-1);
% update x
KF.x = KF.x_pre + KF.K * (KF.z - KF.H * KF.x_pre);

KF.dpsi_nb = KF.x(1:3);
KF.Rnn = R3(KF.dpsi_nb(3)) * R2(KF.dpsi_nb(2)) * R1(KF.dpsi_nb(1));
KF.dv_eb_n = KF.x(4:5);
KF.dllh = KF.x(6:7);
KF.ba = KF.x(8:10);
KF.bg = KF.x(11:13);

KF.P = (eye(13) - KF.K * KF.H) * KF.P * (eye(13) - KF.K * KF.H)'...
       + KF.K * KF.R * KF.K';

% correct the INS
INS.llh_incre_total_corrected = [INS.lat_incre_total + KF.dllh(1);
                                 INS.lon_incre_total + KF.dllh(2);
                                 0];
INS.llh_corrected = [INS.lat0 + INS.llh_incre_total_corrected(1);
                     INS.lon0 + INS.llh_incre_total_corrected(2);
                     0];

INS.v_eb_n_corrected = INS.v_eb_n + KF.dv_eb_n;

INS.eul_nb_corrected = rotm2eul(KF.Rnn * INS.Rnb) * 180 / pi;
% INS.eul_nb_corrected(1) = changeDegRange360(INS.eul_nb_corrected(1));

INS.eul_b0b_corrected = rotm2eul(INS.Rnb0' * KF.Rnn * INS.Rnb) * 180 / pi;
% INS.eul_b0b_corrected(1) = changeDegRange360(INS.eul_b0b_corrected(1));

%% logging
% KF error states
LOG.KF.dpsi_nb(k - range_start + 1, :) = KF.dpsi_nb;
LOG.KF.dv_eb_n(k - range_start + 1, :) = KF.dv_eb_n;
LOG.KF.dllh(k - range_start + 1, :) = KF.dllh;
LOG.KF.ba(k - range_start + 1, :) = KF.ba;
LOG.KF.bg(k - range_start + 1, :) = KF.bg;
LOG.KF.P(k - range_start + 1, :, :) = KF.P;

% INS corrected
LOG.INS.eul_nb_corrected(k - range_start + 1, :) = INS.eul_nb_corrected;
LOG.INS.eul_b0b_corrected(k - range_start + 1, :) = INS.eul_b0b_corrected;
LOG.INS.v_eb_n_corrected(k - range_start + 1, :) = INS.v_eb_n_corrected;
LOG.INS.llh_incre_total_corrected(k - range_start + 1, :) = INS.llh_incre_total_corrected;
LOG.INS.llh_corrected(k - range_start + 1, :) = INS.llh_corrected;
