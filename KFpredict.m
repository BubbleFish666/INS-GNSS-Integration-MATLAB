%% KF propagation
% attitude
KF.dpsi_nb_ = KF.dpsi_nb;
KF.dpsi_nb = KF.dpsi_nb_ + INS.Rnb_fedback * T * KF.bg;
KF.Rnn = R3(KF.dpsi_nb(3)) * R2(KF.dpsi_nb(2)) * R1(KF.dpsi_nb(1));

% velocity
KF.dv_eb_n_ = KF.dv_eb_n;
KF.f_ib_n = INS.Rnb_fedback * (INS.f_ib_b_fedback + [0; 0; 9.81]);
KF.F21n = [0, KF.f_ib_n(3), -KF.f_ib_n(2);
           -KF.f_ib_n(3), 0, KF.f_ib_n(1)];
KF.F23n = zeros(2);
KF.dv_eb_n = KF.dv_eb_n_ + KF.F21n * T * KF.dpsi_nb_...
             + KF.F23n * T * KF.dllh + INS.Rnb_fedback(1:2, 1:3) * T * KF.ba;

% position
KF.F32n = [1 * llh_scale / (meridionalRadius(INS.lat_fedback / llh_scale) + INS.h), 0;
           0, 1 * llh_scale / ((transverseRadius(INS.lat_fedback / llh_scale) + INS.h) * cos(INS.lat_fedback / llh_scale))];
KF.dllh = KF.dllh + KF.F32n * T * KF.dv_eb_n_;  % milli rad

% covariance of estimate
KF.PHI = [eye(3), zeros(3,2), zeros(3,2), zeros(3), INS.Rnb_fedback * T;
          KF.F21n * T, eye(2), KF.F23n * T, INS.Rnb_fedback(1:2, 1:3) * T, zeros(2,3);
          zeros(2,3), KF.F32n * T, eye(2), zeros(2,3), zeros(2,3);
          zeros(3), zeros(3,2), zeros(3,2), eye(3), zeros(3);
          zeros(3), zeros(3,2), zeros(3,2), zeros(3), eye(3)];
KF.P = KF.PHI * KF.P * KF.PHI' + KF.Q;

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


%% logging
% KF error states
LOG.KF.dpsi_nb(k - range_start + 1, :) = KF.dpsi_nb;
LOG.KF.dv_eb_n(k - range_start + 1, :) = KF.dv_eb_n;
LOG.KF.dllh(k - range_start + 1, :) = KF.dllh;
LOG.KF.ba(k - range_start + 1, :) = KF.ba;
LOG.KF.bg(k - range_start + 1, :) = KF.bg;
LOG.KF.P(k - range_start + 1, :, :) = KF.P;

% INS corrected
% LOG.INS.eul_nb_corrected(k - range_start + 1, :) = INS.eul_nb_corrected;
LOG.INS.eul_b0b_corrected(k - range_start + 1, :) = INS.eul_b0b_corrected;
LOG.INS.v_eb_n_corrected(k - range_start + 1, :) = INS.v_eb_n_corrected;
LOG.INS.llh_incre_total_corrected(k - range_start + 1, :) = INS.llh_incre_total_corrected;
LOG.INS.llh_corrected(k - range_start + 1, :) = INS.llh_corrected;
