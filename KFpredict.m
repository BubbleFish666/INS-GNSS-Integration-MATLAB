%% KF propagation
% attitude
KF.dpsi_nb_ = KF.dpsi_nb;
KF.dpsi_nb = KF.dpsi_nb_ + INS.Rnb * T * KF.bg;
KF.Rnn = R3(KF.dpsi_nb(3)) * R2(KF.dpsi_nb(2)) * R1(KF.dpsi_nb(1));

% velocity
KF.dv_eb_n_ = KF.dv_eb_n;
% KF.f_ib_n = INS.Rnb0 * INS.f_ib_b0 + [0; 0; -9.81];
KF.f_ib_n = INS.Rnb * (INS.f_ib_b + [0; 0; 9.81]);
% KF.F21n = [0, KF.f_ib_n(3), -KF.f_ib_n(2);
%            -KF.f_ib_n(3), 0, KF.f_ib_n(1);
%            KF.f_ib_n(2), -KF.f_ib_n(1), 0];
KF.F21n = [0, KF.f_ib_n(3), -KF.f_ib_n(2);
           -KF.f_ib_n(3), 0, KF.f_ib_n(1)];
% KF.F23n = zeros(3);
KF.F23n = zeros(2);
% KF.dv_eb_n = KF.dv_eb_n_ + KF.F21n * T * KF.dpsi_nb_...
%              + KF.F23n * T * KF.dllh + INS.Rnb * T * KF.ba;
% KF.dv_eb_n(3) = 0;  % disable velocity in Down direction
KF.dv_eb_n = KF.dv_eb_n_ + KF.F21n * T * KF.dpsi_nb_...
             + KF.F23n * T * KF.dllh + INS.Rnb(1:2, 1:3) * T * KF.ba;

% position
% KF.F32n = [1 * llh_scale / (meridionalRadius(INS.lat / llh_scale) + INS.h), 0, 0;
%            0, 1 * llh_scale / ((transverseRadius(INS.lat / llh_scale) + INS.h) * cos(INS.lat / llh_scale)), 0;
%            0, 0, -1];
KF.F32n = [1 * llh_scale / (meridionalRadius(INS.lat / llh_scale) + INS.h), 0;
           0, 1 * llh_scale / ((transverseRadius(INS.lat / llh_scale) + INS.h) * cos(INS.lat / llh_scale))];
KF.dllh = KF.dllh + KF.F32n * T * KF.dv_eb_n_;  % milli rad
% KF.dllh(3) = 0;  % disable height

% covariance of estimate
% KF.PHI = [eye(3), zeros(3), zeros(3), zeros(3), INS.Rnb * T;
%           KF.F21n * T, eye(3), KF.F23n * T, INS.Rnb * T, zeros(3);
%           zeros(3), KF.F32n * T, eye(3), zeros(3), zeros(3);
%           zeros(3), zeros(3), zeros(3), eye(3), zeros(3);
%           zeros(3), zeros(3), zeros(3), zeros(3), eye(3)];
KF.PHI = [eye(3), zeros(3,2), zeros(3,2), zeros(3), INS.Rnb * T;
          KF.F21n * T, eye(2), KF.F23n * T, INS.Rnb(1:2, 1:3) * T, zeros(2,3);
          zeros(2,3), KF.F32n * T, eye(2), zeros(2,3), zeros(2,3);
          zeros(3), zeros(3,2), zeros(3,2), eye(3), zeros(3);
          zeros(3), zeros(3,2), zeros(3,2), zeros(3), eye(3)];
KF.P = KF.PHI * KF.P * KF.PHI' + KF.Q;

%% logging
% LOG.KF.dpsi_nb(2 * (k - range_start) + 1, :) = KF.dpsi_nb;
% LOG.KF.dv_eb_n(2 * (k - range_start) + 1, :) = KF.dv_eb_n;
% LOG.KF.dllh(2 * (k - range_start) + 1, :) = KF.dllh;
% LOG.KF.ba(2 * (k - range_start) + 1, :) = KF.ba;
% LOG.KF.bg(2 * (k - range_start) + 1, :) = KF.bg;
% LOG.KF.P{2 * (k - range_start) + 1} = KF.P;
