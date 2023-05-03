%% KF propagation
% attitude
KF.dpsi_nb_ = KF.dpsi_nb;
KF.dpsi_nb = KF.dpsi_nb_ + INS.Rnb * T * KF.bg;

% velocity
KF.dv_eb_n_ = KF.dv_eb_n;
KF.f_ib_n = INS.Rnb * INS.f_ib_b;
KF.F21n = [0, KF.f_ib_n(3), -KF.f_ib_n(2);
           -KF.f_ib_n(3), 0, KF.f_ib_n(1);
           KF.f_ib_n(2), -KF.f_ib_n(1), 0];
KF.F23n = zeros(3);
KF.dv_eb_n = KF.dv_eb_n_ + KF.F21n * T * KF.dpsi_nb_...
             + KF.F23n * T * KF.dllh + INS.Rnb * T * KF.ba;
KF.dv_eb_n(3) = 0;  % disable velocity in Down direction

% position
KF.F32n = [1 / (meridionalRadius(INS.lat / llh_scale) + h), 0, 0;
           0, 1 / ((transverseRadius(INS.lat / llh_scale) + h) * cos(INS.lat / llh_scale)), 0;
           0, 0, -1];
KF.dllh = KF.dllh + KF.F32n * llh_scale * T * KF.dv_eb_n_;
KF.dllh(3) = 0;  % disable height

% covariance of estimate
KF.PHI = [eye(3), zeros(3), zeros(3), zeros(3), INS.Rnb * T;
          KF.F21n * T, eye(3), KF.F23n * T, INS.Rnb * T, zeros(3);
          zeros(3), KF.F32n * T, eye(3), zeros(3), zeros(3);
          zeros(3), zeros(3), zeros(3), eye(3), zeros(3);
          zeros(3), zeros(3), zeros(3), zeros(3), eye(3)];
KF.P = KF.PHI * KF.P * KF.PHI' + KF.Q;