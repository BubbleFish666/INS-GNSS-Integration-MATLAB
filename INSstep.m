%% read data from IMU
INS.w_ib_b = [IMU.gyrox(k); IMU.gyroy(k); IMU.gyroz(k)];
INS.w_ib_b_fedback = [IMU.gyrox(k); IMU.gyroy(k); IMU.gyroz(k)] + INS.bg;

% note that fAcc are already compensated by sensor with gravity
% it seems that the Free Acc is measured in a global floating frame
% With facc multiplied by R_board_facc (a quasi constant orientation
% offset), f_ib_b0 represents the acc measured in b0 frame.
% INS.f_ib_b0 = INS.R_board_facc * [IMU.faccx(k); IMU.faccy(k); IMU.faccz(k)];

INS.f_ib_b = [IMU.faccx(k); IMU.faccy(k); IMU.faccz(k)];
INS.f_ib_b_fedback = [IMU.faccx(k); IMU.faccy(k); IMU.faccz(k)] + INS.ba;

%% strapdown solution
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% rotation %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% raw INS
INS.Rnb_ = INS.Rnb;
INS.w_ibx_b = INS.w_ib_b(1);
INS.w_iby_b = INS.w_ib_b(2);
INS.w_ibz_b = INS.w_ib_b(3);
INS.omega_ib_b = [0, -INS.w_ibz_b, INS.w_iby_b;
                  INS.w_ibz_b, 0, -INS.w_ibx_b;
                  -INS.w_iby_b, INS.w_ibx_b, 0];

% INS.omega_ie_n = w_ie * [0, sin(INS.lat / llh_scale), 0; 
%                          -sin(INS.lat  / llh_scale), 0, -cos(INS.lat / llh_scale);
%                          0, cos(INS.lat / llh_scale), 0];

% INS.Rnb = INS.Rnb_ * (eye(3) + INS.omega_ib_b * T)...
%           - INS.omega_ie_n * INS.Rnb_ * T;
% neglecting earth rotation
INS.Rnb = INS.Rnb_ * (eye(3) + INS.omega_ib_b * T);

% fedback INS
INS.Rnb_fedback_ = INS.Rnb_fedback;
INS.w_ibx_b_fedback = INS.w_ib_b_fedback(1);
INS.w_iby_b_fedback = INS.w_ib_b_fedback(2);
INS.w_ibz_b_fedback = INS.w_ib_b_fedback(3);
INS.omega_ib_b_fedback = [0, -INS.w_ibz_b_fedback, INS.w_iby_b_fedback;
                  INS.w_ibz_b_fedback, 0, -INS.w_ibx_b_fedback;
                  -INS.w_iby_b_fedback, INS.w_ibx_b_fedback, 0];
INS.Rnb_fedback = INS.Rnb_fedback_ * (eye(3) + INS.omega_ib_b_fedback * T);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% velocity %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% raw INS
INS.v_eb_n_ = INS.v_eb_n;
% v_eb_n = v_eb_n_ + (Rnb_ * f_ib_b + g - cross(2 * Ren' * w_ie, v_eb_n)) * T;
% INS.v_eb_n = INS.v_eb_n_ + (INS.Rnb_ * INS.f_ib_b) * T;
% eul_tem = rotm2eul(INS.Rnb0' * INS.Rnb_);
% INS.v_eb_n = INS.v_eb_n_ + (INS.Rnb0 * R3(eul_tem(1)) * INS.f_ib_b) * T;
% INS.v_eb_n = INS.v_eb_n_ + INS.Rnb0 * INS.f_ib_b0 * T;
% INS.v_eb_n(3) = 0;  % disable velocity in Down direction
% INS.v_eb_n = INS.v_eb_n_ + INS.Rnb0(1:2, 1:3) * INS.f_ib_b0 * T;
% INS.v_eb_n = INS.v_eb_n_ + INS.Rnb_(1:2, 1:3) * INS.f_ib_b * T;
%%%%%%%%%%%%%% this v_incre doesn't make a diffrence as the above line, but
%%%%%%%%%%%%%% is more understandable
INS.v_incre = (INS.Rnb_ * INS.f_ib_b + [0; 0; 9.81]) * T;
INS.v_eb_n = INS.v_eb_n_ + INS.v_incre(1:2);

% fedback INS
INS.v_eb_n_fedback_ = INS.v_eb_n_fedback;
INS.v_incre_fedback = (INS.Rnb_fedback_ * INS.f_ib_b_fedback + [0; 0; 9.81]) * T;
INS.v_eb_n_fedback = INS.v_eb_n_fedback_ ...
                     + INS.v_incre_fedback(1:2);
                     % + INS.Rnb_fedback_(1:2, 1:3) * INS.f_ib_b_fedback * T;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% position %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% raw INS
% h = h;
INS.lat_ = INS.lat;
INS.lat_incre = (INS.v_eb_n_(1) / (meridionalRadius(INS.lat_ / llh_scale) + INS.h)...
                 + INS.v_eb_n(1) / (meridionalRadius(INS.lat_ / llh_scale) + INS.h))...
                * 0.5 * T * llh_scale;
INS.lat_incre_total = INS.lat_incre_total + INS.lat_incre;
INS.lat = INS.lat0 + INS.lat_incre_total;

INS.lon_incre = (INS.v_eb_n_(2) / ((transverseRadius(INS.lat_ / llh_scale) + INS.h) * cos(INS.lat_ / llh_scale))...
                + INS.v_eb_n(2) / ((transverseRadius(INS.lat / llh_scale) + INS.h) * cos(INS.lat / llh_scale)))...
                * 0.5 * T * llh_scale;
INS.lon_incre_total = INS.lon_incre_total + INS.lon_incre;
INS.lon = INS.lon0 + INS.lon_incre_total;

% fedback INS
INS.lat_fedback_ = INS.lat_fedback;
INS.lat_incre_fedback = (INS.v_eb_n_fedback_(1) / (meridionalRadius(INS.lat_fedback_ / llh_scale) + INS.h)...
                 + INS.v_eb_n_fedback(1) / (meridionalRadius(INS.lat_fedback_ / llh_scale) + INS.h))...
                 * 0.5 * T * llh_scale;
INS.lat_incre_total_fedback = INS.lat_incre_total_fedback + INS.lat_incre_fedback;
INS.lat_fedback = INS.lat0 + INS.lat_incre_total_fedback;

INS.lon_incre_fedback = (INS.v_eb_n_fedback_(2) / ((transverseRadius(INS.lat_fedback_ / llh_scale) + INS.h) * cos(INS.lat_fedback_ / llh_scale))...
                + INS.v_eb_n_fedback(2) / ((transverseRadius(INS.lat_fedback / llh_scale) + INS.h) * cos(INS.lat_fedback / llh_scale)))...
                * 0.5 * T * llh_scale;
INS.lon_incre_total_fedback = INS.lon_incre_total_fedback + INS.lon_incre_fedback;
INS.lon_fedback = INS.lon0 + INS.lon_incre_total_fedback;

%% logging
% rotm2eul returns angles in yaw - pitch - roll order (rad)
%%%%%%%%%%%%%%%%%%%%%%%%%%%% raw INS %%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % rotation relative to local navigation frame
% INS.eul_nb = rotm2eul(INS.Rnb) * 180 / pi;
% % INS.eul_nb(1) = changeDegRange360(INS.eul_nb(1));
% LOG.INS.eul_nb(k-range_start+1, :) = INS.eul_nb;

% rotation relative to original sensor frame
INS.eul_b0b = rotm2eul(INS.Rnb0'*INS.Rnb) * 180 / pi;
% INS.eul_b0b(1) = changeDegRange360(INS.eul_b0b(1));
LOG.INS.eul_b0b(k-range_start+1, :) = INS.eul_b0b;

% velocity in local navigation frame
LOG.INS.v_eb_n(k-range_start+1, :) = INS.v_eb_n;

% latitude and longtitude
LOG.INS.llh(k-range_start+1, :) = [INS.lat, INS.lon, INS.h];
LOG.INS.llh_incre_total(k-range_start+1, :) = [INS.lat_incre_total, INS.lon_incre_total, 0];
LOG.INS.llh_incre(k-range_start+1, :) = [INS.lat_incre, INS.lon_incre, 0];

%%%%%%%%%%%%%%%%%%%%%%%%%% fedback INS %%%%%%%%%%%%%%%%%%%%%%%%%%
% % rotation relative to local navigation frame
% INS.eul_nb_fedback = rotm2eul(INS.Rnb_fedback) * 180 / pi;
% % INS.eul_nb(1) = changeDegRange360(INS.eul_nb(1));
% LOG.INS.eul_nb_fedback(k-range_start+1, :) = INS.eul_nb_fedback;

% rotation relative to original sensor frame
INS.eul_b0b_fedback = rotm2eul(INS.Rnb0'*INS.Rnb_fedback) * 180 / pi;
% INS.eul_b0b(1) = changeDegRange360(INS.eul_b0b(1));
LOG.INS.eul_b0b_fedback(k-range_start+1, :) = INS.eul_b0b_fedback;

% velocity in local navigation frame
LOG.INS.v_eb_n_fedback(k-range_start+1, :) = INS.v_eb_n_fedback;

% latitude and longtitude
LOG.INS.llh_fedback(k-range_start+1, :) = [INS.lat_fedback, INS.lon_fedback, INS.h];
LOG.INS.llh_incre_total_fedback(k-range_start+1, :) = [INS.lat_incre_total_fedback, INS.lon_incre_total_fedback, 0];
LOG.INS.llh_incre_fedback(k-range_start+1, :) = [INS.lat_incre_fedback, INS.lon_incre_fedback, 0];

LOG.INS.ba(k-range_start+1, :) = INS.ba;
LOG.INS.bg(k-range_start+1, :) = INS.bg;
