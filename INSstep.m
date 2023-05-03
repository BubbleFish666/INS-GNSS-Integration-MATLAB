% read data from IMU
INS.w_ibx_b = IMU.gyrox(k);
INS.w_iby_b = IMU.gyroy(k);
INS.w_ibz_b = IMU.gyroz(k);
% note that fAcc are already compensated by sensor with gravity
INS.f_ib_b = [IMU.faccx(k); IMU.faccy(k); IMU.faccz(k)];

% rotation
INS.Rnb_ = INS.Rnb;
INS.omega_ib_b = [0, -INS.w_ibz_b, INS.w_iby_b;
                  INS.w_ibz_b, 0, -INS.w_ibx_b;
                  -INS.w_iby_b, INS.w_ibx_b, 0];

INS.omega_ie_n = w_ie * [0, sin(INS.lat / llh_scale), 0; 
                         -sin(INS.lat  / llh_scale), 0, -cos(INS.lat / llh_scale);
                         0, cos(INS.lat / llh_scale), 0];

INS.Rnb = INS.Rnb_ * (eye(3) + INS.omega_ib_b * T) - INS.omega_ie_n * INS.Rnb_ * T;

% velocity
INS.v_eb_n_ = INS.v_eb_n;
% v_eb_n = v_eb_n_ + (Rnb_ * f_ib_b + g - cross(2 * Ren' * w_ie, v_eb_n)) * T;
INS.v_eb_n = INS.v_eb_n_ + (INS.Rnb_ * INS.f_ib_b) * T;
INS.v_eb_n(3) = 0;  % disable velocity in Down direction

% position
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

% logging
% rotm2eul returns angles in yaw - pitch - roll order (rad)
INS.eul = rotm2eul(INS.Rnb) * 180 / pi;
LOG.INS.Rnb(k-range_start+1, :) = INS.eul;
LOG.INS.Rb0b(k-range_start+1, :) = rotm2eul(INS.Rnb0'*INS.Rnb) * 180 / pi;
LOG.INS.v_eb_n(k-range_start+1, :) = INS.v_eb_n;
LOG.INS.llh(k-range_start+1, :) = [INS.lat, INS.lon, INS.h];
LOG.INS.llh_incre_total(k-range_start+1, :) = [INS.lat_incre_total, INS.lon_incre_total, 0];
LOG.INS.llh_incre(k-range_start+1, :) = [INS.lat_incre, INS.lon_incre, 0];