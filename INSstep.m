%% read data from IMU
INS.w_ib_b = [IMU.gyrox(k); IMU.gyroy(k); IMU.gyroz(k)];
INS.f_ib_b = [IMU.faccx(k); IMU.faccy(k); IMU.faccz(k)];

%% strapdown solution
% rotation
INS.Rnb_ = INS.Rnb;
INS.w_ibx_b = INS.w_ib_b(1);
INS.w_iby_b = INS.w_ib_b(2);
INS.w_ibz_b = INS.w_ib_b(3);
INS.omega_ib_b = [0, -INS.w_ibz_b, INS.w_iby_b;
                  INS.w_ibz_b, 0, -INS.w_ibx_b;
                  -INS.w_iby_b, INS.w_ibx_b, 0];

INS.omega_ie_n = w_ie * [0, sin(INS.lat / llh_scale), 0; 
                         -sin(INS.lat  / llh_scale), 0, -cos(INS.lat / llh_scale);
                         0, cos(INS.lat / llh_scale), 0];

% neglecting earth rotation
INS.Rnb = INS.Rnb_ * (eye(3) + INS.omega_ib_b * T);

% velocity
INS.v_eb_n_ = INS.v_eb_n;
INS.v_eb_n = INS.v_eb_n_ + INS.Rnb_(1:2, 1:3) * INS.f_ib_b * T;

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

%% logging
% rotm2eul returns angles in yaw - pitch - roll order (rad)
% rotation relative to local navigation frame
INS.eul_nb = rotm2eul(INS.Rnb) * 180 / pi;
% INS.eul_nb(1) = changeDegRange360(INS.eul_nb(1));
LOG.INS.eul_nb(k-range_start+1, :) = INS.eul_nb;

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
