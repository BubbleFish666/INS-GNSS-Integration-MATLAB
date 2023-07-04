% initial orientation
INS.psi0 = deg2rad(ref_traj.psi(1));  % yaw at 26.0 s
INS.theta0 = 0;  % pitch
INS.phi0 = 0;  % roll

% n-frame to MTi-3 board frame (body frame)
INS.Rnb0 = R3(0.5 * pi) * R1(pi);
% align yaw of MTi-3 frame to actual heading
INS.Rb0b = R3(INS.psi0)*R2(INS.theta0)*R1(INS.phi0);
INS.Rnb = INS.Rnb0 * INS.Rb0b;  % n-frame to b-frame

% initial velocity
INS.v_eb_n = [0; 0];

% initial latitude, longitude (rad) and height (m)
INS.lat0 = single(GNSS.lat_GNSS(range_start));
INS.lon0 = single(GNSS.lon_GNSS(range_start));
% INS.h0 = 520;  % height Munich
INS.h0 = 0;

% scale latutude and longitude
llh_scale = 1e3;  % milli rad
INS.lat0 = INS.lat0 * llh_scale;
INS.lon0 = INS.lon0 * llh_scale;

INS.lat_incre_total = 0;
INS.lon_incre_total = 0;
INS.lat = INS.lat0;
INS.lon = INS.lon0;
INS.lat0_int = single(floor(INS.lat0));
INS.lon0_int = single(floor(INS.lon0));
INS.lat0_frac = INS.lat0 - INS.lat0_int;
INS.lon0_frac = INS.lon0 - INS.lon0_int;

INS.h = INS.h0;
