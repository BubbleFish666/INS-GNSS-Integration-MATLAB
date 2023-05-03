% initial orientation
% Rnb = eye(3);
INS.psi0 = deg2rad(360 - heading(range_start));  % yaw
INS.theta0 = 0;  % pitch
INS.phi0 = 0;  % roll
% Rnb0 = [0, 1, 0; 1, 0, 0; 0, 0, -1];  % n-frame to original b-frame
% Rnb0 = [1, 0, 0; 0, -1, 0; 0, 0, -1];  % n-frame to original b-frame
INS.Rnb0 = [-1, 0, 0; 0, 1, 0; 0, 0, -1];  % n-frame to original b-frame
INS.Rb0b = R3(INS.psi0)*R2(INS.theta0)*R1(INS.phi0);  % b-frame to original b-frame
INS.Rnb = INS.Rnb0 * INS.Rb0b;  % n-frame to b-frame

% initial velocity
INS.v_eb_n = [0; 0; 0];

% initial latitude, longitude (degree) and height (m)
% lat0 = single(deg2rad(48.1351));  % lat Munich
% lon0 = single(deg2rad(11.5820));  % lon Munich
INS.lat0 = single(deg2rad(49.065972574));
INS.lon0 = single(deg2rad(9.260714896));
INS.h0 = 520;  % height Munich
% scale latutude and longitude
llh_scale = 1e3;  % milli rad
INS.lat0 = INS.lat0 * llh_scale;
INS.lon0 = INS.lon0 * llh_scale;

INS.lat_incre_total = 0;
INS.lon_incre_total = 0;
INS.lat = INS.lat0;
INS.lon = INS.lon0;
INS.lat0_int = single(int32(INS.lat0));
INS.lon0_int = single(int32(INS.lon0));
INS.lat0_frac = INS.lat0 - INS.lat0_int;
INS.lon0_frac = INS.lon0 - INS.lon0_int;

INS.h = INS.h0;