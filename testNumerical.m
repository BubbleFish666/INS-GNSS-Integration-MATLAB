%% set trajectory
% load data
clear; close all;
% data = readtable("Frames.xlsx");
% t = table2array(data(3:end,1));
% t = datenum(t, 'HH:MM:SS,FFF');
% t = single((t - t(1)) * 86400);
data = readtable("Frames_t.xlsx");
t = table2array(data(3:end,1));

% GPS data
% lat_GNSS = deg2rad(single(table2array(data(3:end,16))));  % deg -> rad
lat_GNSS = table2array(data(3:end,16));  % deg -> rad
lat_GNSS = single(lat_GNSS);
lon_GNSS = deg2rad(single(table2array(data(3:end,17))));  % deg -> rad

% set data range in time (seconds)
% data_range = (22 <= t) & (t <= 35);
data_range = (65 <= t) & (t <= 75);
% data_range = (109 <= t) & (t <= 120);
% data_range = (144 <= t) & (t <= 160);
% data_range = (169 <= t) & (t <= 177);

%%
plot(t(data_range), lat_GNSS(data_range))
title('lat GNSS')
grid on
%% initialize

% initial latitude, longitude (degree) and height (m)
lat0_deg = 49.065972574;
lon0_deg = 9.260714896;
lat0_deg_int = single(int32(lat0_deg));
lon0_deg_int = single(int32(lon0_deg));
lat0_deg_frac = lat0_deg - lat0_deg_int;
lon0_deg_frac = lon0_deg - lon0_deg_int;

lat0 = single(deg2rad(lat0_deg_int) * 1000);
lon0 = single(deg2rad(lon0_deg_int) * 1000);
lat0_int = single(int32(lat0));
lon0_int = single(int32(lon0));

lat0_frac = single(deg2rad(lat0_deg_frac));
lat0_frac = lat0_frac + (lat0 - lat0_int);

lon0_frac = single(deg2rad(lon0_deg_frac));
lon0_frac = lon0_frac + (lon0 - lon0_int);

h0 = 520;
%%
% scale latutude and longitude
llh_scale = 1e3;  % milli rad
% llh_scale = 1e6;  % micro rad
% lat0 = lat0 * llh_scale;
% lon0 = lon0 * llh_scale;

%%
% lat_incre_total = 0;
% lon_incre_total = 0;
% lat = lat0;
% lon = lon0;
% lat0_int = single(int32(lat0));
% lon0_int = single(int32(lon0));
% lat0_frac = lat0 - lat0_int;
% lon0_frac = lon0 - lon0_int;

% h = h0;

range_start = find(data_range,1,'first');
range_end = find(data_range,1,'last');
k = range_start;

LOG.y(1:range_end-range_start+1, 1:2) = nan;
LOG.ll_frac(1:range_end-range_start+1, 1:2) = nan;

%% 
while k <= range_end
    % pay attention to numerical issues,
    % consider milli rad and split int part and decimal part
    % subtraction of 2 nearly equal numbers may cause numerical problem
    lat_frac = lat_GNSS(k) * llh_scale - lat0_int;
    lon_frac = lon_GNSS(k) * llh_scale - lon0_int;
    y = [lat_frac; lon_frac] - [lat0_frac; lon0_frac];
    
    LOG.ll_frac(k-range_start+1, :) = [lat_frac, lon_frac];
    LOG.y(k-range_start+1, :) = [y(1), y(2)];

    k = k + 1;
end

subplot(2,1,1);
plot(t(data_range), LOG.ll_frac(:, 1))
title('lat')
legend('lat frac')
% xticks(0:10:200)
grid on
hold on

subplot(2,1,2);
plot(t(data_range), LOG.y(:, 1))
title('lat')
legend('lat frac - lat0 frac')
% xticks(0:10:200)
grid on
hold on