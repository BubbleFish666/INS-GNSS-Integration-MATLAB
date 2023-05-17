%% initialize a GPS model
GPSModel = gpsSensor('ReferenceFrame','ENU');
GPSModel.SampleRate = 40;  % Hz

% When setting PositionInputFormat as 'Geodetic',
% the gpsSensor object neglects the ReferenceLocation property.
% lla0 = [rad2deg(ref_traj.pos_geo_incre_log(1, 2) * 0.001 + ref_traj.lat0),...
%         rad2deg(ref_traj.pos_geo_incre_log(1, 1) * 0.001 + ref_traj.lon0),...
%         0];
% GPSModel.ReferenceLocation = lla0;

GPSModel.PositionInputFormat = "Geodetic";  % takes llh as input
GPSModel.HorizontalPositionAccuracy = 4.0;  % meter
GPSModel.VerticalPositionAccuracy = 4.0;  % meter
GPSModel.VelocityAccuracy = 0.5;  % m/s
