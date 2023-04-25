function compass_degrees = invTanDegToCompassDeg(x)
% convert the inverse tangent in degrees [-180,180]
% to compass degrees [0,360]
% Note:
% compass degrees:
% 0 deg = N, 90 deg = E, 180 deg = S, 270 deg = W, 360 deg = N
% inverse tangent in degrees:
% 0 deg = +x, 90 deg = +y, 180 deg = -x, -90 deg = -y, -180 = -x
compass_degrees = mod((-x + 360 + 90), 360);
end