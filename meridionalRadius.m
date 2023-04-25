function [mRadius] = meridionalRadius(lat)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
b = 6356752.3131;
a = 6378137;
e2 = (a^2-b^2)/a^2;
mRadius = b^2 / (a*(1-e2*sin(lat)^2)^(3/2));
end

