function [R] = R2(theta)
%R2 Summary of this function goes here
%   Detailed explanation goes here
R = [cos(theta), 0, sin(theta);
     0, 1, 0;
     -sin(theta), 0, cos(theta)];
end

