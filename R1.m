function [R] = R1(phi)
%R1 Summary of this function goes here
%   Detailed explanation goes here
R = [1, 0, 0;
     0, cos(phi), -sin(phi);
     0, sin(phi), cos(phi)];
end

