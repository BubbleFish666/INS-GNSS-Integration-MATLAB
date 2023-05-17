function [out] = changeDegRange360(in)
%UNTITLED6 Summary of this function goes here
%   Detailed explanation goes here
if in < 0
    out = in + 360;
else
    out = in;
end
end