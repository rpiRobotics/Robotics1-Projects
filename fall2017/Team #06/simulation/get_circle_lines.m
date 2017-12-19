function [ x,y ] = get_circle_lines(xo, yo, r)
% Author: Zac Ravichandran
% Generate lines for a circle with the given input parameters

theta = 0 : 0.01 : 2*pi;
x = r * cos(theta) + xo;
y = r * sin(theta) + yo;

end

