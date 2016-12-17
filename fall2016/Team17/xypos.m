%% xypos.m

% Author:   Devon Baughen
% Date:     11/21/16
% Description:
%   This code obtains the x and y position characteristics of the
%   robot. It was intended to be used for data collection, and 
%   furthr development of the robot's capabilties, but these 
%   measurements proved to be unreliable.

function [xy] = xypos(duck)
    xy = [duck.x; duck.y]; 
end