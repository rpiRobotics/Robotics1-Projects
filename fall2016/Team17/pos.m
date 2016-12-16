%% pos.m

% Author:   Devon Baughen
% Date:     11/21/16
% Description:
%   This code takes 15 measurements of the robots lane_pose
%   characteristic, and averages them in order to provide a more
%   accurate position measurement.

function [pos]=pos(duck)
    dv=[];
    pv=[];
    for i=1:15
        lane=duck.lane_pose;
        d=lane.d;
        dv=[dv d];
        p=lane.phi;
        pv=[pv p];
    end
    
    pos=[mean(dv) mean(pv)];

end