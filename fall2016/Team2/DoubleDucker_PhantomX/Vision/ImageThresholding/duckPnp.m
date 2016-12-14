function [T_est] = duckPnp(p) 

% Pose Esitmation
% Uses Corke's RVC Tools:

% CentralCamera.estpose
% efficient_pnp.

% Nov. 9, 2016

% This is incomplete. However if you wish to test this you
% will need the RVCtools. Not sure how how much deep
% the rabit hole will go when search for all the required functions. 
% Did not want to upload all of RVCtools for a function that was
% incomplete and not part of the final design.

calc_cam = CentralCamera('focal', 0.004, 'pixel', 10e-6, ...
    'resolution', [640 480], 'centre', [320 240]);

P = mkcube(0.04); % assume Duke is cub with 5cm sides
P = P(:,5:8);
p(:,5)=[];

try
 T_est = calc_cam.estpose(P,p);
catch
  % Keep making Pose estimates despite errors. For dev purposes.
end
    