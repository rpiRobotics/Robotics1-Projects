% lane following using PI control
% the harded coded offset for wheel calibration
calib_offset_w = 0.165;
k_p_d = -5;           % the proportional gain for distance error
k_d_int = -5;         % the integral gain for distance error
k_p_theta = -3.5;     % the proportional gain for angular error
k_theta_int = -1.0;   % the integral gain for angular error
iteration = 2;        % the variable controls how many old errors should we take in

duck.sendCmd(0,0);
run_d = 0;
run_phi = 0;

while 1
    % tic;
    pose = duck.lane_pose;
    d = pose.d;
    phi = pose.phi;
    
    if length(run_d) < iteration
        run_d = [run_d d];
        run_phi = [run_phi phi];
    else
        run_d = [run_d(2:iteration) d];
        run_phi = [run_phi(2:iteration) phi];
    end
    % toc;
    
    % get the feedback
    int_d = sum( run_d(1:length(run_d)-1) ) / (length(run_d)-1);
    int_phi = sum(run_phi(1:length(run_phi)-1))/(length(run_phi)-1);
    
    % apply PI controller
    w = k_d_int*int_d + ...
        k_p_d*d + ...
        k_theta_int*int_phi + ...
        k_p_theta*phi;
    
    duck.sendCmd(0.215 , w + calib_offset_w);
end
