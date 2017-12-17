% lane following using PI control
% the harded coded offset for wheel calibration
calib_offset_w = 0.165;
k_p_d = -5;
k_d_int = -5;
k_p_theta = -3.5;
k_theta_int = -1.0;
iteration = 2;

i = 0;
dlist = zeros(1, 3000);
philist = zeros(1, 3000);

duck.sendCmd(0,0);
run_d = 0;
run_phi = 0;

while 1
    tic;
    pose = duck.lane_pose;
    i = i+1;
    if i == 3001
        break
    end
    d = pose.d;
    phi = pose.phi;
    dlist(i) = d;
    philist(i) = phi;
    
    if length(run_d) < iteration
        run_d = [run_d d];
        run_phi = [run_phi phi];
    else
        run_d = [run_d(2:iteration) d];
        run_phi = [run_phi(2:iteration) phi];
    end
    toc;
    
    int_d = sum( run_d(1:length(run_d)-1) ) / (length(run_d)-1);
    int_phi = sum(run_phi(1:length(run_phi)-1))/(length(run_phi)-1);
    
    w = k_d_int*int_d + ...
        k_p_d*d + ...
        k_theta_int*int_phi + ...
        k_p_theta*phi;
    
    duck.sendCmd(0.215 , w + calib_offset_w);
%     pose = duck.lane_pose;
%     if (abs(phi(i)) < 0.12) && (abs(d(i)-0.2) < 0.08)
%         omega = -2.25*pose.phi+0.8*pose.d+0.5;
%         duck.sendCmd(0.4, 1.1*omega);
%     else
%         if phi < 0
%             omega = -2.7*phi(i) + 1.2*(d(i)-0.05)+0.4;
%             duck.sendCmd(0.2, 1 * omega);
%         else
%             omega = -2.7*phi(i) + 1.2*(d(i)-0.05)+0.2;
%             duck.sendCmd(0.2, 1.1 * omega);
%         end
%     end
%     i = i + 1;
%     % toc;
end
% duck.sendCmd(0, 0);
% figure(1);
% i = 1:1000;
% plot(i, phi);
% figure(2);
% plot(i, d);
