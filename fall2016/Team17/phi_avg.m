function [avg]=phi_avg(duck)
    duck=duck;
    pv=[];
    for i=1:20
        lane=duck.lane_pose;
        p=lane.phi;
        pv=[pv p];
    end
    
    %return avg=mean(dv);
    avg=mean(pv);
    
    %avg
end