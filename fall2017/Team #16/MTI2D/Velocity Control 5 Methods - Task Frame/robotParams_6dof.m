function [ex,ey,ez,n,P,q,H,type,dq_bounds,q_bounds] = robotParams()
l1 = 0.6*2; l2=0.6*0.7; l3=0.6*1.0;
ex = [1,0,0]';    ey =  [0,1,0]';    ez =  [0,0,1]';
h1 = ez; h2 = ey; h3 = ex; h4 = ey; h5 = ey; h6 = ex;
p01 = [0,0,0]'; p12 = [0,0,0]'; p23 = [0,0,0]'; p34 = [l1,0,0]'; p45 = [l2,0,0]'; p56 = [0,0,0]';
p6T = [l3,0,0]';

H = [h1,h2,h3,h4,h5,h6];
P = [p01,p12,p23,p34,p45,p56,p6T];
q = [0 0 0 0 0 0]';
type = [0 0 0 0 0 0];
n = size(H,2);
dq_bounds = [100,110;90,90;90,90;170,190;120,140;190,235]';
dq_bounds = dq_bounds./180*pi;
q_bounds = [-170,170;-65,85;-180,70;-300,300;-120,120;-360,360];
q_bounds = q_bounds./180*pi;

end