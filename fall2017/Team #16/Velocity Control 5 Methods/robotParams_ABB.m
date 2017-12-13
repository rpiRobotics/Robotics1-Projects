function [ex,ey,ez,n,P,q,H,type,dq_bounds,q_bounds] = robotParams()

I3 = eye(3);
ex = I3(:,1);ey = I3(:,2);ez = I3(:,3);
h1 = ez; h2 = ey; h3 = ey; h4 = ex; h5 = ey; h6 = ex;
P = [0,0,0;0.32, 0, 0.78;0, 0, 1.075;0, 0, 0.2;1.142, 0, 0;0.2, 0, 0;0,0,0]';
q = [0 0 0 0 0 0]';
H = [h1 h2 h3 h4 h5 h6];
type = [0 0 0 0 0 0];
n = 6;
dq_bounds = [100,110;90,90;90,90;170,190;120,140;190,235]';
dq_bounds = dq_bounds./180*pi;

q_bounds = [-170,170;-65,85;-180,70;-300,300;-120,120;-360,360];
q_bounds = q_bounds./180*pi;

end