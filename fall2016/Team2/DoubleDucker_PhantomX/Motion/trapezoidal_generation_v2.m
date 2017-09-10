function [ profile ] = trapezoidal_generation_v2( l_0,l_f,v_0,v_f,vmax,amax,dmax,num_points )
%trapezoidal_generation create a trapezoidal profile to go from l_0 to l_f 
%   while obeying the constraints provided as arguments.  Implemented as 1D
%   solution
%   l_0 = starting position, l_f = ending position
%   v_0 = starting velocity, v_f = ending velocity
%   vmax = maximum velocity, amax = maximum acceleration
%   dmax = maximum decceleration, num_points = desired number of points in
%   profile

% RETURNS set of points going from start to finish
lfin=l_f;
l0in=l_0;
if l0in>lfin
    l_f=l0in;
    l_0=lfin;
end
t1 = (vmax - v_0)/amax; % time at which vmax is reached, accel stops
t2 = t1+1/vmax*(l_f-l_0-(vmax^2-v_0^2)/(2*amax)-(vmax^2-v_f^2)/(2*dmax)); % time when decel begins
% t2=t1/((vmax-v_f)/dmax-1); %For symmetrical
tf = t2+(vmax-v_f)/dmax; % time when l_f is reached

dt = double(tf)/double(num_points);

% generate profile
profile = [];
for t=0:dt:tf
    if 0<=t&&t<t1 % in accel period
        profile = [profile .5*t^2*amax+t*v_0 + l_0];
    elseif t1<=t && t<t2 % in vmax period
        profile = [profile .5*t1^2*amax+t*v_0+(t-t1)*t1*amax + l_0];
    else % in deccel period
        profile = [profile .5*t1^2*amax+(t2-t1)*t1*amax+t*v_0+l_0-.5*(t-t2)^2*dmax+(t-t2)*t1*amax]
    end;
end
if l0in>lfin
    profile=fliplr(profile);
end
end

