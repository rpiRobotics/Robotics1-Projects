function [Q] = ikdobot2(xd,yd,zd )
%Inverse Kinematic of dobot using a mix of trigonametric approach and sub
%problem approach.
l1 = 103;
l2 = 135;
l3 = 160;
Lg = 56;  %length from point T to q4 rotation
d = 115;  %length from q4 to pen end

ex = [1;0;0]; ey = [0;1;0]; ez = [0;0;1];
p01 = [0;0;l1];
p23 = [0;0;l2];
p3t = [l3;0;0];
p0t = [xd;yd;zd];
Q=[1i,1i,1i];
%Find q1
p0g_desired = [xd;yd;zd]; 
q1 = atan(p0g_desired(2)/p0g_desired(1));
if (p0g_desired(2)<0) && (p0g_desired(1)<0)
    q1 = q1 + pi;
end
if (p0g_desired(2)>0)&&(p0g_desired(1)<0)
    q1 = q1 - pi;
end

p4t = [Lg*cos(q1);Lg*sin(q1);-d];
p0t = p0t -p4t;
%Find Q2
sec1 = norm(rot(ez,-q1)*(p0t - p01));
diffq3q2 = subprob3(ey,p3t,-p23,sec1);
p1 = p23 + rot(ey,diffq3q2(1))*p3t;
p2 = rot(ez,-q1)*(p0t-p01);
q21 = subprob1(ey,p1,p2);

p1 = p23 + rot(ey,diffq3q2(2))*p3t;
p2 = rot(ez,-q1)*(p0t-p01);
q22 = subprob1(ey,p1,p2);
%Find Q3
q31 = diffq3q2(1) - q21;
q32 = diffq3q2(2) - q22;

%Make sure q's are not out side of joint constraints
Q =  [q1 q21 q31; q1 q22 q32]*(180/pi);
newQ = [];
for k=1:size(Q,1)
 works = 1;
    if (Q(k,1)<-135) || (Q(k,1)>135) 
        error = 'q1 out of range'
        works = 0;
        break
    end
        
    if (Q(k,3)<-10) || (Q(k,3)>95)
        error = 'q3 out of range'
        works = 0;
        break
    end
            
    if (Q(k,2)<-5) || (Q(k,2)>85)
        error = 'q2 out of range';
        works = 0;
    end
    if works == 1
       newQ = [newQ;Q(k,:)]; 
    end
end
Q = newQ;

end

