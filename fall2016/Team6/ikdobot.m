function [ Q , error ] = ikdobot( xd,yd,zd )
%ikdobot computes inverse kinematics for dobot 
%   takes desired x,y,z and returns q1,q2,q3
%% constants needed
l1 = 103;
l2 = 135;
l3 = 160;
Lg = 56;  %length from point T to q4 rotation
d = 115;  %length from q4 to gripper end

%% initialization of variables
Q=[1i,1i,1i];
syms theta3
works = 0;

%% inverse kinematics
p0g_desired = [xd;yd;zd]; 
q1 = atan(p0g_desired(2)/p0g_desired(1));
if (p0g_desired(2)<0) && (p0g_desired(1)<0)
    q1 = q1 + pi;
end
if (p0g_desired(2)>0)&&(p0g_desired(1)<0)
    q1 = q1 - pi;
end
xgd = Lg*cos(q1); ygd = Lg*sin(q1);
p0t_desired = p0g_desired-[xgd;ygd;-d];
x=p0t_desired(1); y=p0t_desired(2); z=p0t_desired(3);

f=@(theta3)(x/cos(q1) - l3*cos(theta3)-l2*sin( acos( (z-l1+l3*sin(theta3))/l2 ) ));
[q3,~,exitflag,~]=fsolve(f,0,optimoptions('fsolve','Display','off'));

q2 = acos( (z-l1+l3*sin(q3))/l2 ); 
q1 = double(q1);
q2 = real(q2);
q3 = real(q3);

works = 0;
while works == 0
    if exitflag < 1
        error = 'No solution for q3';
        break
    end
    if (q1<-135*pi/180) || (q1>135*pi/180) 
        error = 'q1 out of range';
        break
    end
        
    if (q3<-10*pi/180) || (q3>95*pi/180)
        error = 'q3 out of range';
        break
    end
            
    if (q2<-5*pi/180) || (q2>85*pi/180)
        error = 'q2 out of range';
        break
    end
    

    
    Q = [q1*180/pi q2*180/pi q3*180/pi];
    error = 'None';
    works = 1;
end

end

