
function []= trossen_calibrate(robot)
load('Parameters.mat');
disp('lets calibrate by placing the tip of arm to a known point and and move the arm without moving the tip')
p0T(1,1)=input('x=');
p0T(2,1)=input('y=');
p0T(3,1)=input('z=');
t0=clock;
row=1;
while (etime(clock, t0)<30)
        thetax=double(robot.getJointPositions())- double([Acal; 0]);
        for i=1:1:4
            theta(i,1)=thetax(i,1);
        end
        angle(row,:)=theta'+ Acal';
        theta=theta*5.236/1023;
        H=[1 0 0 0; 0 1 0 0 ; 0 0 -1 0; 0 0 0 1];
        theta=H*theta;
        q1=theta(1);
        q2=theta(2);
        q3=theta(3);
        q4=theta(4);
        Ax=1;
        Ay=1;
        Az=1;
        B= rotz(q1)* roty(q2);
        Bx=B(1,1)+B(1,2)+B(1,3);
        By=B(2,1)+B(2,2)+B(2,3);
        Bz=B(3,1)+B(2,2)+B(3,3);
        C=rotz(q1)*roty(q2+q3);
        Cx=C(1,1)+C(1,2)+C(1,3);
        Cy=C(2,1)+C(2,2)+C(2,3);
        Cz=C(3,1)+C(3,2)+C(3,3);
        D=rotz(q1)*roty(q2+q3+q4);
        Dx=D(1,1)+D(1,2)+D(1,3);
        Dy=D(2,1)+D(2,2)+D(2,3);
        Dz=D(3,1)+D(3,2)+D(3,3);
        
        X(row,1:4)=[Ax Bx Cx Dx ];
        Y(row,1:4)=[Ay By Cy Dy ];
        Z(row,1:4)=[Az Bz Cz Dz ];
        Px(row,1)=p0T(1);
        Py(row,1)=p0T(2);
        Pz(row,1)=p0T(3);
        row=row+1;
end
disp('calculating...')
Xguess=[Pcal(1,1),Pcal(1,3:5)]';
Yguess=[Pcal(1,1),Pcal(1,3:5)]';
Zguess=[Pcal(1,1),Pcal(1,3:5)]';
Xopt=(X'*(X*X')^-1)*(Px-X*Xguess)+Xguess;
Yopt=(Y'*(Y*Y')^-1)*(Py-Y*Yguess)+Yguess;
Zopt=(Z'*(Z*Z')^-1)*(Pz-Z*Zguess)+Zguess;
P01=[Xopt(1); Yopt(1) ;Zopt(1)];
P12=[0;0;0];
P23=[Xopt(2); Yopt(2) ;Zopt(2)];
P34=[Xopt(3); Yopt(3) ;Zopt(3)];
P4T=[Xopt(4); Yopt(4) ;Zopt(4)];
Pcal=[P01, P12, P23, P34, P4T]
for i=1:1:(size(angle,1))
    [Rt, Pt]=trossen_forwardx(angle(i,:)',Pcal,0);
    error(1:4,i)=angle(i,:)'-trossen_ix(Rt,p0T,Pcal,0);
end
    Acal(1,1)=mean(error(1,:));
    Acal(2,1)=mean(error(2,:));
    Acal(3,1)=mean(error(3,:));


disp('lets calibrate by placing the tip of arm to a known point and and move the arm without moving the tip')
p0T(1,1)=input('x=');
p0T(2,1)=input('y=');
p0T(3,1)=input('z=');
t0=clock;

while (etime(clock, t0)<30)
        thetax=double(robot.getJointPositions())- double([Acal;0]);
        for i=1:1:4
            theta(i,1)=thetax(i,1);
        end
        theta=theta*5.236/1023;
        H=[1 0 0 0; 0 1 0 0 ; 0 0 -1 0; 0 0 0 1];
        theta=H*theta;
        q1=theta(1);
        q2=theta(2);
        q3=theta(3);
        q4=theta(4);
        Ax=1;
        Ay=1;
        Az=1;
        B=rotz(q1)*roty(q2);
        Bx=B(1,1)+B(1,2)+B(1,3);
        By=B(2,1)+B(2,2)+B(2,3);
        Bz=B(3,1)+B(2,2)+B(3,3);
        C=rotz(q1)*roty(q2+q3);
        Cx=C(1,1)+C(1,2)+C(1,3);
        Cy=C(2,1)+C(2,2)+C(2,3);
        Cz=C(3,1)+C(3,2)+C(3,3);
        D=rotz(q1)*roty(q2+q3+q4);
        Dx=D(1,1)+D(1,2)+D(1,3);
        Dy=D(2,1)+D(2,2)+D(2,3);
        Dz=D(3,1)+D(3,2)+D(3,3);
        
        X(row,1:4)=[Ax Bx Cx Dx ];
        Y(row,1:4)=[Ay By Cy Dy ];
        Z(row,1:4)=[Az Bz Cz Dz ];
        Px(row,1)=p0T(1);
        Py(row,1)=p0T(2);
        Pz(row,1)=p0T(3);
        row=row+1;
        pause(0.4);

end
Xguess=[Pcal(1,1),Pcal(1,3:5)]';
Yguess=[Pcal(1,1),Pcal(1,3:5)]';
Zguess=[Pcal(1,1),Pcal(1,3:5)]';
Xopt=(X'*(X*X')^-1)*(Px-X*Xguess)+Xguess;
Yopt=(Y'*(Y*Y')^-1)*(Py-Y*Yguess)+Yguess;
Zopt=(Z'*(Z*Z')^-1)*(Pz-Z*Zguess)+Zguess;
P01=[Xopt(1); Yopt(1) ;Zopt(1)];
P12=[0;0;0];
P23=[Xopt(2); Yopt(2) ;Zopt(2)];
P34=[Xopt(3); Yopt(3) ;Zopt(3)];
P4T=[Xopt(4); Yopt(4) ;Zopt(4)];
Pcal=[P01; P12; P23; P34; P4T];

disp('move the tip to another point')
pause(1);

Xnew=input('x=');
Ynew=input('y=');

thetax=double(robot.getJointPositions())- double([Acal; 0]);
for i=1:1:4
     theta(i,1)=thetax(i,1);
end
theta=theta*5.236/1023;
H=[1 0 0 0; 0 1 0 0 ; 0 0 -1 0; 0 0 0 1];
theta=H*theta;

q1=theta(1);
q2=theta(2);
q3=theta(3);
q4=theta(4);
        
 Pt=P01 + rotz(q1)*roty(q2)*P23 + rotz(q1)*roty(q2+q3)*P34 +rotz(q1)*roty(q2+q3+q4)*P4T;
Rsolve=[Pt(1), -Pt(2), Xnew; Pt(2), Pt(1), Ynew];
Rsolve=rref(Rsolve);
phi=atan2(Rsolve(2,3),Rsolve(1,3));
save('Parameters.mat','Acal','Pcal','phi');










      
      
      
        
