function [XYZ] = pinhole( pixelCoor )
%This function calculates the real world xyz position, relative
%to the robot arm using the standard pinhole model
syms xa0 ya0 za0
%Convert pixel corrdinates to real world axis coordinates and convert
%picture coordinate space to standard cartesian space.
xib = (pixelCoor(1) - 640/2)*2.5;
yib = (480/2 - pixelCoor(2))*2.5;
%calculate rotation between camera and robot
roc = rot([0;0;1],pi);
%Extrinisc values from xyz and 
poc = [221.5; -15.0; 210];
pac= [xa0;ya0;za0];
f = 4;
xd = 0;
yd = 0;
zca = 270;
intrin = [f*640 0 xd; 0 f*480 yd; 0 0 1];
%Picture matrix, contains the x,y points of the webcam image
picture = zca*[xib; yib; 1];
%extrinsic matrix
extrin = [roc poc; 0 0 0 1];
extreal = extrin*[pac; 1];
extreal = extreal(1:3);
sol = inv(intrin)*picture;

eq1 = extreal(1) == sol(1);
eq2 = extreal(2) == sol(2);
eq3 = extreal(3) == sol(3);
%Solve the equations to translate the xyz points to cartesian points
[A B] = equationsToMatrix([eq1,eq2,eq3],[xa0,ya0,za0]);
XYZ = transpose(vpa(linsolve(A,B),3)); 
end

