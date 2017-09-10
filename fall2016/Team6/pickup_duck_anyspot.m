clc; clear; close;

% clear all

%%% Initialization
% cam=webcam('Logitech HD Webcam C270');
% robot = RobotRaconteur.Connect('tcp://localhost:10001/dobotRR/dobotController');


Q = zeros(1,3,16);
[Q(:,:,1),~] = ikdobot(180,0,10); %start position


robot = RobotRaconteur.Connect('tcp://localhost:10001/dobotRR/dobotController');
robot.setJointPositions(int16(Q(1,1,1)),int16(Q(1,2,1)),int16(Q(1,3,1)),int16(0),int16(50))
pause(0.1)
robot.setJointPositions(int16(Q(1,1,1)),int16(Q(1,2,1)),int16(Q(1,3,1)),int16(0),int16(50))
uiwait(msgbox({'Ready to go?'}));

cam=webcam('Logitech HD Webcam C270');
[fx] =  [ 1430.688616176092000 ]; % (focal length) * (the number of pixels per world unit)
[fy] =  [ 1425.219974303089900 ]; % (focal length) * (the number of pixels per world unit)
[cx] = [ 663.064313279749970  ]; % the optical center (the principal point)
[cy] = [ 362.320509108192820 ]; % the optical center (the principal point)
s = 0; % the skew parameter 
K = [fx 0 cx; 0 fy cy; 0 0 1];
R_CO = [0 1 0;  1 0 0;  0 0 -1];
P_CO = [0, 0, 815]'; 
z_CA = 815; %mm %%%
inv_K_R = inv(K*R_CO);
% tic

img = snapshot(cam);
% toc

%%% Locate the Duckie
% tic
[XCenter, XRadii, Xsub,OriDuck] = LocateDuckie_v2(img,2);
% XRadii;
Z = 820-820*39.7/XRadii;
% toc
% figure(1);
% imshow(img);
% h = viscircles(XCenter,XRadii);

%%% Transform the Location it in World Frame
P_OA = inv_K_R*(z_CA*[XCenter(1), XCenter(2), 1]'-K*P_CO);
CoX = [   -1.8182   0.0069      4.9584];
CoY = [    0.0744   -1.7816 -356.7428];
Pw(1) = CoX(1)*P_OA(1)+CoX(2)*P_OA(2)+CoX(3);
Pw(2) = CoY(1)*P_OA(1)+CoY(2)*P_OA(2)+CoY(3);
Pw

 
%%% Command the Arm
[ q , error ] = ikdobot( Pw(1),Pw(2), 65);
Theta_Duck = acos(OriDuck(1,1)/sqrt(OriDuck(2,1)^2+OriDuck(1,1)^2));
q4 = real(180*(Theta_Duck-pi/2)/pi-q(1));
disp('this is q4')
disp(q4)

[Q(:,:,2),~] = ikdobot(180,0,75); %move above duck
[Q(:,:,3),~] = ikdobot(Pw(1),Pw(2),75); %move to duck position
[Q(:,:,4),~] = ikdobot(Pw(1),Pw(2),40); %lower onto duck
Q(:,:,5) = [Q(:,1,4),Q(:,2,4)-1,Q(:,3,4)]; %close gripper
[Q(:,:,6),~] = ikdobot(Pw(1),Pw(2),75); %pick up duck at its location
[Q(:,:,7),~] = ikdobot(155,-165,75); %move toward goal
[Q(:,:,8),~] = ikdobot(155,-165,110); %move above obstacle
[Q(:,:,9),~] = ikdobot(90,-225,115); %move toward goal
[Q(:,:,10),~] = ikdobot(90,-225,55); %lower into goal
Q(:,:,11) = [Q(:,1,10),Q(:,2,10)-1,Q(:,3,10)]; %open gripper
Q(:,:,12) = Q(:,:,9); %move out of goal
Q(:,:,13) = Q(:,:,8); %move away from goal
[Q(:,:,14),~] = ikdobot(145,-170,50); %lower outside of goal
[Q(:,:,15),~] = ikdobot(250,0,50); %move toward start
Q(:,:,16) = Q(:,:,1); %return to start

rot = [0 0 q4 q4 q4 q4 0 0 0 0 0 0 0 0 0 0]; 
grip = [25 25 25 25 53 53 53 53 53 53 25 25 25 25 25 25];

for ii = 2:16
    robot.setJointPositions(int16(Q(1,1,ii)),int16(Q(1,2,ii)),int16(Q(1,3,ii)),int16(rot(ii)),int16(grip(ii)))
    pause(2)
end
