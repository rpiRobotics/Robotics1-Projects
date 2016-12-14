

clear all
% load('errorData.mat')
cam=webcam('Logitech HD Webcam C270');
robot = RobotRaconteur.Connect('tcp://localhost:10001/dobotRR/dobotController');

% robot = RobotRaconteur.Connect('tcp://localhost:10001/dobotRR/dobotController');

[fx] =  [ 1430.688616176092000 ]; % (focal length) * (the number of pixels per world unit)
[fy] =  [ 1425.219974303089900 ]; % (focal length) * (the number of pixels per world unit)
[cx] = [ 663.064313279749970  ]; % the optical center (the principal point)
[cy] = [ 362.320509108192820 ]; % the optical center (the principal point)
s = 0; % the skew parameter 
pick=0;%%%%%%%%%%%%%%%%%pick=1 when duickie will go to pick position
K = [fx 0 cx; 0 fy cy; 0 0 1];
R_CO = [0 1 0;  1 0 0;  0 0 -1];
% R_CO = [0 1 0;  -1 0 0;  0 0 1];
P_CO = [0, 0, 815]'; %%%

z_CA = 815; %mm %%%
inv_K_R = inv(K*R_CO);
pick_flag=0;
catchduicke=0;
Ts = 0.515; %sample  time
sigma_m=0.1;%%%%%%%% assume the measurement noise as a normal noise
A = [1,0,Ts,0;0,1,0,Ts;0,0,1,0;0,0,0,1]; % states = [x;y;vx;vy]; this is a model for constant velocity
H = [1,0,0,0;0,1,0,0]; % system outputs
% initialization of the Kalman Filter
X_hat = [-33;297;6;6]; % initial states
X_predict=[83;409]
X_minus=X_hat;
P = [0.25*Ts^4,0,0.5*Ts^3,0;0,0.25*Ts^4,0,0.5*Ts^3;0.5*Ts^3,0,Ts^2,0;0,0.5*Ts^3,0,Ts^2]; % initial Pk
Q = P; % assume zero modeling uncertainty 
I = eye(4,4);
R = sigma_m^2*eye(2); % 
while (~pick_flag)
% tic

img = snapshot(cam);

%%% Locate the Duckie
% [XCenter, XRadii, XColor] = LocateDuckie(img);
[XCenter, XRadii, XColor] = LocateDuckieDR(img,2);
imshow(img);
h = viscircles(XCenter,XRadii);

% K*R_CO*[XCenter(1), XCenter(2), 1]'-K*P_CO
% P_OA = inv_K_R*(z_CA*[XCenter(2), XCenter(1), 1]'-K*P_CO);
% int16(-50): close grip; int16(0): open grip;
P_OA = inv_K_R*(z_CA*[XCenter(1), XCenter(2), 1]'-K*P_CO);

 
%  CoX=[    1.8408     0.031907       265.48];
%  CoY=[     0.00087967       1.6859       312.29];

CoX = [   -1.8182   0.0069      4.9584];
CoY = [    0.0744   -1.7816 -356.7428];

Pw(1) = CoX(1)*P_OA(1)+CoX(2)*P_OA(2)+CoX(3);
Pw(2) = CoY(1)*P_OA(1)+CoY(2)*P_OA(2)+CoY(3);

[XCenter P_OA(1:2)' Pw(1:2)]
 

%%%%%%%%%%%kalman filter
%if abs(Pw(1)-X_minus(1,:)) < 100
t_measured(1,:)=Pw(1); %%%%%%%% data insert(x,y)
t_measured(2,:)=Pw(2);
%end

  % prediction
   X_minus = A*X_hat;
   P_minus = A*P*A' + Q;
    % update observation
   Kal = P_minus*H'*(H*P_minus*H'+R)^(-1);
   %X_hat = X_minus + Kal*(t_measured - H*X_minus);
   X_hat = X_minus + Kal*(t_measured - H*X_minus);
   P = (I - Kal*H) * P_minus;
   Y_hat = H*X_hat;
   pick=pick+1;
    %X_predict(1,:)=(X_minus(1,:)-Pw(1))*5+Pw(1);
    %X_predict(2,:)=(X_minus(2,:)-Pw(2))*5+Pw(2);
%disp(['measured location',num2str(Pw(1))]);
 disp(['predict location',num2str(X_minus(1,:))]);
 %disp(['measured location',num2str(Pw(2))]);
 disp(['predict location',num2str(X_minus(2,:))]);
% toc
if pick>15
    pick_flag=1;
    
end

end
X_predict(1,:)=(X_minus(1,:)-Pw(1))*7.5+Pw(1);
    X_predict(2,:)=(X_minus(2,:)-Pw(2))*7.5+Pw(2);
X_predict
pick_flag

[Q,e1] = ikdobot(X_predict(1),X_predict(2),60); %start position
robot.setJointPositions(int16(Q(1)),int16(Q(2)),int16(Q(3)),int16(0),int16(25))
pause(0.1)
[Q,e1] = ikdobot(X_predict(1),X_predict(2),40)
pause(0.1)
robot.setJointPositions(int16(Q(1)),int16(Q(2)),int16(Q(3)),int16(0),int16(25))
pause(0.1)
robot.setJointPositions(int16(Q(1)),int16(Q(2)-1),int16(Q(3)),int16(0),int16(55))
