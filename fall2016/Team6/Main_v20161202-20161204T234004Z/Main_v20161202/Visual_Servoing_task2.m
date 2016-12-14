clear all
%%% Task 2 : Picking with Different Depth

%%% Initialization
cam=webcam('Logitech HD Webcam C270');
% robot = RobotRaconteur.Connect('tcp://localhost:10001/dobotRR/dobotController');

robot = RobotRaconteur.Connect('tcp://localhost:10001/dobotRR/dobotController');

[Q,e] = ikdobot(230,0,100); %start position
robot.setJointPositions(int16(Q(1,1,1)),int16(Q(1,2,1)),int16(Q(1,3,1)),int16(0),int16(50))
pause(0.1)
robot.setJointPositions(int16(Q(1,1,1)),int16(Q(1,2,1)),int16(Q(1,3,1)),int16(0),int16(50))
uiwait(msgbox({'Ready to go?'}));

while 1

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


moving_flag = 0;
pick_flag = 0;
Position_Seq_size = 1000;
    CoX = [   -1.8182   0.0069      4.9584];
    CoY = [    0.0744   -1.7816 -356.7428];
    
%%% First Frame
clear Position_Seq
    img = snapshot(cam);
    %%% Locate the Duckie
    [XCenter, XRadii, Xsub,OriDuck] = LocateDuckie_v2(img,2);
    Z = 820-820*39.7/XRadii+50
    XRadii_seq(1) = XRadii;
    %%% Transform the Location it in World Frame
    P_OA = inv_K_R*(z_CA*[XCenter(1), XCenter(2), 1]'-K*P_CO);
    Pw(1) = CoX(1)*P_OA(1)+CoX(2)*P_OA(2)+CoX(3);
    Pw(2) = CoY(1)*P_OA(1)+CoY(2)*P_OA(2)+CoY(3);
    Position_Seq(1,:) = [Pw, Z];
cnt = 2;

[Q,e] = ikdobot(Pw(1),Pw(2),Z);
if strcmp(e,'None')
    robot.setJointPositions(int16(Q(1)),int16(Q(2)),int16(Q(3)),int16(0),int16(50))
else
    disp('INVALID')
end
uiwait(msgbox({'Ready to go?'}));

end


% while (~pick_flag)
% % tic
%     img = snapshot(cam);
% % toc
%     %%% Locate the Duckie
%     % tic
%     [XCenter, XRadii, Xsub,OriDuck] = LocateDuckie_v2(img,2);
% %     XRadii
% %     XRadii_seq(cnt) =XRadii;
% %     Z = 0.1435+0.0530*(820-820*34.491/XRadii);
%     Z = 43.6035-1498.97886/XRadii;
%     % toc
%    figure(1);
%    imshow(img);
%    h = viscircles(XCenter,XRadii);
%     
%     %%% Transform the Location it in World Frame
%     P_OA = inv_K_R*(z_CA*[XCenter(1), XCenter(2), 1]'-K*P_CO);
%     Pw(1) = CoX(1)*P_OA(1)+CoX(2)*P_OA(2)+CoX(3);
%     Pw(2) = CoY(1)*P_OA(1)+CoY(2)*P_OA(2)+CoY(3);
% %     Pw
%     Position_Seq(cnt,:) = [Pw, Z];
%     [Pw, Z]
%     if (cnt ==1)
%         prev_cnt = Position_Seq_size;
%     else
%         prev_cnt =cnt-1;        
%     end
%     
%     Dist = norm(Pw-Position_Seq(prev_cnt,1:2));
%     if (moving_flag == 1)
%         if (Dist<5)
% %             pick_flag = 1
%         end
%     end
%     
%     if ( Dist<5)
%         moving_flag = 0;
%     else
%         moving_flag =1;
% %             figure(1);
% %     imshow(img);
% %     h = viscircles(XCenter,XRadii);
%     end
%     
%     cnt = cnt+1;
%     if(cnt == Position_Seq_size + 1)
%         cnt =1;
%     end
%         
% end

%%% Commed the Arm
% 
% [ Q , error, works] = ikdobot( Pw(1),Pw(2), 65);
% Theta_Duck = acos(OriDuck(1,1)/sqrt(OriDuck(2,1)^2+OriDuck(1,1)^2));
% q4 = real(180*(Theta_Duck-pi/2)/pi-Q(1));
% Q = int16(real(Q));
% if (works ==1)
%     robot.setJointPositions(Q(1), Q(2), Q(3), int16(q4) , int16(0) );
% end
% pause(1);
% robot.getJointPositions()