clear all
%%% Task 2 : Picking with Different Depth

%%% Initialization
cam=webcam('Logitech HD Webcam C270');
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
    Z = 820-820*39.7/XRadii+20 %take off the plus 50 to be at same height
    XRadii_seq(1) = XRadii;
    %%% Transform the Location it in World Frame
    P_OA = inv_K_R*(z_CA*[XCenter(1), XCenter(2), 1]'-K*P_CO);
    Pw(1) = CoX(1)*P_OA(1)+CoX(2)*P_OA(2)+CoX(3);
    Pw(2) = CoY(1)*P_OA(1)+CoY(2)*P_OA(2)+CoY(3);
    Position_Seq(1,:) = [Pw, Z];
cnt = 2;

[Q,e] = ikdobot(Pw(1),Pw(2)+30,Z); %will be 30 mm to right of center,same y,z
disp([Pw(1);Pw(2);Z])
if strcmp(e,'None')
    robot.setJointPositions(int16(Q(1)),int16(Q(2)),int16(Q(3)),int16(0),int16(50))
else
    disp('INVALID')
end
uiwait(msgbox({'Ready to go?'}));

end

