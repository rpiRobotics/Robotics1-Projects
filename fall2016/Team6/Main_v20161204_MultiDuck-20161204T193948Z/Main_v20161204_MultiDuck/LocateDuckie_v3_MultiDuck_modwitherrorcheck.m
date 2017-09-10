% clear all
%%% Task 2 : Picking with Different Depth

%%% Initialization
%cam=webcam('Logitech HD Webcam C270');
robot = RobotRaconteur.Connect('tcp://localhost:10001/dobotRR/dobotController');
Q = zeros(1,3,16);
e = {};
[Q(:,:,1),e{1}] = ikdobot(180,0,10); %start position
robot.setJointPositions(int16(Q(1,1,1)),int16(Q(1,2,1)),int16(Q(1,3,1)),int16(0),int16(50))
pause(0.1)
robot.setJointPositions(int16(Q(1,1,1)),int16(Q(1,2,1)),int16(Q(1,3,1)),int16(0),int16(50))
uiwait(msgbox({'Ready to go?'}));


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
    CoXY = [   -1.8182   0.0069      4.9584; 0.0744   -1.7816 -356.7428];
    
%%% First Frame
    clear Position_Seq
    img = snapshot(cam);
    %%% Locate the Duckie
    N_Duck = 4;
    [XCenter, XRadii, OriDuck] = LocateDuckie_v3_MultiDuck(img,2,N_Duck);
    Z = 43.6035-1498.97886./XRadii;
    %%% Transform the Location it in World Frame
    P_OA = inv_K_R*(z_CA*[XCenter, ones(N_Duck,1)]'- K*P_CO*ones(N_Duck,1)');
    P_OA = P_OA(1:end-1,:);
    Pw = CoXY*[P_OA' ones(N_Duck,1)]';
    Pw = Pw';
    Position_Seq(1,:,:) = [Pw, Z'];
    cnt = 2;
    prev_cnt = 1;
    
while (~pick_flag)
% tic
    img = snapshot(cam);
% toc
    %%% Locate the Duckie
    % tic
    
   [XCenter, XRadii, OriDuck] = LocateDuckie_v3_MultiDuck(img,2,N_Duck);
    Z = 43.6035-1498.97886./XRadii;
 
     imshow(img);
     h = viscircles(XCenter,XRadii);
    
    %%% Transform the Location it in World Frame
    P_OA = inv_K_R*(z_CA*[XCenter, ones(N_Duck,1)]'- K*P_CO*ones(N_Duck,1)');
    P_OA = P_OA(1:end-1,:);
    Pw = CoXY*[P_OA' ones(N_Duck,1)]';
    Pw = Pw';
    CostMatrx = zeros(N_Duck,N_Duck);
    for loop1 =1:N_Duck
        for loop2 =1:N_Duck
            CostMatrx(loop1,loop2) = norm(Pw(loop2,:)'-squeeze(Position_Seq(prev_cnt,loop1,1:2)));
        end
    end
    
    Position_Seq(cnt,:,:) = [Pw(GlobalTracking(CostMatrx),:), Z'];

%    squeeze(Position_Seq(cnt,:,:))
   
for loop1 =1:N_Duck
    Dist = norm(squeeze(Position_Seq(cnt,loop1,1:2)-Position_Seq(prev_cnt,loop1,1:2)));
        if (Dist>250)
             pick_flag = 1;
             pick_obj = loop1;
        end
    
end
    
    if (cnt ==1)
        prev_cnt = Position_Seq_size;
    else
        prev_cnt =cnt-1;        
    end

    cnt = cnt+1;
    if(cnt == Position_Seq_size + 1)
        cnt =1;
    end
        
end



Pw = squeeze(Position_Seq(prev_cnt,pick_obj,:))

%%% Commed the Arm
[ q , error ] = ikdobot( Pw(1),Pw(2), 65);
Theta_Duck = acos(OriDuck(1,1)/sqrt(OriDuck(2,1)^2+OriDuck(1,1)^2));
q4 = real(180*(Theta_Duck-pi/2)/pi-q(1));
disp('this is q4')
disp(q4)

[Q(:,:,2),e{2}] = ikdobot(220,0,75); %move above duck
[Q(:,:,3),e{3}] = ikdobot(Pw(1),Pw(2),75); %move to duck position
[Q(:,:,4),e{4}] = ikdobot(Pw(1),Pw(2),40); %lower onto duck
Q(:,:,5) = [Q(:,1,4),Q(:,2,4)-1,Q(:,3,4)]; %close gripper
e{5} = e{4};
[Q(:,:,6),e{6}] = ikdobot(Pw(1),Pw(2),75); %pick up duck at its location
[Q(:,:,7),e{7}] = ikdobot(155,-165,75); %move toward goal
[Q(:,:,8),e{8}] = ikdobot(155,-165,110); %move above obstacle
[Q(:,:,9),e{9}] = ikdobot(90,-225,115); %move toward goal
[Q(:,:,10),e{10}] = ikdobot(90,-225,55); %lower into goal
Q(:,:,11) = [Q(:,1,10),Q(:,2,10)-1,Q(:,3,10)]; %open gripper
e{11} = e{10};
Q(:,:,12) = Q(:,:,9); %move out of goal
e{12} = e{9};
Q(:,:,13) = Q(:,:,8); %move away from goal
e{13} = e{8};
[Q(:,:,14),e{14}] = ikdobot(145,-170,50); %lower outside of goal
[Q(:,:,15),e{15}] = ikdobot(250,0,50); %move toward start
Q(:,:,16) = Q(:,:,1); %return to start
e{16} = e{1};

rot = [0 0 q4 q4 q4 q4 0 0 0 0 0 0 0 0 0 0]; 
grip = [25 25 25 25 54 54 54 54 54 54 25 25 25 25 25 25];

for ii = 2:16
    if strcmp(e{ii},'None')
        robot.setJointPositions(int16(Q(1,1,ii)),int16(Q(1,2,ii)),int16(Q(1,3,ii)),int16(rot(ii)),int16(grip(ii)))
        pause(2)
        disp(Q(:,:,ii))
        robot.getJointPositions()
    else
        disp('No Solution')
        break
    end
end
