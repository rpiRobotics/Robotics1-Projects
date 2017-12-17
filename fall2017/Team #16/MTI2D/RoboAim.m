close all;
clear;
%% connect to robotraconteur
load('connInfo.mat');
robot = RobotRaconteur.Connect(RobotAddr);
mode = uint8(1);
robot.setControlMode(mode);
pause(0.01);
robot.setJointCommand('right',[0,0,0,0,0,0,0]');
%% connect to laser
mti2d = RobotRaconteur.ConnectService(Mti2dAddr);
Temperature = mti2d.getPropertyValue('Temperature')

setExposureTime(mti2d,25);
pause(1);
setFrequency(mti2d,200);
pause(1);
setSignalSelection(mti2d,0);
setIsDoubleSampling(mti2d,0);
%% Control Methods
%Velocity Control:(1); 
%Constrained Velocity Control:(2);
%Constrained Velocity Control with Joint Limits:(3);
%Constrained Velocity with SVD Stablization: (4);
%Constrained Velocity with SVD Stablization with Joint Limits: (5);
ctrl_mode = 3; 
%% Joint Limits
j_lim = 0; %Consider Joint Limits:(1); Don't Consider Joint Limits:(0)
%% Initialize Robot
[ex,ey,ez,n,P,~,H,type,dq_bounds,q_bounds] = robotParams();

%% Initialize Control Parameters
%initial joint angles
% q = [0,0,0,0,0,0,0]';
% q = [pi/2,-0.3,0,0.58,0,-0.31,0.15]';
q =  robot.joint_positions;

R_EE = [1.0000    0.0000    0.0000
        0.0000    0.9848    0.1736
        0.0000   -0.1736    0.9848];

% [R,pos] = fwdkin(q,type,H,P,n);
[pp,RR]=fwdKin_alljoints_sawyer(q,type,H,P,n,R_EE);
R = RR(:,:,end);
pos = pp(:,end);
orien = R2q(R)';
pos_v = [0,0,0]';
ang_v = [1,0,0,0];
dq = zeros(n,1);

%plot options
view_port = [-140 30];%currently can't change viewing angle during the process
axes_lim = [-0.6 1.2 -0.6 1.2 -0.6 1.2];%modify this if a robot is too small or too large
key_info = {'KEYS: exit-''ESC'', stop-''p'', up-''uparrow'', down-''downarrow''','left-''leftarrow'', right-''rightarrow'',forward-''backslash'', backward-''enter'',', 'roll-''a'', pitch-''s'', yaw-''d'', rev roll-''q'', rev pitch-''w'', rev yaw-''e''',''};

%velocities
w_t = [0,0,0]';
v_t = [0,0,0]';

%keyboard controls
%define position and angle step
inc_pos_v = 0.008; % m/s
inc_ang_v = 1.5/180*pi; % rad/s

%optimization params
er = 0.05;
ep = 0.005;
epsilon = 0;%legacy param for newton iters

%scanning utilities
scan_dist = [0,0,0]';


%create a handle of these parameters for interactive modifications
params = ControlParams(ex,ey,ez,n,P,H,type,dq_bounds,q_bounds,q,dq,pos,orien,pos_v,ang_v,w_t,v_t,...
    epsilon,view_port,axes_lim,inc_pos_v,inc_ang_v,0,0,0,er,ep,0,0,robot);

%% Display Robot Initial Position
%plot out with plotting settings
figure('keypressfcn',{@func_keypress,params});
% hold on
% [pp,RR]=fwdKin_alljoints(q,type,H,P,n);
[pp,RR]=fwdKin_alljoints_sawyer(q,type,H,P,n,R_EE);
% view(view_port);
% axis(axes_lim);
% grid on
dt = 0;
counter = 0;
%display loop
tic
while ~params.controls.stop    
    if (counter~=0)        
        dt = toc; %take a time interval measurement
    end    
    tic %start clocking
    counter = counter + 1;
    if (counter~=0)
        %update joint angles
        q_temp = params.controls.q + params.controls.dq*dt;%compute new q use time interval and velocities.
        params.controls.dt = dt;
        if(j_lim)
            if sum(q_temp>q_bounds(:,1)) == n && sum(q_temp<q_bounds(:,2)) == n  
                params.controls.q = q_temp;
            else
                params.controls.dq = zeros(n,1);
            end
        else
            params.controls.q = q_temp;
        end
        %update robot
%         hold off;
        [pp,RR]=fwdKin_alljoints_sawyer(params.controls.q,type,H,P,n,R_EE);
%         [pp,RR] = robot_3d(params.controls.q);
%         view(params.plots.view_port);
%         axis(params.plots.axes_lim);
%         grid on
%         title(key_info);
%         drawnow
        A = mti2d.lineProfile;
        len =A.length;
        X = A.X_data;
        Z = A.Z_data;
        I = double(A.I_data);
        I_scaled = 125-(I/1024*60); 
%         t_elp = toc;
%         tic
        plot(X,Z,'.k','MarkerSize',0.1)
        hold on
        plot(X,I_scaled,'.b','MarkerSize',0.1)
        hold off
        axis([-35,35,55,130]);
        axis ij
        grid on
        drawnow
        
        
        %update current position and orientation
        params.controls.pos = pp(:,end);
        params.controls.orien = R2q(RR(:,:,end))';
        
        
        %compute Jacobian
%         J = getJacobian(params.controls.q, params.def.type, params.def.H, params.def.P, params.def.n);
        if params.keyboard.rec_flag == 0
            scan_dist = [0,0,-mean(Z(630:650))/1000]';
            J = getJacobian_task_sawyer(params.controls.q, params.def.type, params.def.H, params.def.P, params.def.n,R_EE);
        else
            J = getJacobian_task_sawyer(params.controls.q, params.def.type, params.def.H, params.def.P+[zeros(3,params.def.n),scan_dist], params.def.n,R_EE);
        end
        
        
        
        %compute new joint velocities
        axang = quat2axang(params.controls.ang_v);
        if ctrl_mode == 1 % velocity control mode
            M = inv((J*J'));
            params.controls.dq = (J'*M)*[(axang(4)*axang(1:3))'; params.controls.pos_v];%(J'*J)\J'*
            detJ = norm(M);
            if(detJ>1000)
                detJ
            end
        elseif ctrl_mode == 2
            params.controls.dq = getqp_rslt(axang,J,params); % constrained velocity control mode
        elseif ctrl_mode == 3
            params.controls.dq = getqp_rslt_qlim(axang,J,params); % constrained velocity control mode with joint limits

            if params.keyboard.pause_flag == 1
                params.controls.dq = params.controls.dq/1.2;
                robot.setJointCommand('right',params.controls.dq);
                params.controls.pos_v = params.controls.pos_v/1.1;
                params.controls.ang_v = params.controls.ang_v/8;
                if norm(params.controls.pos_v)+norm(params.controls.ang_v(2:4)) <=0.0002
                    params.keyboard.pause_flag = 0;
                    params.controls.pos_v = [0,0,0]';
                    params.controls.ang_v = [1,0,0,0];
                    robot.setJointCommand('right',[0,0,0,0,0,0,0]');
                end
            else
                robot.setJointCommand('right',params.controls.dq);
            end
        elseif ctrl_mode == 4
            params.controls.dq = getqp_rslt_svd(axang,J,params); % constrained velocity control with SVD stablization
        elseif ctrl_mode == 5
            params.controls.dq = getqp_rslt_svd_qlim(axang,J,params); % constrained velocity control with SVD stablization and joint limits
            if params.keyboard.pause_flag == 1
                params.controls.dq = params.controls.dq/1.2;
                robot.setJointCommand('right',params.controls.dq);
                params.controls.pos_v = params.controls.pos_v/1.1;
                params.controls.ang_v = params.controls.ang_v/8;
                if norm(params.controls.pos_v)+norm(params.controls.ang_v(2:4)) <=0.0002
                    params.keyboard.pause_flag = 0;
                    params.controls.pos_v = [0,0,0]';
                    params.controls.ang_v = [1,0,0,0];
                    robot.setJointCommand('right',[0,0,0,0,0,0,0]');
                end
            else
                robot.setJointCommand('right',params.controls.dq);
            end
        end
        
%         freq = 1/t_elp
        scan_dist
    end
end


