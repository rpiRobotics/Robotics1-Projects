close all;
clear;
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
[ex,ey,ez,n,P,q,H,type,dq_bounds,q_bounds] = robotParams();

%% Initialize Control Parameters
%initial joint angles
% q = [0,0,0,0,0,0]';
% q = [0,-1.12,0,0,2.0,0]';
q = [pi/2,-0.3,0,0.58,0,-0.31,0.15]';
% q = [pi/2,-0.3,0,0.58,0,-0.8,0.15]';
[R,pos] = fwdkin(q,type,H,P,n);
orien = R2q(R)';
pos_v = [0,0,0]';
ang_v = [1,0,0,0];
dq = zeros(n,1);

%plot options
view_port = [-30 30];%currently can't change viewing angle during the process
long = max(sum(P,2));
ax_scle = 1.25;
axes_lim = [-long/ax_scle ax_scle*long -long/ax_scle ax_scle*long -long/ax_scle ax_scle*long];
% axes_lim = [-1.5 3.2 -1.5 3.2 -1.5 3.2];%modify this if a robot is too small or too large
key_info = {'KEYS: exit-''ESC'', stop-''p'', up-''uparrow'', down-''downarrow''','left-''leftarrow'', right-''rightarrow'',forward-''backslash'', backward-''enter'',', 'roll-''a'', pitch-''s'', yaw-''d'', rev roll-''q'', rev pitch-''w'', rev yaw-''e''',''};

%velocities
w_t = [0,0,0]';
v_t = [0,0,0]';

%keyboard controls
%define position and angle step
inc_pos_v = 0.05; % m/s
inc_ang_v = 5/180*pi; % rad/s

%optimization params
er = 0.005;
ep = 0.0005;
epsilon = 0;%legacy param for newton iters

%create a handle of these parameters for interactive modifications
params = ControlParams(ex,ey,ez,n,P,H,type,dq_bounds,q_bounds,q,dq,pos,orien,pos_v,ang_v,w_t,v_t,...
    epsilon,view_port,axes_lim,inc_pos_v,inc_ang_v,0,er,ep,0,0);

%% Display Robot Initial Position
%plot out with plotting settings
figure('keypressfcn',{@func_keypress,params});
hold on
[pp,RR] = robot_3d(q);
view(view_port);
axis(axes_lim);
grid on
dt = 0;
counter = 0;
%display loop
while ~params.controls.stop    
    if (counter~=0)        
        dt = toc; %take a time interval measurement     
    end    
    tic %start clocking
    counter = counter + 1;
    if (counter~=0)
        %update joint angles
        q_temp = params.controls.q + params.controls.dq*dt;%compute new q use time interval and velocities.
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
        hold off;
        [pp,RR] = robot_3d(params.controls.q);
        view(params.plots.view_port);
        axis(params.plots.axes_lim);
        grid on
        title(key_info);
        drawnow
        
        
        %update current position and orientation
        params.controls.pos = pp(:,end);
        params.controls.orien = R2q(RR(:,:,end))';
        
        %compute Jacobian
%         J = getJacobian(params.controls.q, params.def.type, params.def.H, params.def.P, params.def.n);
        J = getJacobian_task(params.controls.q, params.def.type, params.def.H, params.def.P, params.def.n);
                
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
        elseif ctrl_mode == 4
            params.controls.dq = getqp_rslt_svd(axang,J,params); % constrained velocity control with SVD stablization
        elseif ctrl_mode == 5
            params.controls.dq = getqp_rslt_svd_qlim(axang,J,params); % constrained velocity control with SVD stablization and joint limits
        end    
    end
end


