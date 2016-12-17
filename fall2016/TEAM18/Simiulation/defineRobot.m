%
% Define a robot for visualization in MATLAB
% 
% my_robot = definerobot(jointtype,H,P,n,scale)
% 
% jointtype= nx1 vector specifying type of joint (0=revolute, 1=primatic)
% H = 3xn matrix for joint motion axes
% P = 3x(n+1) matrix for link vectors
% n = # of joints
% scale = scaling constant for link width (should set nominally 1,
%       smaller for thinner links)
%
% Be sure to use rigidbodyviz_setup() before running
%

function [my_robot,my_robot_structure]=defineRobot(jointtype,H,P,n,scale)

% check if scale exist
if exist('scale')==0;scale=1;end

% some useful constants for defining kinematics
ex = [1;0;0]; ey = [0;1;0]; ez = [0;0;1]; zv = [0;0;0];

% create an empty robot 
my_robot =  defineEmptyRobot(1);

% give it a name
my_robot.name = 'myrobot';

% Kinematics
my_robot.kin.H = H;  
my_robot.kin.P = P;  
my_robot.kin.joint_type = jointtype; 

% Visualization constants

% Define each joint visualization
my_robot.vis.joints = struct('param',cell(1,2),'props',cell(1,2));

% Define each link visualization
my_robot.vis.links = struct('handle', cell(1,4), ...
    'R', cell(1,4), 't', cell(1,4), ...
    'param',cell(1,4),'props',cell(1,4));

% maximum link length (to help with scaling the plot)
maxP = sqrt(max(sum(P.*P,1)));
% some guess of the joint and link appearance
joint_r = maxP/10;
joint_h = 4*joint_r;
link_r = max(sum(P.*P,1))/10*scale;

% initial link (stationary)
my_robot.vis.links(1).handle = @createCylinder;
my_robot.vis.links(1).R = eye(3,3); 
my_robot.vis.links(1).t = P(:,1); 
my_robot.vis.links(1).param = struct('radius', link_r, 'height', norm(P(:,1)));
my_robot.vis.links(1).props = {};

for i=1:n

    % define joint based on type
    if jointtype(i)==0
        my_robot.vis.joints(i).param = struct('radius',joint_r,...
            'height',joint_h);
        if mod(i,3)==1
            facecolor=[1;0;0];
        elseif mod(i,3)==2
            facecolor=[.5;0;1];            
        elseif mod(i,3)==0
            facecolor=[.5;.2;.2];                        
        end
        my_robot.vis.joints(i).props = {'FaceColor', facecolor};      
    else
        my_robot.vis.joints(i).param = struct('width', joint_r, ...
            'length', joint_r, ...
            'height', joint_h, ...
            'sliderscale', 0.8);
        my_robot.vis.joints(i).props = {};
    end

    % define link (nominally in z direction, so need to rotate
    % based on P)
    my_robot.vis.links(i+1).handle = @createCylinder;
    k = cross(ez,P(:,i+1));
    if norm(k)>eps
        k=k/norm(k);
       theta=subproblem1(k,ez,P(:,i+1)/norm(P(:,i+1)));
    else
        k=[1;0;0];theta=0;
    end
    my_robot.vis.links(i+1).R = rot(k,theta); 
    my_robot.vis.links(i+1).t = P(:,i+1)/2; 
    my_robot.vis.links(i+1).param = struct('radius', link_r, ...
        'height', norm(P(:,i+1)));
    my_robot.vis.links(i+1).props = {'FaceColor', [0;1;0],'EdgeAlpha', 0};
end

% Define dimensions of coordinate frame
my_robot.vis.frame = struct('scale', 0.2, 'width', 0.01);

my_robot_structure=defineEmptyRobotStructure;
my_robot_structure.name = my_robot.name;
[my_robot_structure.create_properties] = deal({'CreateFrames','on'});
