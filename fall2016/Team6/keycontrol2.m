clc; clear; close all;
%key control of x,y,z position
robot = RobotRaconteur.Connect('tcp://localhost:10001/dobotRR/dobotController');
P = robot.getJointPositions();
q1=double(P(1)); q2=double(P(2)); q3=double(P(3)); rot = double(P(4));

%compute forward kinematics to get the current x,y,z of gripper
l1 = 103; %length from base to joint 2
l2 = 135; %length from joint 2 to joint 3
l3 = 160; %length from joint 3 to point where rotation unit is attached
Lg = 56;  %length from where rotation unit is attached to q4 rotation
d = 115;  %length from q4 to gripper end
%set up unit vector directions
ex = [1;0;0]; ey = [0;1;0]; ez = [0;0;1]; 
%joint connection vectors
p01 = l1*ez; p12 = 0*ez; p23 = l2*ez; p34=l3*ex; p4t=0*ez;
%vector from base to rotation unit connection point
p0t = p01+rotz(q1*pi/180)*(p12+roty(q2*pi/180)*p23+roty(q3*pi/180)*p34); 

xg = Lg*cos(q1*pi/180); %x component of vector from rot connection point to gripper end
yg = Lg*sin(q1*pi/180); %y component of vector from rot connection point to gripper end
%vector from base to gripper end
p0g = p0t+[xg;yg;-d];

%current x,y,z positions
x = p0g(1); y = p0g(2); z = p0g(3);
disp(x);disp(y);disp(z);

grip = 25; %set starting gripper angle to open
%send the startup joint angles to the robot -> robot usually does not
%respond to first command sent
robot.setJointPositions(int16(q1), int16(q2), int16(q3), int16(rot) , int16(grip) );

Q = [q1 q2 q3];
cancel = 0;
uiwait(msgbox({'The next screen allows you to input the keys to be used to move the robot.' 'Only letter keys may be used.' '* is reserved for exiting the program' ' ' 'A blank figure appears after inputting the key values.' 'This must remain open for the program to work'}));

while 1
    prompt = {'Move left key:','Move right key:','Move forward key:','Move back key:','Move up key:','Move down key:','Rotate left key','Rotate right key','Open key:','Close key:'};
    dlg_title = 'Input desired move keys - One character per line - * is reserved for exiting program';
    num_lines = 1;
    defaultans = {'0','0','0','0','0','0','0','0','0','0'};
    answer = inputdlg(prompt,dlg_title,num_lines,defaultans);
    anschar = char(answer);
    if isempty(anschar)
        cancel = 1;
        break
    elseif (size(anschar,1) == 10) && (size(anschar,2) == 1)
        if (anschar(1) == '*') || (anschar(2) == '*') || (anschar(3) == '*') || (anschar(4) == '*') || (anschar(5) == '*') || (anschar(6) == '*')  || (anschar(7) == '*') || (anschar(8) == '*') || (anschar(9) == '*') || (anschar(10) == '*')
            uiwait(msgbox('Input * is reserved for exiting'))
        elseif (anschar(1) == ' ') || (anschar(2) == ' ') || (anschar(3) == ' ') || (anschar(4) == ' ') || (anschar(5) == ' ') || (anschar(6) == ' ')  || (anschar(7) == ' ') || (anschar(8) == ' ') || (anschar(9) == ' ') || (anschar(10) == ' ')
            uiwait(msgbox('Input cannot be blank'))
        elseif (length(unique(anschar)) ~= length(anschar))
            uiwait(msgbox('Keys must be unique'))
        else
            left = anschar(1); right = anschar(2); forward = anschar(3); back = anschar(4);
            up = anschar(5); down = anschar(6); rotL = anschar(7); rotR = anschar(8);
            open = anschar(9); closed = anschar(10);
            button = questdlg({'Are these values correct?',strcat('left = ',left),strcat('right = ',right),strcat('forward = ',forward),strcat('backward = ',back),strcat('up = ',up),strcat('down = ',down),strcat('rotate left = ',rotL),strcat('rotate right = ',rotR),strcat('open = ',open),strcat('close = ',closed)},'Verify keys','No');
            if strcmp(button,'Yes')
                break
            end
        end
    else
        uiwait(msgbox('Each entry can only contain one character. Please re-enter values.'))
    end
end

Qprev = Q;
gripprev = grip;
rotprev = rot;
work = 0;
dy = 10;
dx = 10;
dz = 2;
dr = 5;
dg = 5;

while work == 0
    if cancel == 1
        disp('Program terminated')
        break
    end
    wwait = waitforbuttonpress;
    if wwait
         key = get(gcf,'CurrentCharacter');
         switch key
             case left
                disp('going left')
                y = y+dy;
                disp(y)
                [Q,error] = ikdobot(x,y,z);
                if Q(1) == 1i
                   disp('cannot go there') 
                   Q = Qprev;
                   y = y-dy;
                end
                disp(Q)
             case right
                disp('going right')
                y = y-dy;
                disp(y)
                [Q,error] = ikdobot(x,y,z);
                if Q(1) == 1i
                   disp('cannot go there') 
                   Q = Qprev;
                   y = y+dy;
                end
                disp(Q)
             case forward
                disp('going forward')
                x = x+dx;
                disp(x)
                [Q,error] = ikdobot(x,y,z);
                if Q(1) == 1i
                   disp('cannot go there') 
                   Q = Qprev;
                   x = x-dx;
                end
                disp(Q)
             case back
                disp('going back')
                x = x-dx;
                disp(x)
                [Q,error] = ikdobot(x,y,z);
                if Q(1) == 1i
                   disp('cannot go there') 
                   Q = Qprev;
                   x = x+dx;
                end
                disp(Q)
             case up
                disp('going up')
                z = z+dz;
                disp(z)
                [Q,error] = ikdobot(x,y,z);
                if Q(1) == 1i
                   disp('cannot go there') 
                   Q = Qprev;
                   z = z-dz;
                end
                disp(Q)
             case down
                disp('going down')
                z = z-dz;
                disp(z)
                [Q,error] = ikdobot(x,y,z);
                if Q(1) == 1i
                   disp('cannot go there') 
                   Q = Qprev;
                   z = z+dz;
                end
                disp(Q)
             case open
                disp('opening')
                grip = grip-dg;
                if grip < 20
                    grip = grip+dg;
                    disp('Cannot open further')
                end
                disp(grip)
             case closed
                disp('closing')
                grip = grip+dg;
                if grip > 60
                    grip = grip-dg;
                    disp('Cannot close further')
                end
                disp(grip)
             case rotL
                disp('rotating left')
                rot = rot+dr;
                if rot > 90
                    rot = rot-dr;
                    disp('Cannot rotate further')
                end
                disp(rot)
             case rotR
                disp('rotating right')
                rot = rot-dr;
                if rot < -90
                    rot = rot+dr;
                    disp('Cannot rotate further')
                end
                disp(rot)
             case '*'
                disp('Exiting Program')
                break
             otherwise
                disp('not a valid key')
         end   
    %if Q(1) ~= 1i    
        if z>3
            if (Q(1)~=Qprev(1)) || (Q(2)~=Qprev(2)) || (Q(3)~=Qprev(3))
                robot.setJointPositions(int16(Q(1)), int16(Q(2)), int16(Q(3)), int16(rot) , int16(grip) );
                robot.getJointPositions()
                disp([x;y;z])
            elseif grip ~= gripprev
                robot.setJointPositions(int16(Q(1)), int16(Q(2)), int16(Q(3)), int16(rot) , int16(grip) );
                robot.getJointPositions()
            elseif rot ~= rotprev
                robot.setJointPositions(int16(Q(1)), int16(Q(2)), int16(Q(3)), int16(rot) , int16(grip) );
                robot.getJointPositions()
            end
        else
            disp('Cannot go through the table')
        end
    %else 
    %    disp('Cannot go to this position')
    %end
    Qprev = Q;
    rotprev = rot;
    gripprev = grip;
    end
    pause(0.1)
end