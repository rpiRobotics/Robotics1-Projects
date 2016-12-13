clc; clear; close all;
%key control of individual joint angles

%connect to RobotRaconteur Service
robot = RobotRaconteur.Connect('tcp://localhost:10001/dobotRR/dobotController');

P = robot.getJointPositions();
q1=double(P(1)); q2=double(P(2)); q3=double(P(3)); rot = double(P(4));
grip = 25;
%send the startup joint angles to the robot -> robot usually does not
%respond to first command sent
robot.setJointPositions(int16(q1), int16(q2), int16(q3), int16(rot) , int16(grip) );

cancel = 0; %variable to check if the dialog screen is canceled

%dialog box to tell the user what to input for the dialog box
uiwait(msgbox({'The next screen allows you to input the keys to be used to move the robot.' 'Only letter keys may be used.' '* is reserved for exiting the program' ' ' 'A blank figure appears after inputting the key values.' 'This must remain open for the program to work'}));

%% get the desired key controls to be used in the program
while 1
    prompt = {'Increase q1 key:','Decrease q1 key:','Increase q2 key:','Decrease q2 key:','Increase q3 key:','Decrease q3 key:','Rotate left key','Rotate right key','Open key:','Close key:'};
    dlg_title = 'Input desired move keys - One character per line - * is reserved for exiting program';
    num_lines = 1;
    defaultans = {'0','0','0','0','0','0','0','0','0','0'};
    answer = inputdlg(prompt,dlg_title,num_lines,defaultans);
    anschar = char(answer); %convert inputs from struct to char
    if isempty(anschar)
        %allows user to cancel program by canceling dialog box
        cancel = 1;
        break
    elseif (size(anschar,1) == 10) && (size(anschar,2) == 1)
        if (anschar(1) == '*') || (anschar(2) == '*') || (anschar(3) == '*') || (anschar(4) == '*') || (anschar(5) == '*') || (anschar(6) == '*')  || (anschar(7) == '*') || (anschar(8) == '*') || (anschar(9) == '*') || (anschar(10) == '*')
            %make sure default exit key is not used
            uiwait(msgbox('Input * is reserved for exiting'))
        elseif (anschar(1) == ' ') || (anschar(2) == ' ') || (anschar(3) == ' ') || (anschar(4) == ' ') || (anschar(5) == ' ') || (anschar(6) == ' ')  || (anschar(7) == ' ') || (anschar(8) == ' ') || (anschar(9) == ' ') || (anschar(10) == ' ')
            %make sure no inputs are left blank
            uiwait(msgbox('Input cannot be blank'))
        elseif (length(unique(anschar)) ~= length(anschar))
            %make sure no duplicate keys are used
            uiwait(msgbox('Keys must be unique'))
        else
            %store inputs as variables and ask user if values are correct
            q1p = anschar(1); q1n = anschar(2); q2p = anschar(3); q2n = anschar(4);
            q3p = anschar(5); q3n = anschar(6); rotp = anschar(7); rotn = anschar(8);
            open = anschar(9); closed = anschar(10);
            button = questdlg({'Are these values correct?',strcat('increase 1 = ',q1p),strcat('decrease 1 = ',q1n),strcat('increase 2 = ',q2p),strcat('decrease 2 = ',q2n),strcat('increase 3 = ',q3p),strcat('decrease 3 = ',q3n),strcat('rotate left = ',rotp),strcat('rotate right = ',rotn),strcat('open = ',open),strcat('close = ',closed)},'Verify keys','No');
            if strcmp(button,'Yes')
                break
            end
        end
    else
        %make sure multiple keys are not used in a single input
        uiwait(msgbox('Each entry can only contain one character. Please re-enter values.'))
    end
end

d1 = 3; %amount to change joint angle 1 by (degrees)
d2 = 3; %amount to change joint angle 2 by (degrees)
d3 = 3; %amount to change joint angle 3 by (degrees)
dr = 5; %amount to change rotation angle by (degrees)
dg = 5; %amount to change gripper angle by (degrees)

work = 0;
while work == 0
    if cancel == 1
        %end program if input dialog box is canceled
        disp('Program terminated')
        break
    end
    wwait = waitforbuttonpress; %wait for keyboard button press
    if wwait
         key = get(gcf,'CurrentCharacter'); %get keyboard character
         switch key
             case q1p
                disp('increasing joint 1')
                q1 = q1+d1;
                if q1 > 134
                    q1 = q1-d1;
                    disp('cannot increase q1 further')
                end
                Q = [q1 q2 q3 rot grip];
                
             case q1n
                disp('decreasing joint 1')
                q1 = q1-d1;
                if q1 < -134
                    q1 = q1+d1;
                    disp('cannot decrease q1 further')
                end
                Q = [q1 q2 q3 rot grip];

             case q2p
                disp('increasing joint 2')
                q2 = q2+d2;
                if q2 > 84
                    q2 = q2-d2;
                    disp('cannot increase q2 further')
                end
                P = robot.getPositions();
                if P(3) <= 10
                    q2 = q2-d2;
                    display('cannot go through table')
                end
                Q = [q1 q2 q3 rot grip];
                
             case q2n
                disp('decreasing joint 2')
                q2 = q2-d2;
                if q2 < -4
                    q2 = q2+d2;
                    disp('cannot decrease q2 further')
                end
                Q = [q1 q2 q3 rot grip];
                
             case q3p
                disp('increasing joint 3')
                q3 = q3+d3;
                if q3 > 94
                    q3 = q3-d3;
                    disp('cannot increase q3 further')
                end
                P = robot.getPositions();
                if P(3) <= 10
                    q3 = q3-d3;
                    display('cannot go through table')
                end
                Q = [q1 q2 q3 rot grip];
                
             case q3n
                disp('decreasing joint 3')
                q3 = q3-d3;
                if q3 < -9
                    q3 = q3+d3;
                    disp('cannot decrease q3 further')
                end
                Q = [q1 q2 q3 rot grip];
                
             case open
                disp('opening')
                grip = grip-dg;
                if grip < 20
                    grip = grip+dg;
                    disp('Cannot open further')
                end
                Q = [q1 q2 q3 rot grip];
                disp(grip)
             case closed
                disp('closing')
                grip = grip+dg;
                if grip > 60
                    grip = grip-dg;
                    disp('Cannot close further')
                end
                Q = [q1 q2 q3 rot grip];
                disp(grip)
             case rotp
                disp('rotating left')
                rot = rot+dr;
                if rot > 90
                    rot = rot-dr;
                    disp('Cannot rotate further')
                end
                Q = [q1 q2 q3 rot grip];
                disp(rot)
             case rotn
                disp('rotating right')
                rot = rot-dr;
                if rot < -90
                    rot = rot+dr;
                    disp('Cannot rotate further')
                end
                Q = [q1 q2 q3 rot grip];
                disp(rot)
             case '*'
                disp('Exiting Program')
                break
             otherwise
                disp('not a valid key')
         end   
         robot.setJointPositions(int16(Q(1)), int16(Q(2)), int16(Q(3)), int16(rot) , int16(grip) );
        
    end
    pause(0.1)
end