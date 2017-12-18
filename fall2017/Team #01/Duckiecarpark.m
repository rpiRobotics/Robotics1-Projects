%{
This program controls the duckiebot, and simulates
keystrokes to create an autonomous, parallel parking
duckiebot. First, the duckiebot follows a line using
lane pose parameters and proportional gain control.
Once an AR tag is detected, the duckiebot will
proceed to parallel park. The steering of the duckiebot
is also constrained to simulate a modern car.

Authors: Sam Ansaldo, Sean Thammakhoune, Yinsu Zhang
%}
import java.awt.*;
import java.awt.event.*;
%Create a Robot-object to do the key-pressing
rob=Robot;

%For speed of 0.2
%Divide by 5 for speed of 0.1
k1 = -2.5; %Distance gain
k2 = -5.5; %Phi gain
global curang;
curang = 0; %current angle
duckkeycar(duck); %graph the steering angle
pause(1)
%initial values:
for j=1:200
    y = duck.april_tags; %Get april tag information
    position = duck.lane_pose; % Get Lane Pose
    dist = position.d; %Get distance
    phi = position.phi; %Get angle
    
    % set angular velocity
    
     w = k1 * dist - k2 * phi; %set angular velocity
     duck.sendCmd(0.2, w); %lane following, speed is .2 m/s
     pause(.2);
     if(isempty(y)~=1)%If tag is detected
        disp("Detect Tag");
        pos = y{1}.pos; %get distance from AR tag
        disp(pos(1)); %display distance
        
        i = 0;
        
        while i == 0
            y = duck.april_tags;
            if(isempty(y)~=1)
                pos = y{1}.pos;
                disp(pos(1)+1);
           
                    if pos(1)+1 >= 1.120 && pos(1)+1 <= 1.130
                        i = 1;
                    %line up the duckiebot
                    elseif (pos(1)+1 >= 1.2)
                        figure(1)
                        rob.keyPress(KeyEvent.VK_UP);
                        pause(.1)
                        rob.keyPress(KeyEvent.VK_UP);
                        pause(3)
                    elseif (pos(1)+1 >= 1.130)
                        disp('is?')
                        figure(1)
                        rob.keyPress(KeyEvent.VK_UP);
                        pause(3)
                
                    elseif (pos(1)+1 <= 1.120)
                        disp('THis?')
                        rob.keyPress(KeyEvent.VK_DOWN);
                        pause(3)
                end
            end
        end
        
        %begin parallel park    
        if (1 == 1)	%check distance
            for i = 0:1:20
           
                rob.keyPress(KeyEvent.VK_LEFT);
                pause(.05);
            end
            for i = 0:1:4
                rob.keyPress(KeyEvent.VK_DOWN); %Straighten out
                pause(.1);
            end
            
            for i = 0:1:9       
                rob.keyPress(KeyEvent.VK_RIGHT); 
                pause(.05);
            end
            
            pause(.1)
            rob.keyPress(KeyEvent.VK_DOWN);
            pause(.1);
            
            for i = 0:1:20        
                rob.keyPress(KeyEvent.VK_RIGHT); 
                pause(.05);
            end
            
            for i = 0:1:3
                
                rob.keyPress(KeyEvent.VK_DOWN); 
                pause(.1);
            end
            
            for i = 0:1:10      
                rob.keyPress(KeyEvent.VK_LEFT); 
                pause(.05);
            end
            rob.keyPress(KeyEvent.VK_UP);
            pause(.1);
            break;
        end
     end
end
% stop the duckiebot
duck.sendCmd(0,0);