function [steps] = rad2step(rads)
% rad2step: change from radians to steps for motors
%   Takes in the desired joint configuration in radians and converts it
%   into stepper motor position. Based off the zero configuration in
%   stepper motor position. See parameter, 'base.'
%   
%   Input(s): joint configuration in radians
%   Output(s): corresponding stepper motor, step position

deg = rads*(180/pi);
step_inc = deg/.29;
base = 500; 
base2 = 810; % use for vertical configuration
base3 = 520; % adjusted from arm calibration
steps = zeros(length(deg),1);
for i = 1:length(deg)
    if i == 3
        steps(i) = base3 - step_inc(i);
% uncomment for vertical configuration ***********************      
    elseif i == 4
        steps(i) = base2 + step_inc(i);

    else
        steps(i) = base + step_inc(i);
    end
end

steps = round(steps);
