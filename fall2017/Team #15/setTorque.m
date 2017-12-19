function [x] = setTorque(omni,torques)
input = [torques; zeros(13,1)];

omni.externalInput = input;

x = omni.extraOutput;

end

