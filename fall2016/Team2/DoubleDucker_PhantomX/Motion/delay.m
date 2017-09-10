function delay(seconds)
% delay: pausing function 
%   Used as opposed to 'pause' for accuaracy
%   seconds = delay time in seconds
%   
%   Input(s): seconds
%   Output(s): null

tic;

while toc < seconds
end

end
%https://www.mathworks.com/matlabcentral/answers/37716-pause-function-in-matlab-for-1-millisecond
