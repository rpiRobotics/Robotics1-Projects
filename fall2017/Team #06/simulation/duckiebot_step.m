function [x, y] = duckiebot_step(v, w, t, phi )
% Author: Zac Ravichandran
% computing the net change in the x and y direction given
% the Duckiebot's movement information

x = -v/w * cos(w*t + phi) + v/w *cos(phi);
y =  v/w * sin(w*t + phi) - v/w * sin(phi);

end

