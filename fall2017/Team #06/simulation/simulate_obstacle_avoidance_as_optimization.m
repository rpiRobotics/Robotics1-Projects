% Author: Zac Ravichandra
% Simulation of path planning by picking waypoints and formulating 
% as optimization problem.

% Obstacle Position
xo = 0;
yo = 0.4;

% plot obstacle
[xol, yol] = get_circle_lines(xo, yo, 0.05);
plot(xol, yol, 'color', 'b'); hold on;

% path in x, y
path = zeros(2,2);
path(1,:) = [-0.200000, 0.400000];
path(2,:) = [0.1, 0.5];

initial_conditions = zeros(3,2);

t = 1;
phi = 0;

for p=1:length(path)
    scatter(path(p,1) + initial_conditions(p,1), ...
        path(p,2) + initial_conditions(p,2), 'r'); hold on;
    [w, v, xf, yf] = get_w_v_from_gradient_descent(path(p,1), path(p,2), t, phi)
    [x, y] = duckiebot_step(v, w, t, phi);
    fprintf('x, y error = (%f %f)\n', abs(x-path(p,1)), abs(y-path(p,2)));
    p_0 = initial_conditions(p,:);
    
    % plot path forward
    for ti = linspace(0, t, 10)
        [xtf, ytf] = duckiebot_step(v, w, ti, phi);
        [xtl, ytl] = get_circle_lines(xtf + p_0(1), ytf + p_0(2), 0.01);
        plot(xtl, ytl, 'color', 'b'); hold on;
    end
    
    if p ~= length(path) 
        initial_conditions(p+1,:) = [x,y] + p_0; 
    end;
    
    phi = w * t
end

xlabel('x'); ylabel('y');
title('Sovling inverse kinematics for waypoints');
axis([-1.5 1.5 -0.5 2])



