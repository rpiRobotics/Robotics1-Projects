%% Autonomous Navigation
function auto_nav(duck)
% reference parameters: minimize error with respect to these parameters to
% maintain lane position
duck.getImage();
pose = duck.lane_pose;
d_zero = pose.d
phi_zero = pose.phi;
i = 0;

% color of the stop lines at intersections
red = [0.55 0.65];

% image size
height = 640;
width = 480;
n = 921600;

while i < 1000
    duck.getImage();
    pose = duck.lane_pose;
    d = pose.d
    phi = pose.phi;

    if d > d_zero
        % turn left
        duck.sendCmd(0.1,0.5);
    elseif d < d_zero
        % turn right
        duck.sendCmd(0.1,-0.5);
    else
        % straight
        duck.sendCmd(0.1,0);
    end
    
    
    capture = duck.getImage();

    % process image into RGB image format
    pixel_array = zeros(width, height, 3, 'uint8');
    pixel_array(:,:,1) = reshape(capture.data(1:3:n), height, width)';
    pixel_array(:,:,2)= reshape(capture.data(2:3:n), height, width)';
    pixel_array(:,:,3)= reshape(capture.data(3:3:n), height, width)';
    
    % search for red stop line at every iteration
    red_detected = filter_image(pixel_array, connected_percent, red);
    
    % if we detect it then we are at an intersection
    % color command determines which direction to take
    if red_detected
        color_command(duck);
    end

    i = i+1;
end
    
    duck.sendCmd(0,0);
end
