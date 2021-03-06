v = 1;

d = 2;
z = 1;
phi = 3;
q = [0; 0; 0];
dq = [0; 0; 0];

desired = 10;

omega_sub = rossubscriber('/path_plan/steer');
d_pub = rospublisher('path_plan/actual_lane_offset', 'std_msgs/Float32');
phi_pub = rospublisher('path_plan/lane_angle', 'std_msgs/Float32');
desired_pub = rospublisher('path_plan/desired_lane_offset', 'std_msgs/Float32');
pause(2);

desired_msg = rosmessage(desired_pub);
desired_msg.Data = desired;
send(desired_pub, desired_msg);

tmax = 3000;
t = 0;
qs = zeros(3, tmax);

while abs(desired - q(d)) > 0.0001 && t < tmax
    d_msg = rosmessage(d_pub);
    d_msg.Data = q(d);
    send(d_pub, d_msg);
    
    phi_msg = rosmessage(phi_pub);
    phi_msg.Data = q(phi);
    send(                                                           , phi_msg);
    
    omega_msg = receive(omega_sub);
    omega = omega_msg.Data;
    dq = [cos(q(phi)); sin(q(phi)); 0].*v + [0; 0; 1].*omega;
    q = q + dq;
    t = t+1;
    qs(:, t) = q;
end

plot(1:t, qs(d:phi, 1:t));
