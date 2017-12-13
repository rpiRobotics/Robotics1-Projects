function h_hat = hat( h )
%HAT Summary of this function goes here
%   Detailed explanation goes here
h_hat=[0 -h(3) h(2); h(3) 0 -h(1); -h(2) h(1) 0];
end

