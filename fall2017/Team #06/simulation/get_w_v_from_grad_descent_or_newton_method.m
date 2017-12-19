function [w, v, x_f, y_f, u] = get_w_v_from_grad_descent_or_newton_method(xd, yd, t, phi)
% Author: Zac Ravichandran
% Get solved v and w from the best result from running gradient descent and
% Newton's method

x_t =@(w, v, t, phi) -v/w * cos(w *t + phi) + v/w *cos(phi);
y_t =@(w, v, t, phi) v/w * sin(w*t + phi) - v/w * sin(phi);

dx_w =@(w_1, v, t, phi) (cos(w_1*t + phi)*v)/w_1^2 + (sin(w_1*t + phi)*v*t)/w_1 - (cos(phi)*v)/w_1^2;
dy_w =@(w_1, v, y, phi) -sin(w_1*t + phi)*v/(w_1^2) + (cos(w_1*t + phi) *v*t)/w_1+ v*sin(phi)/(w_1^2);

dx_v =@(w, v, t, phi) -1/w * cos(w *t + phi) + 1/w *cos(phi);
dy_v =@(w,v, t, phi) 1/w * sin(w*t + phi) - 1/w * sin(phi);


r = 2000;
e_gd = zeros(r,1);

wv = [0.2, 0.2];
k = 0.3;

wv_pinv = wv;
e_pinv = zeros(r,1);
k_pinv = 0.1;

margin = 10^-3;
u_g = 0; u_n = 0;

for i = 1:r
    e_gd(i,1) = norm([yd;xd] - [y_t(wv(1), wv(2), t, phi);x_t(wv(1), wv(2), t, phi)]);
    dw =  k*[dy_w(wv(1), wv(2), t, phi) dy_v(wv(1), wv(2), t, phi);...
        dx_w(wv(1), wv(2), t, phi), dx_v(wv(1), wv(2), t, phi ) ]'...
        * ([y_t(wv(1), wv(2), t, phi); x_t(wv(1), wv(2), t, phi)] - [yd;xd]);
    
    wv = wv - k * dw';
    
    e_pinv(i) = norm([yd;xd] - [y_t(wv_pinv(1), wv_pinv(2), t, phi);x_t(wv_pinv(1), wv_pinv(2), t, phi)]);
    
    dw_pinv = [dy_w(wv_pinv(1), wv_pinv(2), t, phi), dy_v(wv_pinv(1), wv_pinv(2), t, phi);
               dx_w(wv_pinv(1), wv_pinv(2), t, phi), dx_v(wv_pinv(1), wv_pinv(2), t, phi)];
    
    wv_pinv = wv_pinv -  [k_pinv*pinv(dw_pinv(:,1)) * ([y_t(wv_pinv(1), wv_pinv(2), t, phi); x_t(wv_pinv(1), wv_pinv(2), t, phi)] - [yd;xd]),...
                        k_pinv*pinv(dw_pinv(:,2)) * ([y_t(wv_pinv(1), wv_pinv(2), t, phi); x_t(wv_pinv(1), wv_pinv(2), t, phi)] - [yd;xd])];
    
   
    if e_pinv(i) > margin
        u_n = i;
    end
    if e_gd(i) > margin
        u_g = i;
    end
end

if e_gd(end) < e_pinv(end) || 1
    w = wv(1); v=wv(2);
    x_f = x_t(wv(1), wv(2), t, phi); y_f = y_t(wv(1), wv(2), t, phi);
    used_gd = 1;
    
    u = u_g;
else
    w = wv_pinv(1); v=wv_pinv(2);
    x_f = x_t(wv_pinv(1), wv_pinv(2), t, phi); y_f = y_t(wv_pinv(1), wv_pinv(2), t, phi);
    used_pinv = 1;
    u = u_n;
end

end