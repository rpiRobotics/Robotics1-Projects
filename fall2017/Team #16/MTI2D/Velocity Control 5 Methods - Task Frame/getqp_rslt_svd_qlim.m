function [dq] = getqp_rslt_qlim(axang,J,params)
%params for qp   
vr = (axang(4)*axang(1:3))';
H = getqp_H(params.controls.dq, J, vr, params.controls.pos_v, params.opt.er, params.opt.ep);
f = getqp_f(params.controls.dq,params.opt.er,params.opt.ep);

%bounds for qp
if(params.opt.upper_dq_bounds)
    bound = params.def.dq_bounds(2,:);
else
    bound = params.def.dq_bounds(1,:);
end            

LB = [-0.1*bound,0,0]';
UB = [0.1*bound,1,1]';

%add joint limits
n = params.def.n;
q_lim_upper = params.def.q_bounds(:,2);
q_lim_lower = params.def.q_bounds(:,1);

k1_qlim_thr = 0.8; % when should the near-joint-limit slowing down kicks in.
k2_qlim_thr = 0.9; % when should the near-joint-limit push-off kicks in.
ita = 1-k2_qlim_thr; % Level-2 budget threshold
epsilon = k2_qlim_thr-k1_qlim_thr; % Level-1 budget threshold

ub_check = 1-params.controls.q./q_lim_upper;
lb_check = 1-params.controls.q./q_lim_lower;
%check threshold 1
ub_ck_idx_1 = ub_check <= ita+epsilon & ub_check >ita;
lb_ck_idx_1 = lb_check <= ita+epsilon & lb_check >ita;
%check threshold 2
ub_ck_idx_2 = ub_check <= ita & ub_check >0;
lb_ck_idx_2 = lb_check <= ita & lb_check >0;
%check negative situation
ub_ck_idx_neg = ub_check <=0;
lb_ck_idx_neg = lb_check <=0;

c = 0.9;
e = 0.5*min(UB(1:n));

A_neg = zeros(n+2,1); %QP requires A*x<=b
b_neg = zeros(n+2,1); %here we use A_neg*x>=b_neg

A_neg(1:n) = 1;
b_neg(1:n) = -tan(c*pi/2);

if sum(ub_ck_idx_neg)>=1 %Infeasible start, upper bound, push to the negative direction
    A_neg(ub_ck_idx_neg) = -1;
    b_neg(ub_ck_idx_neg) = e;
end

if sum(lb_ck_idx_neg)>=1 %Infeasible start, lower bound, push to the positive direction
    A_neg(lb_ck_idx_neg) = 1;
    b_neg(lb_ck_idx_neg) = e;
end

if sum(ub_ck_idx_2)>=1 %level-2 upper bound, push to the negative direction
    A_neg(ub_ck_idx_2) = -1;
    b_neg(ub_ck_idx_2) = e*(ita-ub_check(ub_ck_idx_2))/ita;
end

if sum(lb_ck_idx_2)>=1 %level-2 lower bound, push to the positive direction
    A_neg(lb_ck_idx_2) = 1;
    b_neg(lb_ck_idx_2) = e*(ita-ub_check(lb_ck_idx_2))/ita;
end

if sum(ub_ck_idx_1)>=1 %level-1 upper bound, positive dir slow down
    A_neg(ub_ck_idx_1) = 1;
    b_neg(ub_ck_idx_1) = -tan(c*pi*(ub_check(ub_ck_idx_1)-ita)/2/epsilon);
end

if sum(lb_ck_idx_1)>=1 %level-1 lower bound, negative dir slow down
    A_neg(lb_ck_idx_1) = -1;
    b_neg(lb_ck_idx_1) = -tan(c*pi*(lb_check(lb_ck_idx_1)-ita)/2/epsilon);
end

%quadratic programming
options = optimset('Display', 'off');
dq_sln = quadprog(H,f,diag(-A_neg),-b_neg,[],[],LB,UB,params.controls.dq,options);        
%assign results
dq = dq_sln(1:params.def.n);
% if params.def.n<=6
    [~,S,V]=svd(J); 
    dq = V'*dq;
    dq(diag(S)<=0.05) = 0;
    dq = V*dq;
% end

%check velocity direction
V_desired = params.controls.pos_v
V_now = J(4:6,:)*params.controls.dq        
V_scaled = dq_sln(end)*V_desired;

diag(-A_neg);

q_lower_upper_dq=[params.controls.q,params.def.q_bounds,dq]

if norm(V_now)>=1e-3 && norm(V_scaled)>=1e-3
    direrr =  1-abs(dot(V_now/norm(V_now),V_scaled/norm(V_scaled)));
    if direrr<1-cosd(1)
        disp('following direction');
    elseif direrr<1-cosd(5)

        fprintf('following direction imprecisely, dir mismatch(degrees): %.2f, max joint velo: %.4f, ap: %4f\n',acosd(1-direrr),max(abs(params.controls.dq)),dq_sln(end)); 
    else
        fprintf('Wrong direction, dir mismatch(degrees): %.2f, max joint velo: %.4f, ap: %4f\n',acosd(1-direrr),max(abs(params.controls.dq)),dq_sln(end));
    end
else
    fprintf('Zero Velocity Occurs\n');
end

% dq

end

