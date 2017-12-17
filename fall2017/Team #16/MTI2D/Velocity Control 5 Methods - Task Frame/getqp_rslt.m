function [dq] = getqp_rslt(axang,J,params)
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

LB = [-bound,0,0]';
UB = [bound,1,1]';
%quadratic programming
options = optimset('Display', 'off');
dq_sln = quadprog(H,f,[],[],[],[],LB,UB,params.controls.dq,options);        
%assign results
dq = dq_sln(1:params.def.n);

%check velocity direction
V_desired = params.controls.pos_v
V_now = J(4:6,:)*params.controls.dq        
V_scaled = dq_sln(end)*V_desired;

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

