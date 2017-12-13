classdef ControlParams < handle
    properties
        def;% = struct('ex',{},'ey',{},'ez',{},'n',{},'P',{},'H',{},'type',{});
        controls;% = struct('q',{},'w_t',{},'v_t',{});
        plots;% = struct('view_port',{},'axes_lim',{});
        keyboard;% = struct('inc_pos',{},'inc_ang',{});
        opt;
    end
    methods
    function params = ControlParams(ex,ey,ez,n,P,H,type,dq_bounds,q_bounds,q,dq,pos,orien,pos_v,ang_v,...
            w_t,v_t,epsilon,view_port,axes_lim,inc_pos_v,inc_ang_v,stop,er,ep,upper_dq_bounds,dt)
        if nargin > 0
            params.def.ex = ex;
            params.def.ey = ey;
            params.def.ez = ez;
            params.def.n = n;
            params.def.P = P;        
            params.def.H = H;
            params.def.type = type;
            params.def.dq_bounds = dq_bounds;
            params.def.q_bounds = q_bounds;
            params.controls.q = q;
            params.controls.dq = dq;
            params.controls.pos = pos;
            params.controls.orien = orien;
            params.controls.pos_v = pos_v;
            params.controls.ang_v = ang_v;
            params.controls.w_t = w_t;
            params.controls.v_t = v_t;
            params.controls.epsilon = epsilon;
            params.plots.view_port = view_port;
            params.plots.axes_lim = axes_lim;
            params.keyboard.inc_pos_v = inc_pos_v;
            params.keyboard.inc_ang_v = inc_ang_v;
            params.controls.stop = stop;
            params.opt.er = er;
            params.opt.ep = ep;
            params.opt.upper_dq_bounds = upper_dq_bounds;
            params.controls.dt = dt;
        end
    end
    end
end