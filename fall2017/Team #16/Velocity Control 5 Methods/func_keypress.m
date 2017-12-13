function [] = func_keypress(~,event,params)
%% Define Keyboard Functions
if (strcmp(event.Key,'uparrow'))
    disp('up');
    params.controls.pos_v = params.controls.pos_v + [0,0,params.keyboard.inc_pos_v]';
    params.controls.ang_v = params.controls.ang_v;
elseif (strcmp(event.Key,'downarrow'))
    disp('down');
    params.controls.pos_v = params.controls.pos_v + [0,0,-params.keyboard.inc_pos_v]';
    params.controls.ang_v = params.controls.ang_v;
elseif (strcmp(event.Key,'leftarrow'))
    disp('left');
    params.controls.pos_v = params.controls.pos_v + [0,params.keyboard.inc_pos_v,0]';
    params.controls.ang_v = params.controls.ang_v;
elseif (strcmp(event.Key,'rightarrow'))
    disp('right');
    params.controls.pos_v = params.controls.pos_v + [0,-params.keyboard.inc_pos_v,0]';
    params.controls.ang_v = params.controls.ang_v;
elseif (strcmp(event.Key,'backslash'))
    disp('forward');
    params.controls.pos_v = params.controls.pos_v + [params.keyboard.inc_pos_v,0,0]';
    params.controls.ang_v = params.controls.ang_v;
elseif (strcmp(event.Key,'return'))
    disp('backward');
    params.controls.pos_v = params.controls.pos_v + [-params.keyboard.inc_pos_v,0,0]';
    params.controls.ang_v = params.controls.ang_v;
elseif (strcmp(event.Key,'a'))
    disp('roll');
    params.controls.pos_v = params.controls.pos_v;    
    params.controls.ang_v = quatmultiply([cos(params.keyboard.inc_ang_v/2),...
        sin(params.keyboard.inc_ang_v/2),0,0],params.controls.ang_v);
elseif (strcmp(event.Key,'s'))
    disp('pitch');
    params.controls.pos_v = params.controls.pos_v;    
    params.controls.ang_v = quatmultiply([cos(params.keyboard.inc_ang_v/2),...
        0,sin(params.keyboard.inc_ang_v/2),0],params.controls.ang_v);
elseif (strcmp(event.Key,'d'))
    disp('yaw');
    params.controls.pos_v = params.controls.pos_v;    
    params.controls.ang_v = quatmultiply([cos(params.keyboard.inc_ang_v/2),...
        0,0,sin(params.keyboard.inc_ang_v/2)],params.controls.ang_v);
elseif (strcmp(event.Key,'q'))
    disp('reverse roll');
    params.controls.pos_v = params.controls.pos_v;    
    params.controls.ang_v = quatmultiply([cos(-params.keyboard.inc_ang_v/2),...
        sin(-params.keyboard.inc_ang_v/2),0,0],params.controls.ang_v);
elseif (strcmp(event.Key,'w'))
    disp('reverse pitch');
    params.controls.pos_v = params.controls.pos_v;    
    params.controls.ang_v = quatmultiply([cos(-params.keyboard.inc_ang_v/2),...
        0,sin(-params.keyboard.inc_ang_v/2),0],params.controls.ang_v);
elseif (strcmp(event.Key,'e'))
    disp('reverse yaw');
    params.controls.pos_v = params.controls.pos_v;    
    params.controls.ang_v = quatmultiply([cos(-params.keyboard.inc_ang_v/2),...
        0,0,sin(-params.keyboard.inc_ang_v/2)],params.controls.ang_v);
elseif (strcmp(event.Key,'p'))
    disp('robot stopped');
    params.controls.pos_v = [0,0,0]';    
    params.controls.ang_v = [1,0,0,0];
elseif (strcmp(event.Key,'escape'))
    params.controls.stop = 1;
else
    disp(event.Key)
    disp('key choices: exit-''ESC'', stop-''p'', up-''uparrow'', down-''downarrow'', left-''leftarrow'', right-''rightarrow'',forward-''\'', backward-''enter'', roll-''a'',pitch-''s'',yaw-''d'',rev roll-''q'',rev pitch-''w'',rev yaw-''e''');
end

end

