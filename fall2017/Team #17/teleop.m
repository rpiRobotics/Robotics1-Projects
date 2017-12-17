function teleop(duck)
    h=figure;
    set(h,'KeyPressFcn',{@duckdrive,duck});    
end

function duckdrive(hobj,callbackdata,duck)
    T = 0.2;
    switch callbackdata.Key
        case{'uparrow'}
            duck.sendCmd(0.5,0);
            pause(T)
            %duck.sendCmd(0,0);            
        case{'downarrow'}
            duck.sendCmd(-0.5,0);
            pause(T)
            %duck.sendCmd(0,0); 
        case{'leftarrow'}
            duck.sendCmd(0.6, 3);
            pause(T)
            %duck.sendCmd(0,0); 
        case{'rightarrow'}
            duck.sendCmd(0.5, -4);
            pause(T)
            %duck.sendCmd(0,0); 
        %case{'s'}
            %duck.sendCmd(0,0);
            %color_command(duck);
        case{'d'}
            duck.sendCmd(0,0);
    end
end
