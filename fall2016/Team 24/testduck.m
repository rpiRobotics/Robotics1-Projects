%W_max=15; about 2pi rad in one sec.
%tested multiple times to find out corrolation w/ real world speed
%at v=0.1 takes 7 seconds to cross one block. max speed not tested 
% so it's 0.7s for 0.6 m, 0.857 m/s at v=1.
v=0.1;w=0;T=5;
duck.sendCmd(v,w);
pause(T)
duck.sendCmd(0,0);  