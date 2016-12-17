
function Main
    global i;
    global c;
    global X;
    global Y;
    global Z;
    global robot;
   robot = RobotRaconteur.Connect('tcp://10.13.215.145:10001/phantomXRR/phantomXController');
   %robot = RobotRaconteur.Connect('tcp://localhost:10001/phantomXRR/phantomXController')
  % trossen_calibrate(robot);
  global poff
   poff=input('define bottom right of page [x;y;z]');
   
    Z=1;
    X=0;
    Y=0;
    c=1;
    i=1;
    [x y]=ginput(1);
    axis([0 10 0 7])
    i=1;
    set(gcf,'WindowButtonMotionFcn', @mouseMove);
end


function mouseMove(object, eventdata)
    global i;
    global c;
    global X;
    global Y;
    global Z;
    global poff
    point=get(gca,'CurrentPoint');
    point=point(1,:);
    set(gcf,'WindowButtonDownFcn', @clicky)
    set(gcf,'WindowButtonUpFcn',@up)   
    X(i)=point(1,1)+poff(1,1);
    Y(i)=point(1,2)+poff(2,1);
    Z(i)=c +poff(3,1);
   P=[X(i);Y(i);Z(i)];
   drawing(P);
    i=i+1;
    plot(X,Y);
    axis([0 1 0 1]);
end

function clicky(object, eventdata)
    global c;
    c=0;
end

function up(object, eventdata)
    global c;
    c=1;
end
    
