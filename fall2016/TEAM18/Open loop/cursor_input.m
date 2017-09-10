
function Main
    global i;
    global X;
    global Y;
    global Z;
    Z=0;
    X=0;
    Y=0;
    [x y]=ginput(1);
    hold;
    i=1;
    set(gcf,'WindowButtonMotionFcn', @mouseMove);
end


function mouseMove(object, eventdata)
    global i;
    global X;
    global Y;
    global Z;
    point=get(gca, 'CurrentPoint');
   set(gcf,'WindowButtonDownFcn', @clicky)
    set(gcf,'WindowButtonUpFcn',@up)   
    X(i)=point(1,1);
    Y(i)=point(1,2);
    Z(i)=c;
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
    


