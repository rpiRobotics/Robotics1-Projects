% connect to the python server
robot = RobotRaconteur.Connect('tcp://localhost:10001/dobotRR/dobotController');

% find location and orientation of paper wrt the camera
gotoAngles(robot,[0 0 0 0]);
bitmask=createMask;
[o,R,h,w]=cornerDetection(bitmask);

% determine current pen length
d=calcPenLength;

% determine a packing for several squares onto the paper, and draw them
%drawSquares(robot,R,o',[20 10 10 10],40,40,d+5.2);
%drawSquares(robot,R,o',[40 20 10 10],80,80,d+2.7);
drawSquares(robot,R,o',[5],30,30,d+3);
pause(.5);
gotoAngles(robot,[0 0 0 0]);
