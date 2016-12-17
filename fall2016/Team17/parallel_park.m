function parallel_park(duck)
duck.sendCmd(.3,0);
pause(.5);
duck.sendCmd(0,0);
pause(1);
duck.sendCmd(-.3,4);
pause(.65);
duck.sendCmd(0,0);
pause(1);
duck.sendCmd(-.3,-5);
pause(.5);
duck.sendCmd(0,0);
pause(.5);
duck.sendCmd(.3,-1);
pause(.3);
duck.sendCmd(0,0);
end