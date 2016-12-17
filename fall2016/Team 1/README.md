Team 1: (No Project Name)

Barry Hu & Ryan Feldhausen

Duckiebot: duckieryan

In the SDCA folder is the code pertaining to our attempt at using
machine learning to get a self-driving duckiebot that would drive
more human-like. We do not recommend that you attempt to run the code.

The RobotRaconteurCode folder has all the Python code we did using
the Robot Raconteur interface. Our primary file is PID_Control_Used.py,
which contains code for our closed loop controller. Despite its name,
it is actually a P controller, not a PID controller. The main file for
our open loop control is Open_Control2.py. While it does move the 
duckiebot, it does not move it as desired. The other files are included
for full disclosure of our work.

The Robot Raconteur code is dependent on having the Robot Raconteur library
for Python and its dependencies. The connection to the duckiebot is made using
a specific IP address. If the IP address of the duckiebot differs from the one 
in the files, please change the code to use the duckiebot's current IP address.
