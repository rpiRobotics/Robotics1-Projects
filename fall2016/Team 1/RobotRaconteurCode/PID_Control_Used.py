import RobotRaconteur as RR
RRN=RR.RobotRaconteurNode.s

#Various constants used by the P controller
TRIM_CONSTANT = 2.7        # Used to set the trim. This makes the bot go forward when omega = 0
D_CONSTANT = 0.030700389   # Used to fix crappy lane_pose info. Based on 1,200 data points.
PHI_CONSTANT = 0.029922179 # Used to fix crappy lane_pose info. Baed on 1,200 data points. 
P_D_GAIN = 20.0            # Proportional-Distance gain. Determined experimentally.
P_PHI_GAIN = 7.5           # Proportional-Phi gain. Determined experimentally. 
VELOCITY = 0.6             # Forward Velocity of the bot. 

#Wrapper to rename the stop function to be faster to type
def s():
    duck.sendCmd(0,0)

if __name__ == '__main__':
    RRN.UseNumPy=True
    # Register a Local transport
    t1 = RR.LocalTransport()
    RRN.RegisterTransport(t1)

    # Register a TCP transpor
    t2 = RR.TcpTransport()
    RRN.RegisterTransport(t2)
    print("Connecting...")
    duck = RRN.ConnectService("tcp://10.13.215.108:2000/DuckiebotServer.duckieryan/Duckiebot")
    print("Connection Successful!")

    while(True):
        #P controller dependent upon d and phi
        #The values are too noisy and the timescale is too fast for derivative or integral control to be of any real use.
        a = duck.lane_pose.d   - D_CONSTANT   # Offset d based on calibration from 1200 data points at standstill
        b = duck.lane_pose.phi - PHI_CONSTANT # Offset phi based on calibrations from 1200 data points at standstill
        a = - P_D_GAIN * a
        b = - P_PHI_GAIN * b
        # Stop the bot by interrupting the loop and feeding the bot the s() function.
        duck.sendCmd(VELOCITY, a + b + VELOCITY * TRIM_CONSTANT)
        # Optionally print variables representing omega
        # print(" "*(20-len(str(a)))+str(a)+" "*(20-len(str(b)))+str(b)+" "*(20-len(str(a+b)))+str(a+b))
