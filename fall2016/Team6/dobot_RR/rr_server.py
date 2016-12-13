#! /usr/bin/env python
from DobotStatusMessage import DobotStatusMessage
from DobotSerialInterface import DobotSerialInterface
import time
import RobotRaconteur as RR


RRN = RR.RobotRaconteurNode.s


class DobotObject():
	def __init__(self, port):
		self.desired_joint_angles = [0,0,0,0];
		self.new_command = False;
		self.new_command2 = False;
		self.dobot_interface = DobotSerialInterface(port)
		self.dobot_interface.set_speed()
		self.dobot_interface.set_playback_config()
				
	def setJointPositions(self,q1,q2,q3,q4,grip):
		self.new_command = True;
		self.desired_joint_angles = [q1,q2,q3,q4,grip];
		
	def send_absolute_position(self, x, y, z, rot, grip, move_mode=1):
		self.new_command2 = True;
		self.desired_cart_coord=[True,x, y, z, rot,grip, move_mode];
		
	def getJointPositions(self):
		return self.dobot_interface.current_status.angles

	def getPositions(self):
		return self.dobot_interface.current_status.position
	def getGripperAngle(self):
		return self.dobot_interface.current_status.gripperAngle
		
	def loop(self):
		print 'hi'
		while 1:
			time.sleep(.01)
			if self.new_command:
				self.dobot_interface.send_absolute_angles(self.desired_joint_angles[0], self.desired_joint_angles[1], self.desired_joint_angles[2], self.desired_joint_angles[3], self.desired_joint_angles[4]) 
				self.new_command = False
			elif self.new_command2:
			      self.dobot_interface.send_absolute_position(self.desired_cart_coord[1],self.desired_cart_coord[2],self.desired_cart_coord[3],self.desired_cart_coord[4],self.desired_cart_coord[5])
			      self.new_command2 = False;   
	
def main():    


    port = 10001       
    t1 = RR.LocalTransport()
    t1.StartServerAsNodeName("dobotRR")
    RRN.RegisterTransport(t1)

    t2 = RR.TcpTransport()
    t2.EnableNodeAnnounce()
    t2.StartServer(port)
    RRN.RegisterTransport(t2)
    
    my_dobot = DobotObject('COM4')
	

    with open('dobotRR.robodef', 'r') as f:
            service_def = f.read()
    
    RRN.RegisterServiceType(service_def)
    RRN.RegisterService("dobotController", "dobotRR.DobotObject", my_dobot)
    print "Conect string: tcp://localhost:" + str(port) + "/dobotRR/dobotController"
    my_dobot.loop()

    RRN.Shutdown()


if __name__ == '__main__':
    main()

