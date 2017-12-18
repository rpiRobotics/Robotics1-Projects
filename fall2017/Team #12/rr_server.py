#! /usr/bin/env python
from DobotStatusMessage import DobotStatusMessage
from DobotSerialInterface import DobotSerialInterface
import time
import RobotRaconteur as RR
RRN = RR.RobotRaconteurNode.s
import settings
settings.init()

sign = lambda x: (1,-1)[x<0]

class DobotObject():
	def __init__(self, port):
		self.desired_joint_angles = [0,0,0,0];
		self.current_angle = [0,0,0,0];
		self.integer_part = [0,0,0,0];
		self.i=0;
		self.dobot_interface = DobotSerialInterface(port)
		self.dobot_interface.set_speed()
		self.dobot_interface.set_playback_config()
		time.sleep(1);
		self.dobot_interface.send_absolute_angles(self.desired_joint_angles[0], self.desired_joint_angles[1], self.desired_joint_angles[2], self.desired_joint_angles[3]) 
		
		
	def setJointPositions(self,q1,q2,q3,q4): 

		self.i=0;
		time.sleep(.25)
		temp=self.desired_joint_angles = [float(q1)/100, float(q2)/100, float(q3)/100, float(q4)/100]
		print temp,self.current_angle
		if((temp[0] != self.current_angle[0]) or (temp[1] != self.current_angle[1]) or (temp[2] != self.current_angle[2]) or (temp[3] != self.current_angle[3])):
			self.desired_joint_angles = temp
			self.current_angle = self.desired_joint_angles;
			print "\nPRESCRIBING NEW ANGLES :",self.desired_joint_angles, "\n"
			self.dobot_interface.send_absolute_angles(self.desired_joint_angles[0], self.desired_joint_angles[1], self.desired_joint_angles[2], self.desired_joint_angles[3]) 
		time.sleep(.25)	
		
		
		
	# def setJointPositions(self,q1,q2,q3,q4): 
		# if (self.i==0):
			# self.i=1;
			# self.integer_part = [q1,q2,q3,q4];
		# else:
			# self.i=0;
			# time.sleep(.25)
			# temp=self.desired_joint_angles = [sum(x) for x in zip(self.integer_part, [sign(self.integer_part[0])*float(q1)/1000,sign(self.integer_part[1])*float(q2)/1000,sign(self.integer_part[2])*float(q3)/1000,sign(self.integer_part[3])*float(q4)/1000])]
			# if((temp[0] != self.current_angle[0]) or (temp[1] != self.current_angle[1]) or (temp[2] != self.current_angle[2]) or (temp[3] != self.current_angle[3])):
				# self.desired_joint_angles = temp
				# self.current_angle = self.desired_joint_angles;
				# print "\nPRESCRIBING NEW ANGLES :",self.desired_joint_angles, "\n"
				# self.dobot_interface.send_absolute_angles(self.desired_joint_angles[0], self.desired_joint_angles[1], self.desired_joint_angles[2], self.desired_joint_angles[3]) 
			# time.sleep(.25)
			
	def getJointPositions(self):
		return self.dobot_interface.current_status.angles
		
	def loop(self):
		while 1:
			time.sleep(.01)
	
def main():    
    port = 10001       
    t1 = RR.LocalTransport()
    t1.StartServerAsNodeName("dobotRR")
    RRN.RegisterTransport(t1)

    t2 = RR.TcpTransport()
    t2.EnableNodeAnnounce()
    t2.StartServer(port)
    RRN.RegisterTransport(t2)
    
    my_dobot = DobotObject(settings.comport)
	
    with open('dobotRR.robodef', 'r') as f:
        service_def = f.read()
    
    RRN.RegisterServiceType(service_def)
    RRN.RegisterService("dobotController", "dobotRR.DobotObject", my_dobot)
    print "Connecting to tcp://localhost:" + str(port) + "/dobotRR/dobotController"
    my_dobot.loop()

    RRN.Shutdown()

if __name__ == '__main__':
    main()