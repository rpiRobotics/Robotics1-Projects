import DobotModel
from keyboard_control import keyboard_control
from campose import get_pose

# Capture some poses with the camera
# For the last one set the end effector on the AR tag
(angle_list,pca_list) = keyboard_control('/dev/ttyACM0')
(ptc_est,Rtc_est,p0a_est) = get_pose(angle_list[0:-1],pca_list[0:-1])
p0a = DobotModel.forward_kinematics(angle_list[-1])
ptc_est[2] = ptc_est[2] + p0a[2] - p0a_est[2] # could use all elements if confident about p0a
#ptc_est = ptc_est + np.reshape(p0a,(3,1)) - p0a_est
print ptc_est
print Rtc_est
