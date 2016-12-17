import numpy as np
import matplotlib.pyplot as plt
import DobotModel

data = np.loadtxt('/Users/peter/Documents/MATLAB/robotics/arm_cal.csv',delimiter=',')
angles = data[:,0:3]
xy_ref = data[:,3:5]*(25.4/5)
xy_arm = np.zeros(xy_ref.shape)

p0t = DobotModel.forward_kinematics(angles[0,:])
xy0 = p0t[0:2]
for k in range(xy_ref.shape[0]):
    p0t = DobotModel.forward_kinematics(angles[k,:])
    xy_arm[k,:] =  p0t[0:2] - xy0

np.savetxt('/Users/peter/Documents/MATLAB/robotics/data1.csv',np.hstack((angles,xy_ref,xy_arm)),delimiter=',')

#plt.ion()
#p = plt.plot(xy_ref[:,0],xy_ref[:,1],'+',xy_arm[:,0],xy_arm[:,1],'.')
#plt.xlabel('X (mm)')
#plt.ylabel('Y (mm)')
#plt.legend(('Reference','Arm'))
#plt.axis(np.array([-5,1,-5,5])*25.4)
#plt.grid()
#plt.xticks(np.unique(xy_ref[:,0]))
#plt.yticks(np.unique(xy_ref[:,1]))


