#!/usr/bin/python

import sys
import os
import fcntl
import time
import rospy
import roslib
import numpy as np
from std_msgs.msg import Float64
from baxter_pykdl import baxter_kinematics
import matplotlib.pyplot as plt


def convert_dict_to_list(arm,_dict):
    '''
    Getting joint info from msg file is in this order:
    w0,w1,w2,e0,e1,s0,s1
    
    Should be:
    s0,s1,e0,e1,w0,w1,w2
    '''
    dictlist=[]
    dictlist.append(_dict[arm+'_s0'])
    dictlist.append(_dict[arm+'_s1'])
    dictlist.append(_dict[arm+'_e0'])
    dictlist.append(_dict[arm+'_e1'])
    dictlist.append(_dict[arm+'_w0'])
    dictlist.append(_dict[arm+'_w1'])
    dictlist.append(_dict[arm+'_w2'])    
    return dictlist

def convert_mat_to_list(mat):
    matlist=[]
    for i in range(0,np.shape(mat)[0]):
        matlist.append(float(mat[i][0]))
    return matlist

def convert_list_to_dict(keys,values):
    from collections import OrderedDict
    
    result = OrderedDict(zip(keys, values))
    return result

def stop_joints(_dict):
    for key in _dict:
        _dict[key]=0        
        
def return_to_init(dict_keys,arm_name):
    from baxter_interface import Limb    
    '''
    Joint angles that give us desired initial position:
    {'left_w0': -0.029529130166794215, 'left_w1': 0.08436894333369775, 'left_w2': -0.08782040010643993, 
    'left_e0': 0.10009224640952324, 'left_e1': 0.03604854851530722, 
    'left_s0': -0.8061069040337849, 'left_s1': -0.13690778531877318}
    '''
    init_pos=[-0.8061069040337849,-0.13690778531877318,0.10009224640952324,0.03604854851530722,
              -0.029529130166794215,0.08436894333369775,-0.08782040010643993]    
    init_pos_dict=convert_list_to_dict(dict_keys,init_pos)
    arm=Limb(arm_name)
    t_end=time.time()+4                    # Loop timer (in seconds)
    while time.time()<t_end:    
        arm.set_joint_positions(init_pos_dict)

    
def main():
    rospy.init_node('baxter_kinematics')    
    kin = baxter_kinematics('left')        
    pos = [0.582583, -0.180819, 0.216003]
    rot = [0.03085, 0.9945, 0.0561, 0.0829]
    
    fl = fcntl.fcntl(sys.stdin.fileno(), fcntl.F_GETFL)
    fcntl.fcntl(sys.stdin.fileno(), fcntl.F_SETFL, fl | os.O_NONBLOCK)
    
    # Read initial positions:
    from sensor_msgs.msg import JointState
    from baxter_interface import Limb

    right_arm=Limb('right')
    left_arm=Limb('left')

    joints=left_arm.joint_names()
    positions=convert_dict_to_list('left',left_arm.joint_angles())
    velocities=convert_dict_to_list('left',left_arm.joint_velocities())
    force=convert_dict_to_list('left',left_arm.joint_efforts())                

    positions=np.reshape(positions,[7,1])               # Converts to matrix       
    velocities=np.reshape(velocities,[7,1])             # Converts to matrix
    force=np.reshape(force,[7,1])                       # Converts to matrix  
    
    # Initial states
    q_previous=positions                            # Starting joint angles
    q_dot_previous=velocities                       # Starting joint velocities
    x_previous=kin.forward_position_kinematics()    # Starting end effector position
    x_dot_previous=np.zeros((6,1))
    
    # Set model parameters:
    C=50
    B=1
    f_des=np.reshape(np.array([0,15,0,0,0,0]),[6,1])   
    
    J=kin.jacobian()
    J_T=kin.jacobian_transpose()
    J_PI=kin.jacobian_pseudo_inverse()
    J_T_PI=np.linalg.pinv(J_T)
    
    x=kin.forward_position_kinematics()
    x=x[:-1]
    x_ref=np.reshape(x,[6,1])               # Reference position
    
    x_ref_dot=J*velocities                  # Reference velocities
    
    t_stop=15
    t_end=time.time()+t_stop                    # Loop timer (in seconds)
    
    # Initial plot parameters
    time_cum=0
    time_vec=[time_cum]
    
    f=J_T_PI*force
    force_vec=[convert_mat_to_list(f[1])[0]]
    
    while time.time()<t_end:
        from sensor_msgs.msg import JointState
        from baxter_interface import Limb
    
        right_arm=Limb('right')
        left_arm=Limb('left')       
        
        J=kin.jacobian()
        J_T=kin.jacobian_transpose()
        J_PI=kin.jacobian_pseudo_inverse()
        J_T_PI=np.linalg.pinv(J_T)    
        
        joints=left_arm.joint_names()
        positions=convert_dict_to_list('left',left_arm.joint_angles())
        velocities=convert_dict_to_list('left',left_arm.joint_velocities())
        force=convert_dict_to_list('left',left_arm.joint_efforts())                
        
        positions=np.reshape(positions,[7,1])               # Converts to matrix       
        velocities=np.reshape(velocities,[7,1])             # Converts to matrix
        force=np.reshape(force,[7,1])                       # Converts to matrix      
        
        x=kin.forward_position_kinematics()
        x=x[:-1]
        x_current=np.reshape(x,[6,1])
        
        x_dot_current=J*velocities
        
        # Only interested in y-axis:
        x_dot_current[0]=0
        #x_dot_current[1]=0
        x_dot_current[2]=0
        x_dot_current[3]=0
        x_dot_current[4]=0
        x_dot_current[5]=0                
        
        # Model goes here
        f=J_T_PI*force                # spacial force

        # Only interested in y-axis:
        f[0]=[0]
        #f[1]=[0]                  
        f[2]=[0]
        f[3]=[0]                       
        f[4]=[0]
        f[5]=[0]
        

        x_dot_des=-B*(f_des+f)/C                         # Impedance control
        
        # Control robot                
        q_dot_ctrl=J_PI*x_dot_des                           # Use this for damper system
        q_dot_ctrl=np.multiply(q_dot_ctrl,np.reshape(np.array([1,0,0,0,0,0,0]),[7,1]))  # y-axis translation corresponds to first shoulder joint rotation
        
        q_dot_ctrl_list=convert_mat_to_list(q_dot_ctrl)
        q_dot_ctrl_dict=convert_list_to_dict(joints,q_dot_ctrl_list)
        
        left_arm.set_joint_velocities(q_dot_ctrl_dict)      # Velocity control function
        
        # Update values before next loop
        x_previous=x_current                        # Updates previous position for next loop iteration
        x_dot_previous=x_dot_current                # Updates previous velocity for next loop iteration
        
        # Update plot info
        time_cum+=.05                               
        time_vec.append(time_cum)
        
        force_vec.append(convert_mat_to_list(f[1])[0])
        
    
    print(time_vec)
    print(force_vec)
    plt.plot(time_vec,force_vec)
    plt.title('Force applied over time')
    plt.xlabel('Time (sec)')
    plt.ylabel('Force (N)')
    plt.show()
    
    time.sleep(1)
            
if __name__ == "__main__":
    main()