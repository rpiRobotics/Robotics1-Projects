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


def derivative(previous,current,time_step):
    return (current-previous)/time_step

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
    positions=left_arm.joint_angles()
    velocities=left_arm.joint_velocities()
    force=convert_dict_to_list('left',left_arm.joint_efforts())
    
    # Initial states
    q_previous=positions                                # Starting joint angles
    q_dot_previous=velocities                           # Starting joint velocities
    x_previous=kin.forward_position_kinematics()        # Starting end effector position
    x_dot_previous=np.zeros((6,1))
    
    # Set model parameters:
    m=1
    K=0.2
    C=15
    B=12
    
    while True:
        '''                
        Following code breaks loop upon user input (enter key)
        '''
        try:
            stdin = sys.stdin.read()
            if "\n" in stdin or "\r" in stdin:
                return_to_init(joints,'left')
                break
        except IOError:
            pass
        
        # Gather Jacobian information:
        J=kin.jacobian()
        J_T=kin.jacobian_transpose()
        J_PI=kin.jacobian_pseudo_inverse()
        J_T_PI=np.linalg.pinv(J_T)
        
        # Extract sensor data:       
        from sensor_msgs.msg import JointState
        from baxter_interface import Limb
        
        right_arm=Limb('right')
        left_arm=Limb('left')
        
        # Information is published at 100Hz by default (every 10ms=.01sec)
        time_step=0.01
        
        
        joints=left_arm.joint_names()
        positions=convert_dict_to_list('left',left_arm.joint_angles())
        velocities=convert_dict_to_list('left',left_arm.joint_velocities())
        force=convert_dict_to_list('left',left_arm.joint_efforts())
                
        force_mag=np.linalg.norm(force)       
        print(force_mag)
        
        if (force_mag<28):                         # Add deadzone
            continue
        else:
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
            
            x=kin.forward_position_kinematics()
            x=x[:-1]
            x_ref=np.reshape(x,[6,1])                           # Reference position
            
            x_ref_dot=J*velocities                              # Reference velocities
            
            t_stop=10
            t_end=time.time()+t_stop                            # Loop timer (in seconds)
            
            # Initial plot parameters
            time_plot_cum=0
            time_vec=[time_plot_cum]
            pos_vec=[x_ref[1][0]]      
            
            # For integral control
            x_ctrl_cum=0
            time_cum=0.00001                        # Prevent divide by zero
            x_ctrl_prev=0                           # Initial for derivative ctrl
            
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
                
                x_ddot=derivative(x_dot_previous,x_dot_current,time_step) 
                
                # Model goes here
                f=J_T_PI*force                      # spacial force
                
                # Only interested in y-axis:
                f[0]=[0]
                #f[1]=[0]
                f[2]=[0]
                f[3]=[0]                       
                f[4]=[0]
                f[5]=[0]
                                
                x_des=((B*f-m*x_ddot-C*(x_dot_current-x_ref_dot))/K)+x_ref          # Spring with damper

                
                # Control robot
                Kp=0.0004
                Ki=0.00000022
                Kd=0.0000005
                
                x_ctrl=x_current-x_des
                
                # Only interested in y-axis:
                x_ctrl[0]=0                                 
                #x_ctrl[1]=0                                
                x_ctrl[2]=0                                 
                x_ctrl[3]=0
                x_ctrl[4]=0
                x_ctrl[5]=0
                
                q_dot_ctrl=J_T*np.linalg.inv(J*J_T+np.identity(6))*(-Kp*x_ctrl
                                                                    -Ki*(x_ctrl_cum)
                                                                    -Kd*(x_ctrl_prev-x_ctrl)/time_step)

                
                q_dot_ctrl_list=convert_mat_to_list(q_dot_ctrl)
                q_dot_ctrl_dict=convert_list_to_dict(joints,q_dot_ctrl_list)
                
                left_arm.set_joint_velocities(q_dot_ctrl_dict)                  # Velocity control function
                
                # Update plot info
                time_cum+=.05                               
                time_vec.append(time_cum)
                
                pos_vec.append(x_current[1][0])
                
                # Update integral control parameters
                x_ctrl_cum+=x_ctrl*time_step
                
                # Update derivative control parameters
                x_ctrl_prev=x_ctrl
                
                # Update values before next loop
                x_previous=x_current                        # Updates previous position for next loop iteration
                x_dot_previous=x_dot_current                # Updates previous velocity for next loop iteration
                
                
            print(time_vec)
            print(pos_vec)
            plt.plot(time_vec,pos_vec)
            plt.title('Position over time')
            plt.xlabel('Time (sec)')
            plt.ylabel('Position')
            plt.show()  
            
            stop_joints(q_dot_ctrl_dict)
            left_arm.set_joint_velocities(q_dot_ctrl_dict)
            time.sleep(1)
            break
            
if __name__ == "__main__":
    main()