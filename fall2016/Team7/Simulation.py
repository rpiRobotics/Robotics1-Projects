# Model of the dobot arm to be used for collision detection
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import intersect
import DobotModel

class Simulation:
    obstacles = np.array([])

    def __init__(self):
        plt.ion()

    def add_obstacles(self,obs):
        """
        Add a triangle mesh to the simulation.
        """
        if (obs.shape[1] != 3) or (obs.shape[2] != 3):
            print "Triangle mesh must be of shape (n,3,3)"
        elif (len(self.obstacles) == 0):
            self.obstacles = obs
        else:
            self.obstacles = np.vstack((self.obstacles,obs))

    def collision(self,angles):
        """
        Returns whether the Dobot intersects any obstacles in a given configuration.
        """
        arm = DobotModel.get_mesh(angles)

        # Check for collisions
        for Ta in arm:
            for To in self.obstacles:
                if intersect.triangles(Ta,To):
                    return True
        return False

    def display(self,angles):
        """
        Plots wireframe models of the Dobot and obstacles.
        """
        arm = DobotModel.get_mesh(angles)

        #fig = plt.figure()
        fig = plt.gcf()
        ax = Axes3D(fig)
        #plt.axis('equal')
        for Ta in arm:
            ax.plot(Ta[[0,1,2,0],0],Ta[[0,1,2,0],1],Ta[[0,1,2,0],2],'b')
        for To in self.obstacles:
            ax.plot(To[[0,1,2,0],0],To[[0,1,2,0],1],To[[0,1,2,0],2],'b')
        
        r_max = DobotModel.l1 + DobotModel.l2 + DobotModel.d

        plt.xlim([-np.ceil(r_max/np.sqrt(2)),r_max])
        plt.ylim([-r_max,r_max])
        ax.set_zlim(-150, 250)
        ax.view_init(elev=30.0, azim=60.0)
        plt.show()
        return fig
