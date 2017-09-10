# Demonstrate path planning with the roadmap
import numpy as np
import time
import matplotlib.pyplot as plt

import Simulation
import Roadmap
import SerialInterface

table = np.array([[[-245,-345,-100],[-245,345,-100],[345,345,-100]], \
    [[-245,-345,-100],[345,-345,-100],[345,345,-100]]]) # table
# obstacle = np.array([[[230.0,-10,0],[230,10,0],[240,0,200]], \
#     [[230,-10,0],[250,-10,0],[240,0,200]], \
#     [[230,10,0],[250,10,0],[240,0,200]], \
#     [[250,-10,0],[250,10,0],[240,0,200]]]) # poll
obstacle = np.array([[[215,-250,-100],[215,250,-100],[215,-250,15]], \
    [[215,250,-100],[215,250,15],[215,-250,15]]]) # wall
sim = Simulation.Simulation()
sim.add_obstacles(table)
sim.add_obstacles(obstacle)
prm = Roadmap.Roadmap(sim,200,np.array([[-90,90],[0,60],[0,60]]))
# path = prm.get_path((45,60,0),(-45,60,0))
path = prm.get_path((0,10,10),(0,0,55))
# prm.plot_path(path)
# sim.display(path[0])

interface = SerialInterface.SerialInterface(5)
time.sleep(2)
interface.send_absolute_angles(path[0][0],path[0][1],path[0][2],0)

sim.display(path[0])
fig = plt.gcf()
fig.canvas.draw()
plt.pause(0.1)

# execute path
time.sleep(2)
for p in path:
    interface.send_absolute_angles(p[0],p[1],p[2],0)
    plt.clf()
    sim.display(p)
    fig.canvas.draw()
    plt.pause(0.1)
time.sleep(2)
path = prm.get_path((0,0,55),(0,10,10))
for p in path:
    interface.send_absolute_angles(p[0],p[1],p[2],0)
    plt.clf()
    sim.display(p)
    fig.canvas.draw()
    plt.pause(0.1)
    