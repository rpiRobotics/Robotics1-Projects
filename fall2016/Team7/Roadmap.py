# Probabilistic RoadMap path planning for the dobot
import numpy as np
import numpy.random as rng
import scipy.spatial.kdtree as kdt
import networkx as nx

import Simulation
import DobotModel

class Roadmap:
    """
    Probabalistic RoadMap

    Input: sim - Simulation object to use for collision detection
           n - number of desired samples (in free space)
           bounds - array of lower and upper bounds for sampling the workspace (3,2)
    """

    #G - graph of configurations sampled from Qfree
    #tree - KD-tree of positions sampled from workspace 

    def __init__(self,sim,n,bounds=DobotModel.limits):
        self.sim = sim
        self.generate(n,bounds)

    def generate(self,n,bounds=DobotModel.limits):
        """
        Generates (or regenerates) the PRM given a target number of samples n 
        """
        self.G = nx.Graph()

        # Sample environment
        ps = self._sample_cs(n,bounds)
        # ps = self._sample_ws(n,np.array([[0,300],[-200,200],[0,200]]))

        self.tree = kdt.KDTree(ps)

        # Connect samples
        for k in xrange(self.tree.n):
            self._connect(k,self.tree.data[k])

        # Skipping enhancement stage based on the assumption that Qfree >> Q!free

    def get_path(self,q0,qf):
        """
        Searches for a path in the PRM between configurations q0 and qf.
        Returns a list of configuration tuples describing the path or []
        if no path is found.
        """
        q0 = np.reshape(np.array(q0),3)
        qf = np.reshape(np.array(qf),3)
        if all(q0 == qf):
            return [qf]

        n0 = len(self.G.node)
        nf = n0 + 1

        # Add the start and end configs to G, so we can just search it
        self.G.add_node(n0,cfg=q0)
        self.G.add_node(nf,cfg=qf)
        for k in [n0,nf]:
            self._connect(k,DobotModel.forward_kinematics(self.G.node[k]['cfg']))

        if not nx.has_path(self.G,n0,nf):
            path = [] # could not find a path
        else:
            nodes = nx.dijkstra_path(self.G,n0,nf,'weight')
            path = [self.G.node[k]['cfg'] for k in nodes]

        # Remove the start and end configs so G remains consistent with tree
        self.G.remove_node(n0)
        self.G.remove_node(nf)

        return path

    def plot_path(self,path):
        """
        Plots points for all sampled positions of the end effector and draws lines
        along the path described by the configurations "path".
        """
        if (len(path) > 0):
            ps = self.tree.data

            fig = self.sim.display(path[0])
            path = np.array([DobotModel.forward_kinematics(q) for q in path])
            fig.axes[0].plot(path[:,0],path[:,1],path[:,2],'r')
            fig.axes[0].plot(ps[:,0],ps[:,1],ps[:,2],'.g')
            fig.canvas.draw()

    def display(self):
        """
        Display the nodes and edges of the PRM in Cartesian coordinates.
        """
        ps = self.tree.data
        fig = self.sim.display((0,0,0))
        ax = fig.axes[0]
        for (pair) in self.G.edges():
            if (pair[0] < pair[1]): # bidirectional graph so don't plot edges twice
                ax.plot(ps[pair,0],ps[pair,1],ps[pair,2],'r')
        ax.plot(ps[:,0],ps[:,1],ps[:,2],'.g')
        fig.canvas.draw()

    def _connect(self,k,p,knn=5):
        # Attempt to connect a node k (position p) with its k nearest neighbors (knn)
        (dists,inds) = self.tree.query(p,knn)
        for (d,i) in zip(dists,inds):
            if (k != i) and self._path_exists(k,i):
                self.G.add_edge(k,i,weight=d)

    def _path_exists(self,k0,k1):
        # Test whether there is a direct path between nodes k0 and k1 (no collisions)
        q0 = np.array(self.G.node[k0]['cfg'])
        q1 = np.array(self.G.node[k1]['cfg'])

        # Sample the path at ~10mm intervals based on dp/dq at the midpoint
#       J = DobotModel.jacobian(tuple((q0+q1)/2.0))
#       dp = np.dot(J,np.reshape(q1-q0)
#       n = int(np.ceil(np.sqrt(np.dot(dp,np.transpose(dp)))/10.0))
        n = 5

        # Interpolate intermediate configurations
        qk = zip(np.linspace(q0[0],q1[0],n),np.linspace(q0[1],q1[1],n), \
            np.linspace(q0[2],q1[2]))

        # Test for collisions for each
        for q in qk:
            if self.sim.collision(q):
                return False

        return True

    def _sample_ws(self,n,bnds):
        # Sample the workspace until n valid points are found within the bounds
        ps = np.zeros((n,3))
        k = 0
        while (k < n):
            p = rng.rand(bnds.shape[0])*(bnds[:,1] - bnds[:,0]) + bnds[:,0]
            q = DobotModel.inverse_kinematics(p)
            if not any(np.isnan(q)) and not self.sim.collision(q):
                self.G.add_node(k,cfg=q)
                ps[k] = p
                k+=1
        return ps

    def _sample_cs(self,n,bnds):
        # Sample the configuration space until n valid points are found within the bounds
        ps = np.zeros((n,3))
        k = 0
        while (k < n):
            q = tuple(rng.rand(bnds.shape[0])*(bnds[:,1] - bnds[:,0]) + bnds[:,0])
            if DobotModel.valid_angles(q) and not self.sim.collision(q):
                self.G.add_node(k,cfg=q)
                ps[k] = DobotModel.forward_kinematics(q)
                k+=1
        return ps

