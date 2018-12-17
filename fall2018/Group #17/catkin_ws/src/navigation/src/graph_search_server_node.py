#!/usr/bin/env python

import rospy, sys, os, cv2, pickle
from navigation.graph import Graph
from navigation.graph_search import GraphSearchProblem
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from navigation.srv import *
from navigation.generate_duckietown_map import graph_creator
import numpy as np
from duckietown_msgs.msg import BoolStamped, SourceTargetNodes
from std_msgs.msg import Int16

class graph_search_server():
    def __init__(self):
        rospy.loginfo('Graph Search Service Started')

        # Input: csv file
        self.map_name = rospy.get_param('/map_name')

        # Loading map
        self.script_dir = os.path.dirname(__file__)
        self.map_path = self.script_dir + '/maps/' + self.map_name
        self.map_img = self.script_dir + '/maps/map.png'

        gc = graph_creator()
        self.duckietown_graph = gc.build_graph_from_csv(csv_filename=self.map_name)
        self.duckietown_problem = GraphSearchProblem(self.duckietown_graph, None, None)             
        print "Map loaded successfully!\n"

	self.current_node = 0
	
        #Subscribers
        
        #road block detected
        self.sub_rb = rospy.Subscriber('~road_blocked', BoolStamped, self.handle_road_blocked)
        #duckie is turning; propagate now
        self.sub_turn = rospy.Subscriber("~turn_type", Int16, self.propagate_location)        
        
	#Publishers
	#publish to here to request an a* search from ActionsDispatcherNode
	self.pub_plan_request = rospy.Publisher("~plan_request", SourceTargetNodes, queue_size = 1)	
	
        self.image_pub = rospy.Publisher("~map_graph",Image, queue_size = 1, latch=True)
        self.bridge = CvBridge()

        # Send graph through publisher
        self.duckietown_graph.draw(self.script_dir, highlight_edges=None, map_name = self.map_name)
        cv_image = cv2.imread(self.map_path + '.png', cv2.CV_LOAD_IMAGE_COLOR)
        overlay = self.prepImage(cv_image)
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(overlay, "bgr8"))
    
    def initCurrentNode(self,currentNode):
	self.current_node = currentNode
	#if current node isn't at an intersection, have to do some propagation
	#what happens with a dead end? future work...
	if self.current_node[0:3] == 'turn': #ensure that last_int_node is actually at an intersection
	    current_edge_set = self.duckietown_problem.graph._edges[last_node]
	    for i in [1, 2]:
		#traversing forward to next intersection
		while len(current_edge_set) == 1: #haven't reached an intersection yet
		    # find edge(s) after the one we're on
		    current_edge_set = self.duckietown_problem.graph._edges[next(iter(current_edge_set)).target]
		    last_node = next(iter(current_edge_set)).source
		    
    
		#reached the next intersection
		u_turn_lut = get_flip_lut()
		last_node = u_turn_lut[next(iter(current_edge_set)).source]
    
		#repeat the process going in the anti-parallel lane
		current_edge_set = self.duckietown_problem.graph._edges[last_node]
    
	    self.current_node = last_node            
       

    def setupParameter(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def handle_graph_search(self,req):
        
        # Checking if nodes exists
        rospy.loginfo(list( self.duckietown_graph._nodes)) #.nodes() ?
        if (req.source_node not in self.duckietown_graph) or (req.target_node not in self.duckietown_graph):
            rospy.loginfo("Source or target node do not exist.")
            self.publishImage(req, [])
            return GraphSearchResponse([])

	#set current node to be requested start node
	self.initCurrentNode(req.source_node)	

        # Running A*
        self.duckietown_problem.start = req.source_node
        self.duckietown_problem.goal = req.target_node
        path = self.duckietown_problem.astar_search()

        # Publish graph solution
        self.publishImage(req, path)
	
        return GraphSearchResponse(path.actions)        

    def publishImage(self, req, path):
        if path:
            self.duckietown_graph.draw(self.script_dir, highlight_edges=path.edges(), map_name = self.map_name, highlight_nodes = [req.source_node, req.target_node])
        else:
            self.duckietown_graph.draw(self.script_dir, highlight_edges=None, map_name = self.map_name)
        cv_image = cv2.imread(self.map_path + '.png', cv2.CV_LOAD_IMAGE_COLOR)
        overlay = self.prepImage(cv_image)
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(overlay, "bgr8"))

    def prepImage(self, cv_image):
        map_img = cv2.imread(self.map_img, cv2.CV_LOAD_IMAGE_COLOR)
        map_crop = map_img[16:556,29:408,:]
        map_resize = cv2.resize(map_crop,(cv_image.shape[1],955),interpolation=cv2.INTER_AREA)
        cv_image = cv_image[0:955,:,:]
        cv_image = 255 - cv_image
	overlay = cv_image                                                                                                                                                                            
        #overlay = cv2.addWeighted(cv_image,0.65,map_resize,0.35,0)                                                                                                                                             
        overlay = cv2.resize(overlay,(0,0),fx=0.9,fy=0.9,interpolation=cv2.INTER_AREA) 
        overlay *= 1.4
        return overlay

    # path observing and map modifying methods
    def handle_road_blocked(self,data):
	# runs when a complete roadblock is seen
	# modify working graph, then plan a new path
	if data.data == True:
	    self.duckietown_problem.graph = self.destroy_edges()
	    self.pub_plan_request.publish(self.current_node,self.duckietown_problem.goal)
	
    
    def propagate_location(self,data):
	# run each time a turn command is issued; follow along in the graph
	data = data.data
	possible_edges = self.duckietown_problem.graph._edges[self.current_node]
	actionLUT = {1:'s', 2:'r', 0:'l', -1:'w'}
#	actionLUT = {"1":'s', "2":'r', "0":'l', "-1":'w'}
#	print("actionLUT: ", actionLUT)
#	print("data: ", data)
#	print("actionLUT: ", actionLUT[data])
	#follow the straight roads
	while len(possible_edges)==1:
	    edge = possible_edges.pop()
	    possible_edges = self.duckietown_problem.graph._edges[edge.target]
	
	#found an intersection
	for edge in possible_edges:
            #print("edge.action: ", edge.action)
	    if edge.action == actionLUT[data]:            
		self.current_node = edge.target
		break
    
	rospy.loginfo('reached node %s',self.current_node)
    
    def destroy_edges(self):
	# this is to be called when the current edge is a completely blocked road
	# traverse edges forward until the next numbered node while deleting
	# use flip LUT to delete the lane anti-parallel to the path just travelled
    
	# Doesn't work for blockages in intersections (yet...)
    
    
	# make a working copy of the graph        
	#w_graph = Graph(orig_graph.node_label_fn,orig_graph._nodes,orig_graph._edges,orig_graph.node_positions)
	w_graph = self.duckietown_problem.graph.copy()
        
	current_edge_set = w_graph._edges[self.current_node]
	
	for i in [1, 2]: #delete this lane and the anti-parallel lane
	    #traversing forward to next intersection
	    while len(current_edge_set) == 1: #haven't reached an intersection yet
		# delete the current edge
		w_graph._edges[self.current_node] = w_graph._edges[self.current_node] - current_edge_set
		# find edge(s) after the one we're on
		current_edge_set = w_graph._edges[next(iter(current_edge_set)).target]
		self.current_node = next(iter(current_edge_set)).source
    
	    if i == 2: break
	    #reached the next intersection
	    u_turn_lut = self.get_flip_lut()
	    self.current_node = u_turn_lut[next(iter(current_edge_set)).source]
    
	    current_edge_set = w_graph._edges[self.current_node]
    
	return w_graph
    
    def get_flip_lut(self):
	# look up table for u-turn node results. For every node at an intersection, this contains the node in the opposite lane closest to the current node
	# In the future, this LUT could be autogenerated from the data in the map csv. Once that data is used to generate a graph, it's harder to use that data to generate the lut
    
	# keys: current node
	# entries: node in other direction
	# no particular order to these
	u_turn_lut = {'1':'6',
                      '2':'3',
                      '4':'5',
                      '7':'12',
                      '8':'9',
                      '10':'11',
                      '13':'20',
                      '14':'15',
                      '16':'17',
                      '18':'19',
                      '21':'26',
                      '22':'23',
                      '24':'25',
                      '32':'27',
                      '28':'29',
                      '30':'31',
                      }

	# autogenerate the LUT that reverses all these key, entry pairs
	second_half_u_turn_lut = {}
	for key in u_turn_lut.keys():
	    second_half_u_turn_lut[u_turn_lut[key]] = key

	#print len(u_turn_lut.keys())

	# add the autogenerated content to the original
	u_turn_lut.update(second_half_u_turn_lut)
	#print len(u_turn_lut.keys())
	return u_turn_lut         

if __name__ == "__main__":	
    rospy.init_node('graph_search_server_node')
    gss = graph_search_server()
    print 'Starting server...\n'
    s = rospy.Service('graph_search', GraphSearch, gss.handle_graph_search)
    rospy.spin()
