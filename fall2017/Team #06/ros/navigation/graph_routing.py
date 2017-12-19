#!/usr/bin/env python


"""
Author: Zac Ravichandran
Module to find paths through Duckietown
"""

import heapq
import numpy as np

"""
Package to create car insructions 
"""

class DuckietownEdge():
	def __init__(self, from_node, to_node, cost):
		self.from_node = from_node
		self.to_node = to_node 
		self.cost = cost

class DuckietownNode():
	"""
	All directions oriented facing door to the lab
	"""
	def __init__(self, node_id, n, s, e, w):
		self.node_id = node_id
		self.edges = {"n": DuckietownEdge(node_id, n[0], n[1]),
		 "s":DuckietownEdge(node_id, s[0], s[1]), 
		 "e":DuckietownEdge(node_id, e[0], e[1]),
		 "w":DuckietownEdge(node_id, w[0], w[1])}

class DuckietownGraph():
	def __init__(self, nodes = {}):
		self.nodes = nodes 

	def add_node(self, node):
		self.nodes[node.node_id] = node

	def trim_graph(self):
		for node in self.nodes.values():
			for edge_key in node.edges.keys():
				itr_vals = node.edges.keys()
				itr_vals.remove(edge_key)
				for comp_edge_key in itr_vals:
					if node.edges[edge_key].to_node == node.edges[comp_edge_key].to_node:
						if node.edges[edge_key].cost < node.edges[comp_edge_key].cost:
							node.edges[comp_edge_key].to_node = -1


	def find_path(self, start_node_id, start_edge, end_node):
		self.trim_graph()
		start_node = self.nodes[start_node_id]
		costs = {0:np.Inf, 1:np.Inf, 2:np.Inf, 3:np.Inf, 4:np.Inf}
		back_node = {0:None, 1:None, 2:None, 3:None, 4:None}
		queue = []

		costs[start_node_id] = 0
		back_node[start_node_id] = -1

		for key in start_node.edges.keys():
			if key is not start_edge and start_node.edges[key].to_node is not -1:
				costs[start_node.edges[key].to_node] = start_node.edges[key].cost
				back_node[start_node.edges[key].to_node] = start_node.node_id
				heapq.heappush(queue, (start_node.edges[key].cost, start_node.edges[key].to_node))

		
		while len(queue) > 0:
			t = heapq.heappop(queue)
			cost = t[0]
			node = self.nodes[t[1]]
		
			if node.node_id == end_node:
				break

			for key in node.edges.keys():
				if node.edges[key].to_node is not -1:
					if cost + node.edges[key].cost < costs[node.edges[key].to_node]:
						back_node[node.edges[key].to_node] = node.edges[key].from_node
						costs[node.edges[key].to_node] = cost + node.edges[key].cost
						heapq.heappush(queue, (cost + node.edges[key].cost, node.edges[key].to_node))

		path = [end_node]

		while True:
			if back_node[path[-1]] == -1:
				break
			else:
				path.append(back_node[path[-1]])

		path.reverse()
		return self.get_turns(path, start_edge)

	def egde_to_turn(self, se, ee):
		"""
		return 0 - left
				1 - right
				2 - stright
		"""
		if se == "s":
			if ee=="e":
				return 1
			elif ee=="n":
				return 2
			elif ee=="w":
				return 0
		elif se=="w":
			if ee=="e":
				return 2
			elif ee=="n":
				return 0
			elif ee=="s":
				return 1
		elif se=="n":
			if ee=="e":
				return 0
			elif ee=="s":
				return 2
			elif ee=="w":
				return 1
		elif se=="e":
			if ee=="n":
				return 1
			elif ee=="s":
				return 0
			elif ee=="w":
				return 2

	def get_edge_from_next_node_reference(self, first_node, last_node):
		for key in self.nodes[first_node].edges.keys():
			if self.nodes[first_node].edges[key].to_node == last_node:
				return key

	def get_turns(self, path, start_edge):
		last_edge = start_edge 
		last_node = path[0]
		turns = []
		for next_node in path[1:]:
			for key in self.nodes[last_node].edges.keys():
				if self.nodes[last_node].edges[key].to_node == next_node:
					turn_edge = key
					turns.append(self.egde_to_turn(last_edge, turn_edge))
					last_edge = self.get_edge_from_next_node_reference(next_node, last_node) 
					last_node = next_node
		return turns, last_edge

def get_duckietown_path(start_node, start_edge, end_node):
	graph = DuckietownGraph()
	graph.add_node( DuckietownNode(0, (2,3.5), (4,1), (-1, -1), (1, 1)) )
	# long edge
	long_edge_length = 7
	graph.add_node( DuckietownNode(1, (2, 2), (3, long_edge_length), (0, 1), (3,1)) )
	graph.add_node( DuckietownNode(2, (-1,-1), (1, 2), (0, 3.5), (3, 3.5)) )

	# long edge
	graph.add_node( DuckietownNode(3, (2,3.5), (1, long_edge_length), (1,1), (-1,-1)) )

	# garage node
	graph.add_node( DuckietownNode(4, (0,1), (-1,-1), (-1,-1), (-1,-1)) )

	return graph.find_path(start_node, start_edge, end_node)
	
def get_duckietown_route(nodes, start_edge, end_command = []):
	path = []
	current_edge = start_edge
	for i in range(1, len(nodes)):
		path_segment, end_edge = get_duckietown_path(nodes[i-1], current_edge, nodes[i])
		path.extend(path_segment)
		current_edge = end_edge

	if len(end_command) > 0:
		path.extend(end_command)

	return path, current_edge

def main():
	print(get_duckietown_route([0,3,1,4], 's'))


if __name__ == "__main__":
	main()