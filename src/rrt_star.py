#!/usr/bin/env python
import rospy
import numpy as np
from nav_msgs.srv import *
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose, PoseStamped

class SampleSpace():
	def __init__(self, x_min, x_max, y_min, y_max, z_min, z_max):
		self.x_min = x_min
		self.y_min = y_min
		self.z_min = z_min
		self.x_max = x_max
		self.y_max = y_max
		self.z_max = z_max

class Node():
	def __init__(self, position):
		self.position = position
		self.cost = 0
		self.parent = None

class RRTStar():
	def __init__(self, start_pose, goal_pose, collision_check_function, sample_space, max_step_size, max_iterations, goal_sample_rate, goal_check_rate):
		self.start_node = self.pose_to_node(start_pose)
		self.goal_node = self.pose_to_node(goal_pose)
		self.collision_check = collision_check_function
		self.sample_space = sample_space
		self.max_step_size = max_step_size
		self.max_iterations = max_iterations
		self.goal_sample_rate = goal_sample_rate
		self.goal_check_rate = goal_check_rate

	def node_to_pose(self, n):
		p = Pose()
		p.position.x = n.position[0]
		p.position.y = n.position[1]
		p.position.z = n.position[2]
		p.orientation.w = 1
		return p
	
	def pose_to_node(self, p):
		n = Node(np.array((p.position.x,p.position.y,p.position.z)))
		return n
	
	def generate_random_node(self):
		'''
		Samples a random point from the SampleSpace and returns it as a node. The goal node is returned with goal_sample_rate % of the time
		'''
		if np.random.randint(1,100) > self.goal_sample_rate:
			x = np.random.uniform(self.sample_space.x_min, self.sample_space.x_max)
			y = np.random.uniform(self.sample_space.y_min, self.sample_space.y_max)
			z = np.random.uniform(self.sample_space.z_min, self.sample_space.z_max)
			random_node = Node(np.array((x,y,z)))
		else:
			random_node = self.goal_node
		return random_node
	
	def distance_between(self, n1, n2):
		return np.linalg.norm(n1.position - n2.position)
	
	def get_nearest_node(self, n):
		dlist = [self.distance_between(n, node) for node in self.nodes]
		min_index = dlist.index(min(dlist))
		return self.nodes[min_index]
		
	def steer(self, n1, n2):
		dv = n2.position - n1.position
		dv_mod = np.linalg.norm(dv)
		if dv_mod == 0:
			return n1
		if dv_mod < self.max_step_size:
			return n2
		return Node(n1.position + dv*self.max_step_size/dv_mod)
	
	def find_nearest_nodes(self, n):
		'''
		Returns a list of nodes within r^2 distance of n
		'''
		r = 50.0 * np.sqrt(np.log(len(self.nodes)+1)/ len(self.nodes))
		dlist = [self.distance_between(n, node) for node in self.nodes]
		nearinds = [dlist.index(i) for i in dlist if i <= r**2]
		nodes = [self.nodes[i] for i in nearinds]
		return nodes
	
	def obstacle_free(self, n1, n2):
		'''
		Returns True if the path from n1 to n2 is collision free. The collision free check is performed at intervals of max_step_size
		'''
		dv = n2.position - n1.position
		dv_mod = np.linalg.norm(dv)
		for i in range(int(dv_mod/self.max_step_size)):
			temp_node = Node(n1.position + i*self.max_step_size*dv/dv_mod)
			if self.collision_check(temp_node):
				return False
		return True
	
	def choose_parent(self, n, nodes):
		if len(nodes) == 0:
			return n
		d = []
		for node in nodes:
			if self.obstacle_free(n, node):
				d.append(node.cost + self.distance_between(n, node))
			else:
				d.append(float("inf"))
		min_cost = min(d)
		min_node = nodes[d.index(min_cost)]
		if min_cost == float("inf"):
			print("Minimum cost is inf")
			return n
		n.cost = min_cost
		n.parent = min_node
		return n
	
	def rewire(self, n, nodes):
		for node in nodes:
			d = self.distance_between(n, node)
			scost = n.cost + d
			if node.cost > scost:
				if obstacle_free(n, node):
					node.parent = n
					node.cost = scost
	
	def get_best_last_node(self):
		if self.goal_node in self.nodes:
			return self.goal_node 
		distances_to_goal = [self.distance_between(self.goal_node, node) for node in self.nodes]
		goal_indices = [distances_to_goal.index(i) for i in distances_to_goal if i <= self.max_step_size]
		if len(goal_indices) == 0:
			return None
		
		min_cost = min([self.nodes[i].cost for i in goal_indices])
		for i in goal_indices:
			if self.nodes[i].cost == min_cost:
				return self.nodes[i]
		return None
		
	def generate_path(self, n):
		path = []
		while n is not None:
			path.append(n)
			n = n.parent
		return path
		
	def path_to_poses(self, path):
		poses = []
		for node in path:
			poses.append(self.node_to_pose(node))
		return poses
	
	def generate_response(self, poses):
		res = GetPlanResponse()
		stamped_poses = []
		for i in poses:
			ps = PoseStamped()
			ps.pose = i
			stamped_poses.append(ps)
		res.plan.poses = stamped_poses
		res.plan.header.stamp = rospy.Time.now()
		return res

	def plan(self):
		self.nodes = [self.start_node]
		for i in range(self.max_iterations):
			random_node = self.generate_random_node()
			nearest_node = self.get_nearest_node(random_node)
			new_node = self.steer(nearest_node, random_node)
			
			#print(len(self.nodes))
			if i == self.max_iterations - 10:
				new_node = self.goal_node
				
			if not self.collision_check(new_node):
				nearest_nodes = self.find_nearest_nodes(new_node)
				new_node = self.choose_parent(new_node, nearest_nodes)
				self.nodes.append(new_node)
				self.rewire(new_node, nearest_nodes)
			
			if np.random.randint(1,100) < self.goal_check_rate:
				if self.distance_between(self.get_nearest_node(self.goal_node),self.goal_node) < self.max_step_size:
					break
		last_node = self.get_best_last_node()
		if last_node is None:
			return GetPlanResponse()
		path = self.generate_path(last_node)
		poses = self.path_to_poses(path)
		poses.reverse()
		return self.generate_response(poses)

def collision_check_function(node):
	'''
	This function needs to be implemented as per your environment. Returns True if the node is in collision.
	'''
	# By only returning false, it means that this is in free space.
	return False

def service_cb(msg):
	rospy.loginfo(msg)
	
	############## Change this according to your requirements
	x_min = min(msg.start.pose.position.x, msg.goal.pose.position.x)-1
	y_min = min(msg.start.pose.position.y, msg.goal.pose.position.y)-1
	z_min = min(msg.start.pose.position.z, msg.goal.pose.position.z)-1
	x_max = max(msg.start.pose.position.x, msg.goal.pose.position.x)+1
	y_max = max(msg.start.pose.position.y, msg.goal.pose.position.y)+1
	z_max = max(msg.start.pose.position.z, msg.goal.pose.position.z)+1
	sample_space = SampleSpace(x_min, x_max, y_min, y_max, z_min, z_max)
	##############
	
	rospy.loginfo("Planning")
	planner = RRTStar(msg.start.pose, msg.goal.pose, collision_check_function, sample_space, 0.1, 200, 40, 30)
	response = planner.plan()
	rospy.loginfo(response)
	return response 
	
def main():
	rospy.init_node("rrt_star_planner")
	s = rospy.Service("planner", GetPlan, service_cb)
	print("Server Ready")
	rospy.spin()

if __name__=="__main__":
	try:
		main()
	except rospy.ROSInterruptException:
		pass
