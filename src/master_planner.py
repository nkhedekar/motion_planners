#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point, Pose, PoseStamped
from nav_msgs.srv import *
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray

pose_publisher = rospy.Publisher("command/pose", Pose, queue_size=10)
marker_publisher = rospy.Publisher("trajectory", MarkerArray, queue_size=10, latch=True)

def pose_stamped_to_point(pose_stamped):
	p = Point()
	p.x = pose_stamped.pose.position.x
	p.y = pose_stamped.pose.position.y
	p.z = pose_stamped.pose.position.z
	return p

def publish(res):
	l = res.plan.poses
	rate = rospy.Rate(0.5)
	ma = MarkerArray()
	for i in range(len(l)-1):
		m = Marker()
		m.header.frame_id = "world"
		m.ns = "robot"
		m.id = i
		m.type = 0
		m.action = 0
		p1 = pose_stamped_to_point(l[i])
		p2 = pose_stamped_to_point(l[i+1])
		m.points.append(p1)
		m.points.append(p2)
		m.scale.x = 0.01
		m.scale.y = 0.03
		m.color.r = 1
		m.color.a = 1
		ma.markers.append(m)
		marker_publisher.publish(ma)
		pose_publisher.publish(l[i].pose)
		rate.sleep()
	pose_publisher.publish(l[i+1].pose)
		
def get_points():
	p1 = PoseStamped()
	print("start location")
	p1.pose.position.x =  float(input("x1:"))
	p1.pose.position.y = float(input("y1:"))
	p1.pose.position.z = float(input("z1:"))
	p2 = PoseStamped()
	print("goal location")
	p2.pose.position.x = float(input("x2:"))
	p2.pose.position.y = float(input("y2:"))
	p2.pose.position.z = float(input("z2:"))
	return (p1, p2)
	
def main():
	rospy.init_node("master_planner", anonymous=True)
	try:
		planner = rospy.ServiceProxy('planner', GetPlan)
		while True:
			start, goal = get_points()
			res = planner(start, goal, 0.0)
			publish(res)
	except rospy.ServiceException, e:
		print("Service call failed: %s" %e)
		
if __name__=="__main__":
	try:
		main()
	except rospy.ROSInterruptException:
		pass
