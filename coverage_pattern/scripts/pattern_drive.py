#! /usr/bin/env python3
"""
Reference
"""
import time
import numpy as np
from math import *

import rospy
import rospkg
import tf
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped, PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib import SimpleActionClient
from actionlib_msgs.msg import GoalStatus
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetPlan

from shapely.geometry import Polygon

from libs.list_helper import *
from libs.new_visualization import New_Visualization
from libs.boustro import create_path as boustro

INSCRIBED_INFLATED_OBSTACLE = 253

class PatternDrive(New_Visualization):
	def __init__(self):
		rospy.init_node('Pattern_Drive')
		self.rospack = rospkg.RosPack()
		New_Visualization.__init__(self)

		self.clicked_points = []
		self.global_frame = "map" # read from "/clicked_point"
		self.global_costmap = None
		self.local_costmap = None
		self.robot_width = rospy.get_param("~robot_width", 0.5)
		self.costmap_max_non_lethal = rospy.get_param("~costmap_max_non_lethal", 70)
		self.base_frame = rospy.get_param("~base_frame", "base_link")

		rospy.Subscriber("/clicked_point", PointStamped, self.rviz_point_received)
		rospy.Subscriber("/move_base/global_costmap/costmap", OccupancyGrid, self.global_costmap_received)
		rospy.Subscriber("/move_base/local_costmap/costmap", OccupancyGrid, self.local_costmap_received)
		
        ## Tf Listener ##
		self.tfBuffer = tf2_ros.Buffer()
		listener = tf2_ros.TransformListener(self.tfBuffer)
		
        ## Create actionlib ##
		self.move_base = SimpleActionClient("move_base", MoveBaseAction)
		rospy.loginfo("Waiting for move_base action server to come up...")
		self.move_base.wait_for_server()
		self.move_base_plan = rospy.ServiceProxy('/move_base/make_plan', GetPlan)
		rospy.loginfo("Connected to move_base")
		rospy.loginfo("Pattern Node is ready...")
		rospy.on_shutdown(self.on_shutdown)
		rospy.loginfo("Use PublishPoint Tool in Rviz to select Area")

	def global_costmap_received(self, costmap):
		self.global_costmap = costmap

	def local_costmap_received(self, costmap):
		self.local_costmap = costmap
		self.local_costmap_width = costmap.info.width*costmap.info.resolution
		self.local_costmap_height = costmap.info.height*costmap.info.resolution

	def rviz_point_received(self, received_points):
		if self.global_costmap is None or self.local_costmap is None:
			rospy.logerr("No global or local costmap, doing nothing.")
			return
		
		self.clicked_points.append(received_points)
		points = [(p.point.x, p.point.y) for p in self.clicked_points]
		self.global_frame = received_points.header.frame_id
		
		if len(self.clicked_points) > 2:
			## All points must have same frame_id
			if len(set([p.header.frame_id for p in self.clicked_points])) != 1:
				raise ValueError()
			
			points_x = [p.point.x for p in self.clicked_points]
			points_y = [p.point.y for p in self.clicked_points]
			avg_x_dist = list_avg_dist(points_x)
			avg_y_dist = list_avg_dist(points_y)
			dist_x_first_last = abs(points_x[0] - points_x[-1])
			dist_y_first_last = abs(points_y[0] - points_y[-1])
			if dist_x_first_last < avg_x_dist/10.0 and dist_y_first_last < avg_y_dist/10.0:
				## Last point is close to maximum, construct polygon
				rospy.loginfo("Creating polygon %s" % (str(points)))
				self.visualization_polygon(points, close=True)
				pattern = self.create_pattern(Polygon(points))
				self.drive_pattern(pattern)
				self.clicked_points = []
				return
		self.visualization_polygon(points, close=False)
		
	def create_pattern(self, polygon):
		pattern = boustro(polygon, self.robot_width)
		self.visualization_pattern(pattern)
		return pattern
		
	def move(self, x, y, angle):
		rospy.loginfo("Moving to (%f, %f, %.0f)" % (x, y, angle*180/pi))

		goal = MoveBaseGoal()
		angle_quat = tf.transformations.quaternion_from_euler(0, 0, angle)
		goal.target_pose.header.frame_id = self.global_frame
		goal.target_pose.header.stamp = rospy.Time.now()
		goal.target_pose.pose.position.x = x
		goal.target_pose.pose.position.y = y
		goal.target_pose.pose.orientation.x = angle_quat[0]
		goal.target_pose.pose.orientation.y = angle_quat[1]
		goal.target_pose.pose.orientation.z = angle_quat[2]
		goal.target_pose.pose.orientation.w = angle_quat[3]
		self.move_base.send_goal(goal)

		self.move_base.wait_for_result()
		if self.move_base.get_state() == GoalStatus.SUCCEEDED:
			rospy.loginfo("The base moved to (%f, %f)" % (x, y))
		else:
			rospy.logerr("The base failed moving to (%f, %f)" % (x, y))

	def get_closes_possible_goal(self, pos_last, pos_next, angle, tolerance):
		angle_quat = tf.transformations.quaternion_from_euler(0, 0, angle)
		start = PoseStamped()
		start.header.frame_id = self.global_frame
		start.pose.position.x = pos_last[0]
		start.pose.position.y = pos_last[1]
		start.pose.orientation.x = angle_quat[0]
		start.pose.orientation.y = angle_quat[1]
		start.pose.orientation.z = angle_quat[2]
		start.pose.orientation.w = angle_quat[3]
		goal = PoseStamped()
		goal.header.frame_id = self.global_frame
		goal.pose.position.x = pos_next[0]
		goal.pose.position.y = pos_next[1]
		goal.pose.orientation.x = angle_quat[0]
		goal.pose.orientation.y = angle_quat[1]
		goal.pose.orientation.z = angle_quat[2]
		goal.pose.orientation.w = angle_quat[3]
		plan = self.move_base_plan(start, goal, tolerance).plan
		if len(plan.poses) == 0:
			return None
		#pdb.set_trace()
		closest = None
		for pose in plan.poses:
			pose.header.stamp = rospy.Time(0) # time for lookup does not need to be exact since we are stopped

			local_pose = self.tfBuffer.transform(pose, self.local_costmap.header.frame_id)

			cellx = round((local_pose.pose.position.x-self.local_costmap.info.origin.position.x)/self.local_costmap.info.resolution)
			celly = round((local_pose.pose.position.y-self.local_costmap.info.origin.position.y)/self.local_costmap.info.resolution)
			cellidx = int(celly*self.local_costmap.info.width+cellx)
			if cellidx < 0 or cellidx >= len(self.local_costmap.data):
				rospy.logwarn("get_closes_possible_goal landed outside costmap, returning original goal.")
				return pos_next
			cost = self.local_costmap.data[cellidx]

			if (cost >= INSCRIBED_INFLATED_OBSTACLE):
				break

			closest = pose
		return (closest.pose.position.x, closest.pose.position.y)

	def drive_pattern(self, pattern):
		total_pos = len(pattern)
		pos_counts = 0
		
		initial_pos = self.tfBuffer.lookup_transform(self.global_frame, self.base_frame, rospy.Time(0), rospy.Duration(0.0))
		pattern.insert(0, (initial_pos.transform.translation.x, initial_pos.transform.translation.y))
		
		for pos_last,pos_next in pairwise(pattern):
			if rospy.is_shutdown(): return

			pos_diff = np.array(pos_next)-np.array(pos_last)
			## Angle from last to current position
			angle = atan2(pos_diff[1], pos_diff[0])

			if abs(pos_diff[0]) < self.local_costmap_width/2.0 and abs(pos_diff[1]) < self.local_costmap_height/2.0:
				# goal is visible in local costmap, check path is clear
				tolerance = min(pos_diff[0], pos_diff[1])
				closest = self.get_closes_possible_goal(pos_last, pos_next, angle, tolerance)
				if closest is None:
					continue
				pos_next = closest

			self.move(pos_last[0], pos_last[1], angle) ## Rotate in direction of next goal
			time.sleep(1)
			self.move(pos_next[0], pos_next[1], angle)
			
            ## Calculate Percentage ##
			pos_counts += 1
			percentage = (pos_counts) / total_pos * 100
			rospy.loginfo(f"Travel Percentage: {percentage:.2f}%")
			
		self.visualization_pattern(pattern, show=False)
		rospy.loginfo("Pattern Drive Completed.")

	def on_shutdown(self):
		rospy.loginfo("Canceling all goals")
		self.visualization_cleanup()
		self.move_base.cancel_all_goals()

if __name__ == "__main__":
	p = PatternDrive()
	rospy.spin()