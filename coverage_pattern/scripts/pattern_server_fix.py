#!/usr/bin/env python3
"""
Pattern Service Server, predefined area
"""
import time
import numpy as np
from math import *

import rospy
import tf
import tf2_ros
from geometry_msgs.msg import PointStamped, PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib import SimpleActionClient
from actionlib_msgs.msg import GoalStatus
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetPlan

from shapely.geometry import Polygon

from libs.list_helper import *
from libs.new_visualization import New_Visualization 
from libs.boustro_grid import create_grid as boustro

from coverage_pattern.srv import pattern, patternResponse

INSCRIBED_INFLATED_OBSTACLE = 253

class PatternDrive(New_Visualization):
    def __init__(self):
        rospy.init_node('Pattern_Server')
        New_Visualization.__init__(self)

        self.clicked_points = []
        self.global_frame = "map"
        self.local_costmap = None
        self.global_costmap = None

        self.robot_width = rospy.get_param("~robot_width", 0.5)
        self.robot_length = rospy.get_param("~robot_length", 0.5)
        self.base_frame = rospy.get_param("~base_frame", "base_link")

        rospy.Subscriber("/move_base/global_costmap/costmap", OccupancyGrid, self.global_costmap_received)
        rospy.Subscriber("/move_base/local_costmap/costmap", OccupancyGrid, self.local_costmap_received)

        ## Service to trigger coverage path planning ##
        self.pattern_service = rospy.Service('/pattern_movement', pattern, self.pattern_callback)

        # Tf Listener ##
        self.tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(self.tfBuffer)

        # Create the actionlib service
        self.move_base = SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for the move_base action server to come up...")
        self.move_base.wait_for_server()
        self.move_base_plan = rospy.ServiceProxy('/move_base/make_plan', GetPlan)
        rospy.loginfo("Connected to move_base")
        rospy.on_shutdown(self.on_shutdown)
        rospy.loginfo("Pattern Server is ready...")

    def global_costmap_received(self, costmap):
        #rospy.loginfo("Global Costmap Received")
        self.global_costmap = costmap
    
    def local_costmap_received(self, costmap):
        #rospy.loginfo("Local Costmap Received")
        self.local_costmap = costmap
        self.local_costmap_width = costmap.info.width*costmap.info.resolution
        self.local_costmap_height = costmap.info.height*costmap.info.resolution

    def pattern_callback(self, request):
        #rospy.loginfo("Service 'pattern_movement' is called.")
        if self.global_costmap is None or self.local_costmap is None:
            rospy.logerr("No Global or Local Costmap")
            return
        
        self.clicked_points.append(request.points)
        points = [(p.point.x, p.point.y) for sublist in self.clicked_points for p in sublist]
        
        ## Construct polygon ##
        rospy.loginfo("Creating polygon %s" % (str(points)))
        self.visualization_polygon(points, close=True)
        pattern = self.create_pattern(Polygon(points))
        self.drive_pattern(pattern)
        self.clicked_points = []
        return patternResponse(result = "Pattern Running Completed.")

    def create_pattern(self, polygon):
        pattern = boustro(polygon, self.robot_width, self.robot_length)
        self.visualization_pattern(pattern)
        return pattern
    
    def move(self, x, y, angle):
        rospy.loginfo("Moving to (%f, %f, %f)" % (x, y, angle*180/pi))

        goal = MoveBaseGoal()
        angle_quat = tf.transformations.quaternion_from_euler(0, 0, angle)
        goal.target_pose.header.frame_id = self.global_frame
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x 
        goal.target_pose.pose.position.y = y 
        goal.target_pose.pose.position.z = 0
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

    def drive_pattern(self, pattern):
        total_pos = len(pattern)
        pos_counts = 0

        initial_pos = self.tfBuffer.lookup_transform(self.global_frame, self.base_frame, rospy.Time(0), rospy.Duration(0.0))
        pattern.insert(0, (initial_pos.transform.translation.x, initial_pos.transform.translation.y))
        
        for pos_last,pos_next in pairwise(pattern):
            if rospy.is_shutdown(): return

            pos_diff = np.array(pos_next) - np.array(pos_last)
            ## Angle from last to current position
            angle = atan2(pos_diff[1], pos_diff[0])

            self.move(pos_last[0], pos_last[1], angle) ## Rotate in direction of next goal
            time.sleep(1.0)
            self.move(pos_next[0], pos_next[1], angle)
            time.sleep(1.0)
            
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