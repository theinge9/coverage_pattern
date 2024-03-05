#! /usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

from libs.list_helper import *

class New_Visualization:
    def __init__(self):
        self.last_point = {}
        self.last_pattern = None
        self.pub_marker = rospy.Publisher("pattern_marker", Marker, queue_size=10)
        self.global_frame = rospy.get_param('~global_frame','map')

    def visualization_cleanup(self):
        for id, points in self.last_point.items():
            if points is not None:
                self.visualization_polygon(points, id=id, show=False)
            self.last_point = {}
        if self.last_pattern is not None:
            self.visualization_pattern(self.last_pattern, show=False)
            self.last_pattern = None
        
    def visualization_polygon(self, points, show=True, close=True, id=0):
        if len(points) < 2: return

        self.last_point[id] = points if show else None

        msg = Marker()
        msg.header.frame_id = self.global_frame
        msg.header.stamp = rospy.Time.now()
        msg.ns = "polygon"
        msg.lifetime = rospy.Duration(0)
        msg.id = id
        msg.type = Marker.LINE_STRIP
        msg.action = Marker.ADD if show else Marker.DELETE
        msg.pose.orientation.x = 0
        msg.pose.orientation.y = 0
        msg.pose.orientation.z = 0
        msg.pose.orientation.w = 1
        msg.scale.x = 0.07
        ## Blue
        msg.color.r = 0.0
        msg.color.g = 0.0
        msg.color.b = 1.0
        msg.color.a = 1.0
        if close:
            points = points + [points[0]]
        for point in points:
            point_msg = Point()
            point_msg.x = point[0]
            point_msg.y = point[1]
            msg.points.append(point_msg)
        self.pub_marker.publish(msg)

    def visualization_pattern(self, pattern, show=True):
        i = 0
        self.last_pattern = pattern if show else None
        for last_pos,cur_pos in pairwise(pattern):
            msg = Marker()
            msg.header.frame_id = self.global_frame
            msg.header.stamp = rospy.Time.now()
            msg.ns = "pattern"
            msg.lifetime = rospy.Duration(0)
            msg.id = i
            msg.type = Marker.ARROW
            msg.action = Marker.ADD if show else Marker.DELETE
            msg.pose.orientation.x = 0
            msg.pose.orientation.y = 0
            msg.pose.orientation.z = 0
            msg.pose.orientation.w = 1
            msg.scale.x = 0.02
            msg.scale.y = 0.04
            ## Green
            msg.color.r = 0.0
            msg.color.g = 1.0
            msg.color.b = 0.0
            msg.color.a = 1.0

            point_msg_start = Point()
            point_msg_start.x = last_pos[0]
            point_msg_start.y = last_pos[1]
            msg.points.append(point_msg_start)
            point_msg_end = Point()
            point_msg_end.x = cur_pos[0]
            point_msg_end.y = cur_pos[1]
            msg.points.append(point_msg_end)

            i+=1
            self.pub_marker.publish(msg)

    def visualization_square_marker(self, pattern, width, length, arrived=False):
        i = 0
        for point in pattern:
            msg = Marker()
            msg.header.frame_id = self.global_frame
            msg.header.stamp = rospy.Time.now()
            msg.ns = "square_areas"
            msg.id = i
            msg.type = Marker.CUBE
            msg.action = Marker.ADD
            msg.pose.position.x = point[0]
            msg.pose.position.y = point[1]
            msg.pose.position.z = 0
            msg.pose.orientation.x = 0
            msg.pose.orientation.y = 0
            msg.pose.orientation.z = 0
            msg.pose.orientation.w = 1
            msg.scale.x = width
            msg.scale.y = length
            msg.scale.z = 0.01  # Small thickness for visualization
            if arrived:
                msg.color.a = 0.8  # Set alpha (transparency)
                msg.color.r = 1.0
                msg.color.g = 0.0
                msg.color.b = 0.0
            else:
                msg.color.a = 0.5  # Set alpha (transparency)
                msg.color.r = 0.0
                msg.color.g = 1.0
                msg.color.b = 0.0

            i+=1
            self.pub_marker.publish(msg)

    def visualization_arrived(self, pose, width, length):
            msg = Marker()
            msg.header.frame_id = self.global_frame
            msg.header.stamp = rospy.Time.now()
            msg.ns = "square_areas"
            msg.id = 0
            msg.type = Marker.CUBE
            msg.action = Marker.ADD
            msg.pose.position.x = pose[0]
            msg.pose.position.y = pose[1]
            msg.pose.position.z = 0
            msg.pose.orientation.x = 0
            msg.pose.orientation.y = 0
            msg.pose.orientation.z = 0
            msg.pose.orientation.w = 1
            msg.scale.x = width
            msg.scale.y = length
            msg.scale.z = 0.01  # Small thickness for visualization
            msg.color.a = 0.8  # Set alpha (transparency)
            msg.color.r = 1.0
            msg.color.g = 0.0
            msg.color.b = 0.0

            self.pub_marker.publish(msg)