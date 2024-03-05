#!/usr/bin/env python3
"""
Room loop in predefined area
"""
import sys

import rospy
from geometry_msgs.msg import PointStamped, Point
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib import SimpleActionClient
from coverage_pattern.srv import pattern, patternRequest

class LoopAndClean:
    def __init__(self):
        rospy.init_node('loop_and_clean')

        self.last_arrived_room = None
        self.room_name = None

        # Define predefined waypoints for rooms
        self.room_waypoints = [
            self.create_waypoint("Room 1 and 2", -4.0, 0.01, 0.0),
            self.create_waypoint("Room 3 and 4", 0.1, 0.01, 0.0),
            # Add more rooms as needed
        ]

        # Connect to move_base
        self.move_base = SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for the move_base action server to come up")
        self.move_base.wait_for_server()
        rospy.loginfo("Connected to move_base")

    def send_service_request(self, points):
        # Service request with the predefined points
        request = patternRequest()
        request.points = points
        #print(request.points)
        
        # Call the service 
        try:
            # Service client for pattern movement
            rospy.loginfo("Pattern Movement is running...")
            self.pattern_client = rospy.ServiceProxy('/pattern_movement', pattern)
            rospy.wait_for_service('/pattern_movement')
            response = self.pattern_client(request)
            rospy.loginfo("Service response: %s", response.result)
        
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", str(e))
    
    def predefined_points(self, room_name):
        rospy.loginfo("I am at %s.", str(room_name))
        if room_name == "Room 1 and 2":
            area_points = [
                PointStamped(header=rospy.Header(frame_id="map"), point=Point(x=-4.0, y=6.8, z=0)),
                PointStamped(header=rospy.Header(frame_id="map"), point=Point(x=-4.0, y=-2.0, z=0)),
                PointStamped(header=rospy.Header(frame_id="map"), point=Point(x=-2.7, y=-2.0, z=0)),
                PointStamped(header=rospy.Header(frame_id="map"), point=Point(x=-0.3, y=-2.3, z=0)),
                PointStamped(header=rospy.Header(frame_id="map"), point=Point(x=-0.25, y=6.8, z=0)),
            ]
            
        elif room_name == "Room 3 and 4":
            area_points = [
                PointStamped(header=rospy.Header(frame_id="map"), point=Point(x=0.78, y=1.95, z=0)),
                PointStamped(header=rospy.Header(frame_id="map"), point=Point(x=0.55, y=-6.76, z=0)),
                PointStamped(header=rospy.Header(frame_id="map"), point=Point(x=4.7, y=-6.74, z=0)),
                PointStamped(header=rospy.Header(frame_id="map"), point=Point(x=4.7, y=2.61, z=0)),
                PointStamped(header=rospy.Header(frame_id="map"), point=Point(x=2.36, y=1.84, z=0)),
                PointStamped(header=rospy.Header(frame_id="map"), point=Point(x=0.78, y=1.95, z=0)),
            ]

        else:
            rospy.logerr(f"Invalid room name: {room_name}")
            return[]
        return area_points

    def create_waypoint(self, name, x, y, theta):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.z = theta
        goal.target_pose.pose.orientation.w = 1.0
        return{"name": name, "goal": goal}
    
    def move(self, waypoint):
        rospy.loginfo(f"Sending goal: {waypoint['name']}")
        self.move_base.send_goal(waypoint["goal"])
        self.move_base.wait_for_result()
        result_state = self.move_base.get_state()
        rospy.loginfo(f"Result State: {result_state}")
        return result_state == 3

    def wait_for_input(self):
        choice = True
        while choice:
            user_input = input("Do you want to run Pattern in this room? Enter 'yes' to run, 'no' to skip and 'end' to finish : ")
            if user_input == "yes":
                rospy.loginfo ("Pattern service is calling")
                area_defined_points = self.predefined_points(self.room_name)
                #print(area_defined_points)
                self.send_service_request(area_defined_points)
                self.return_to_room_waypoint()
                choice = False
            elif user_input == "no":
                rospy.loginfo("Skipping and continuing to next room")
                choice = False
            elif user_input == "end":
                rospy.loginfo("Ending the process")
                sys.exit()
            else:
                rospy.loginfo("Invalid input. Please enter'yes', 'no', 'end'")
        #return user_input.lower() == 'yes'

    def return_to_room_waypoint(self):
        if self.last_arrived_room:
            rospy.loginfo(f"Returning to last arrived room: {self.last_arrived_room['name']}")
            success = self.move(self.last_arrived_room)
            if success:
                rospy.loginfo(f"Arrived back at {self.last_arrived_room['name']}.")
                self.service_count = 0
                self.send_waypoint()
            else:
                rospy.logwarn(f"Failed to return to {self.last_arrived_room['name']}.")

    def send_waypoint(self):
        try:
            while not rospy.is_shutdown():
                for waypoint in self.room_waypoints:
                    rospy.loginfo(f"Moving to {waypoint['name']}")
                    success = self.move(waypoint)
                    if success:
                        rospy.loginfo(f"Arrived at {waypoint['name']}.")
                        self.last_arrived_room = waypoint
                        self.room_name = waypoint['name']
                        choice = self.wait_for_input()
                        if choice == False:
                            continue
                    else:
                        rospy.logwarn(f"Failed to reach {waypoint['name']}.")
        except rospy.ROSInterruptException:
            pass

if __name__ == "__main__":
    roomlooping = LoopAndClean()
    roomlooping.send_waypoint()
    rospy.spin()